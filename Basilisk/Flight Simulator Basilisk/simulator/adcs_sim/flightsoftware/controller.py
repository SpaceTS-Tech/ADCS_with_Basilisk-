"""Basilisk ``SysModel`` wrapping the generated ADCS flight software.

Inventory items **C1** (the bridge itself), **C2** (the sensor/actuator
interface, see :mod:`adcs_sim.flightsoftware.structs`) and **C3** (defensive
clipping at the controller boundary).

The bridge behaves like real sensors and actuators: the flight software gets
the attitude quaternion of the body relative to inertial and the body angular
rate in body axes, and its torque outputs are taken as body-frame commands in
every mode.  No frame is converted on either side of the DLL.

Data flow, once per ``CONTROLLER_STEP_SECONDS``:

.. code-block:: text

    Basilisk messages  ->  sensor values, as measured   ->  ExtU struct
                                                                |
                                                          DLL step()
                                                                |
    Basilisk messages  <-  limits (body-frame commands) <-  ExtY struct
"""

import os
import time
from pathlib import Path

import numpy as np
from Basilisk.architecture import messaging, sysModel
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import macros

from adcs_sim.config import camera_feed as feed_cfg
from adcs_sim.config import gains as gains_cfg
from adcs_sim.config import simulation as sim_cfg
from adcs_sim.flightsoftware import structs
from adcs_sim.flightsoftware.camera_feed import (STAR_TRACKER_FEED_SHAPE,
                                                 CameraFeedReader)
from adcs_sim.flightsoftware.diagnostics import ControllerDiagnostics
from adcs_sim.flightsoftware.library import NullADCSLibrary, open_library
from adcs_sim.geometry import ground_target
from adcs_sim.models import magnetorquers, reaction_wheels, thrusters

#: Valid range of the generated model's mission command.
MISSION_CMD_RANGE = (0, 11)

#: Print a line of camera centroid/torque/rate diagnostics to stdout while a
#: camera feed is active.  Set ``ADCS_OPTICAL_DEBUG=1`` in the environment
#: before launching the scenario to turn this on; it costs nothing when unset.
#: Throttled by ``OPTICAL_DEBUG_INTERVAL_S`` so it stays readable at 20 Hz.  The
#: line includes the ACS mode number so optical-tracking steps can be picked out
#: without the bridge having to know the flight software's mode numbering.
OPTICAL_DEBUG = os.environ.get("ADCS_OPTICAL_DEBUG") == "1"
OPTICAL_DEBUG_INTERVAL_S = 0.5

#: Pixel value above which a pixel counts as target, matching the threshold in
#: the optical block's MATLAB Function (``u(u>5) = 255``).  Used only by the
#: ``ADCS_OPTICAL_DEBUG`` trace to recompute the centroid.
OPTICAL_PIXEL_THRESHOLD = 5.0


def optical_centroid(frame, width=structs.CAMERA_FEED_WIDTH,
                     height=structs.CAMERA_FEED_HEIGHT):
    """Return ``(x_cm, y_cm, valid)`` for a flat camera-feed buffer.

    Mirrors ``<S18>/MATLAB Function2`` (threshold to 255, column-major layout,
    intensity-weighted centroid minus half-extent) so the bridge-side law acts
    on exactly the quantity the block was meant to produce.

    ``x_cm`` is the column offset and ``y_cm`` the row offset, both in pixels
    from frame centre.  Per the measured image-axis regression documented in
    :mod:`adcs_sim.flightsoftware.structs`, increasing column is **+X_body**
    and increasing row is **+Y_body**.

    ``valid`` is False for a missing or wholly unlit frame.  The block divides
    by the total intensity unguarded, so a dark frame gives it ``0/0 = NaN``
    which propagates into ``RW_tau``; returning a flag instead keeps that out
    of the control path.
    """
    if frame is None:
        return 0.0, 0.0, False

    pixels = np.asarray(frame, dtype=np.float64).reshape((height, width),
                                                          order="F")
    thresholded = np.where(pixels > OPTICAL_PIXEL_THRESHOLD, 255.0, pixels)
    total = thresholded.sum()
    if total <= 0.0:
        return 0.0, 0.0, False

    col_index = np.arange(1, width + 1, dtype=np.float64)
    row_index = np.arange(1, height + 1, dtype=np.float64)
    x_cm = (thresholded * col_index[None, :]).sum() / total - width / 2.0
    y_cm = (thresholded * row_index[:, None]).sum() / total - height / 2.0
    return x_cm, y_cm, True


class GeneratedADCSController(sysModel.SysModel):
    """Marshal Basilisk messages into and out of the generated ADCS DLL.

    Parameters
    ----------
    library_path : str or pathlib.Path or None
        Path to ``ADCS_Software_v1.dll``.  ``None`` (or ``"none"``) runs with
        **no flight software**: every actuator command is zero, so no control
        torque or force acts on the spacecraft.
    wheel_inertia : float
        Wheel spin inertia handed to the controller [kg m^2].
    max_wheel_momentum : float
        ``h_max`` handed to the controller [N m s].
    mission_cmd : int, optional
        Mission command loaded at reset.
    record_history : bool, optional
        Pass False for indefinite live runs; see
        :class:`~adcs_sim.flightsoftware.diagnostics.ControllerDiagnostics`.
    star_tracker_camera : StarTrackerCamera, optional
        Simulated star tracker camera feeding ``STR 1``.  Given, its frames
        are used and Vizard's are not read; see
        :mod:`adcs_sim.config.star_tracker_camera`.  Register the camera in
        the task *before* this controller so each step sees that step's frame.
    """

    def __init__(self, library_path, wheel_inertia, max_wheel_momentum,
                 mission_cmd=sim_cfg.MISSION_CMD, record_history=True,
                 star_tracker_camera=None):
        super().__init__()
        self.ModelTag = "ADCS_Software_v1"

        self.library = open_library(library_path)
        self.diagnostics = ControllerDiagnostics(record_history=record_history)
        self.camera_feed = (CameraFeedReader()
                            if feed_cfg.ENABLE_CAMERA_FEED else None)
        #: ``STR 1`` source: the in-simulation camera when one is given,
        #: otherwise Vizard's rendered files (the old path).
        self.star_tracker_camera = (star_tracker_camera
                                    if feed_cfg.ENABLE_STAR_TRACKER_FEED
                                    else None)
        self.star_tracker_feed = (
            CameraFeedReader(
                camera_id=feed_cfg.STAR_TRACKER_FRAME_CAMERA_ID,
                poll_interval_s=feed_cfg.STAR_TRACKER_POLL_INTERVAL_S,
                max_frame_age_s=feed_cfg.STAR_TRACKER_MAX_FRAME_AGE_S,
                shape=STAR_TRACKER_FEED_SHAPE)
            if feed_cfg.ENABLE_STAR_TRACKER_FEED
            and self.star_tracker_camera is None else None)
        self._star_tracker_frames_fed = 0
        #: :class:`~adcs_sim.flightsoftware.link_runtime.LinkRuntime` when a
        #: connector link file drives the wiring; ``None`` = the hand-written
        #: mapping in ``_write_inputs`` / ``_publish``.  Set by the builder.
        self.link = None
        #: ``{dll_symbol: value}`` actually in force, filled at reset.
        self.applied_gains = {}

        self._wheel_inertia = wheel_inertia
        self._max_wheel_momentum = max_wheel_momentum
        #: Sim time [s] of the last optical-debug print; see OPTICAL_DEBUG.
        self._last_optical_debug_s = None
        #: ``(x_cm, y_cm, valid)`` from the most recent optical step.
        self._last_optical_centroid = (0.0, 0.0, False)
        # Validate here too, not just in set_mission_command: an out-of-range
        # value fits in the uint8 the DLL takes, so it raises no error and
        # simply leaves the controller in no recognised mode -- the spacecraft
        # then drifts uncontrolled, which is easy to mistake for a tuning
        # problem.
        self.current_mission_cmd = self._validated_mission_cmd(mission_cmd)

        # -- inputs --------------------------------------------------------
        self.attNavInMsg = messaging.NavAttMsgReader()
        self.transNavInMsg = messaging.NavTransMsgReader()
        self.rwSpeedsInMsg = messaging.RWSpeedMsgReader()
        self.magFieldInMsg = messaging.MagneticFieldMsgReader()
        self.sunStateInMsg = messaging.SpicePlanetStateMsgReader()
        self.earthStateInMsg = messaging.SpicePlanetStateMsgReader()

        # -- outputs -------------------------------------------------------
        self.rwMotorTorqueOutMsg = messaging.ArrayMotorTorqueMsg()
        self.mtbDipoleOutMsg = messaging.MTBCmdMsg()
        # CmdTorqueBodyMsg is used here only as a three-component voltage
        # telemetry container.  It is *not* connected to a torque effector.
        self.mtqVoltageOutMsg = messaging.CmdTorqueBodyMsg()
        self.thrTorqueOutMsg = messaging.CmdTorqueBodyMsg()

    # -- lifecycle ---------------------------------------------------------

    def Reset(self, current_sim_nanos):
        """Initialise and re-tune the generated model at simulation start."""
        del current_sim_nanos
        self.library.initialize()
        self.library.set_mission_cmd(self.current_mission_cmd)

        # Control-law gains and plant constants, from config.gains.  These are
        # exported globals rather than tunable parameters, which is why the
        # ADCS_set_rw_inertia wrapper call is a no-op on this build -- I_RW is
        # set here instead, along with every gain.
        self.applied_gains = self.library.apply_gains(gains_cfg.configured())

        self.library.set_h_max(self._max_wheel_momentum)

        configured_h_max = self.library.get_h_max()
        if not np.isclose(configured_h_max, self._max_wheel_momentum):
            raise RuntimeError(
                f"Failed to configure generated h_max: {configured_h_max}")

        self.library.set_ads_fine_available(1)
        self.diagnostics.reset()
        self._last_optical_debug_s = None
        self._last_optical_centroid = (0.0, 0.0, False)

        # Start both image inports from an explicitly empty frame rather than
        # whatever the DLL's static storage happens to hold.
        if self.camera_feed is not None:
            self.camera_feed.reset()
        self.library.clear_camera_feed()
        if self.star_tracker_feed is not None:
            self.star_tracker_feed.reset()
        self._star_tracker_frames_fed = 0
        self.library.clear_star_tracker_feed()
        self._open_truth_log()

    def _open_truth_log(self):
        """Start the per-step true-attitude CSV, if configured."""
        if getattr(self, "_truth_log", None) is not None:
            self._truth_log.close()
        self._truth_log = None
        path = feed_cfg.STAR_TRACKER_TRUTH_LOG
        if path is None or (self.star_tracker_feed is None
                            and self.star_tracker_camera is None):
            return
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        # Line-buffered so a live run killed with Q or Ctrl+C still leaves
        # every step written so far.
        self._truth_log = open(path, "w", buffering=1, encoding="ascii")
        # Wall-clock start, so frames from this run can be told apart from
        # older ones with the same sim-time file names.
        self._truth_log.write(f"# run_started_unix={time.time():.3f}\n")
        self._truth_log.write("t_s,q0,q1,q2,q3\n")

    def _log_truth(self, current_sim_nanos):
        """Append the ``q_BI`` just handed to the DLL to the truth CSV."""
        if getattr(self, "_truth_log", None) is None:
            return
        q = self.library.inputs.q_BI
        self._truth_log.write(
            f"{current_sim_nanos * macros.NANO2SEC:.6f},"
            f"{q[0]:.12f},{q[1]:.12f},{q[2]:.12f},{q[3]:.12f}\n")

    @staticmethod
    def _validated_mission_cmd(mission_cmd):
        """Return the command as an int, or raise if the DLL would ignore it."""
        mission_cmd = int(mission_cmd)
        low, high = MISSION_CMD_RANGE
        if not low <= mission_cmd <= high:
            raise ValueError(
                f"Mission command must be between {low} and {high}, got "
                f"{mission_cmd}. Out-of-range values are silently accepted by "
                "the DLL (they fit in its uint8 input) but match no mission "
                "mode, leaving the spacecraft uncontrolled.")
        return mission_cmd

    @property
    def has_flight_software(self):
        """False when running with no ADCS software (zero commands)."""
        return not isinstance(self.library, NullADCSLibrary)

    def describe_gains(self):
        """Return printable lines listing the gains actually in force.

        Read back from the DLL rather than echoed from config, so the banner
        reports what the flight software is really using.
        """
        if not self.has_flight_software:
            return ["  flight software: NONE -- no control torque or force "
                    "(RW, MTQ and thruster commands are zero)"]
        if not self.applied_gains:
            return ["  control gains: not applied yet (call Reset first)"]

        order = ["I_RW", "k_desat", "kp_point", "kd_point", "kp_hold",
                 "kd_hold", "kp_slew_fast", "kd_slew_fast", "kp_slew_slow",
                 "kd_slew_slow", "kp_spin", "kd_spin", "kd_detumble"]
        shown = [symbol for symbol in order if symbol in self.applied_gains]
        shown += [s for s in sorted(self.applied_gains) if s not in shown]

        lines = ["  control gains (read back from the DLL):"]
        for index in range(0, len(shown), 4):
            chunk = shown[index:index + 4]
            lines.append("    " + "  ".join(
                f"{symbol}={self.applied_gains[symbol]:g}" for symbol in chunk))
        return lines

    def set_mission_command(self, mission_cmd):
        """Change the generated controller's mission mode while running."""
        self.current_mission_cmd = self._validated_mission_cmd(mission_cmd)
        self.library.set_mission_cmd(self.current_mission_cmd)

    # -- per-step execution ------------------------------------------------

    def UpdateState(self, current_sim_nanos):
        """Read sensors, step the flight code, write actuator commands."""
        attitude = self.attNavInMsg()
        translation = self.transNavInMsg()
        wheels = self.rwSpeedsInMsg()
        magnetic = self.magFieldInMsg()
        sun = self.sunStateInMsg()
        earth = self.earthStateInMsg()

        if self.link is None:
            self._write_inputs(attitude, translation, wheels, magnetic, sun,
                               earth)
        else:
            self.link.write_software_inputs(self.library)
        self._log_truth(current_sim_nanos)
        self._write_camera_feed(current_sim_nanos)
        self._write_star_tracker_feed(current_sim_nanos)

        self.library.step()

        outputs = self.library.outputs
        block_signals = self.library.block_signals
        self.diagnostics.update(current_sim_nanos, outputs, block_signals)
        commands = self._read_outputs(outputs)
        if self.link is None:
            self._publish(commands, current_sim_nanos)
        else:
            # The link writes every actuator input; the MTQ voltage message
            # is telemetry only (nothing subscribes to it), kept for plots.
            self.link.publish(self.library, current_sim_nanos, self.moduleID,
                              zero=not self.has_flight_software)
            self._publish_mtq_voltage(commands, current_sim_nanos)

        if OPTICAL_DEBUG and self.camera_feed is not None:
            self._print_optical_debug(current_sim_nanos, attitude, commands,
                                      int(block_signals.actual_acs))

    # -- input marshalling -------------------------------------------------

    def _write_inputs(self, attitude, translation, wheels, magnetic, sun,
                      earth):
        """Populate the generated ``ExtU`` struct for this step.

        Every value is what the corresponding sensor would report; see
        :mod:`adcs_sim.flightsoftware.structs` for the interface definition.
        """
        inputs = self.library.inputs

        # Star tracker / estimator: attitude of the body relative to inertial,
        # Basilisk's q_BN (scalar first), passed through unchanged.
        q_BN = rbk.MRP2EP(attitude.sigma_BN)
        for index in range(4):
            inputs.q_BI[index] = q_BN[index]

        # Magnetometer: the field is measured in body axes.  Basilisk provides
        # it in inertial components, so this is the sensor model, not an
        # interface conversion.
        dcm_BN = rbk.MRP2C(attitude.sigma_BN)
        magnetic_field_B = dcm_BN @ np.asarray(magnetic.magField_N)

        # Gyro: body angular rate relative to inertial, in body axes, unchanged.
        omega_BN_B = np.asarray(attitude.omega_BN_B)

        for index in range(3):
            inputs.omega[index] = omega_BN_B[index]
            inputs.RW_omega[index] = wheels.wheelSpeeds[index]
            # With Earth configured as Basilisk's central body, these are the
            # requested Earth-centred inertial position and velocity vectors.
            inputs.r_sat_i[index] = translation.r_BN_N[index]
            inputs.v_sat_i[index] = translation.v_BN_N[index]
            inputs.B[index] = magnetic_field_B[index]
            # With the SPICE interface zero-based at Earth, these are the
            # Sun's Earth-centred J2000 / ECI position and velocity vectors.
            inputs.r_sun_i[index] = sun.PositionVector[index]
            inputs.v_sun_i[index] = sun.VelocityVector[index]

        target_position_N = ground_target.position_inertial(earth.J20002Pfix)
        for index in range(3):
            inputs.r_p_i[index] = target_position_N[index]

    def _write_camera_feed(self, current_sim_nanos):
        """Push the newest rendered frame into the ``Camera feed`` inport.

        The frame is written through the wrapper's ``memcpy`` entry point, not
        by ctypes element assignment -- 262144 writes per step is not viable.

        The inport carries no timestamp or validity flag, so the model cannot
        tell a fresh frame from a held-over one.  When the newest frame is
        older than ``MAX_FRAME_AGE_S`` the inport is blanked instead, so the
        absence of imagery is at least explicit.
        """
        self._write_image_feed(self.camera_feed, self.library.set_camera_feed,
                               self.library.clear_camera_feed,
                               current_sim_nanos)

    def _write_star_tracker_feed(self, current_sim_nanos):
        """Push the newest star tracker frame into the ``STR 1`` inport.

        From the in-simulation camera: copy each new frame once (one
        ``memcpy``); between frames the inport keeps the last one, as a real
        sensor interface would.  From Vizard: same handling as
        :meth:`_write_camera_feed`.
        """
        camera = self.star_tracker_camera
        if camera is not None:
            if camera.frame_count != self._star_tracker_frames_fed:
                self.library.set_star_tracker_feed(camera.feed_buffer())
                self._star_tracker_frames_fed = camera.frame_count
            return
        self._write_image_feed(self.star_tracker_feed,
                               self.library.set_star_tracker_feed,
                               self.library.clear_star_tracker_feed,
                               current_sim_nanos)

    @staticmethod
    def _write_image_feed(reader, set_frame, clear_frame, current_sim_nanos):
        """Copy a reader's newest frame into an inport, or blank it if stale."""
        if reader is None:
            return

        status = reader.update(current_sim_nanos * macros.NANO2SEC)
        frame = reader.frame()

        if frame is None or status.stale:
            if feed_cfg.CLEAR_WHEN_STALE:
                clear_frame()
            return

        if status.updated:
            set_frame(frame)

    # -- output marshalling ------------------------------------------------

    def _read_outputs(self, outputs):
        """Convert the generated ``ExtY`` struct into actuator commands.

        ``RW_tau`` and ``THR_tau`` are torques on the spacecraft in **body**
        axes in every ACS mode -- no rotation and no mode check.  A
        reaction-wheel *motor* torque produces the opposite torque on the
        spacecraft, hence the minus sign.  ``MTQ_tau`` is a per-rod voltage.
        """
        rw_torque_B = np.asarray(outputs.RW_tau, dtype=float)
        rw_motor_torque_B = reaction_wheels.limit_motor_torque(rw_torque_B)

        applied_mtq_voltage = magnetorquers.limit_voltage(
            np.asarray(outputs.MTQ_tau))
        mtb_dipole_cmd = magnetorquers.voltage_to_dipole(applied_mtq_voltage)

        thr_torque_B = thrusters.limit_body_torque(
            np.asarray(outputs.THR_tau, dtype=float))

        return {
            "rw_motor_torque_B": rw_motor_torque_B,
            "mtq_voltage": applied_mtq_voltage,
            "mtb_dipole": mtb_dipole_cmd,
            "thr_torque_B": thr_torque_B,
            # Debug-only: the generated RW_tau before the motor-torque sign
            # flip and clipping.
            "rw_torque_body": rw_torque_B,
        }

    def _publish(self, commands, current_sim_nanos):
        """Write the actuator command and telemetry messages."""
        # Assign the complete SWIG property in every case below.  Slice
        # assignment only mutates a temporary Python list and silently leaves
        # the message payload at zero.
        motor_torque = messaging.ArrayMotorTorqueMsgPayload()
        motor_torque.motorTorque = commands["rw_motor_torque_B"].tolist()
        self.rwMotorTorqueOutMsg.write(motor_torque, current_sim_nanos,
                                      self.moduleID)

        self._publish_mtq_voltage(commands, current_sim_nanos)

        dipole_command = messaging.MTBCmdMsgPayload()
        dipole_command.mtbDipoleCmds = commands["mtb_dipole"].tolist()
        self.mtbDipoleOutMsg.write(dipole_command, current_sim_nanos,
                                   self.moduleID)

        thruster_command = messaging.CmdTorqueBodyMsgPayload()
        thruster_command.torqueRequestBody = commands["thr_torque_B"].tolist()
        self.thrTorqueOutMsg.write(thruster_command, current_sim_nanos,
                                   self.moduleID)

    def _publish_mtq_voltage(self, commands, current_sim_nanos):
        """Write the MTQ voltage telemetry message."""
        voltage_telemetry = messaging.CmdTorqueBodyMsgPayload()
        voltage_telemetry.torqueRequestBody = commands["mtq_voltage"].tolist()
        self.mtqVoltageOutMsg.write(voltage_telemetry, current_sim_nanos,
                                    self.moduleID)

    # -- optical-mode diagnostics -------------------------------------------

    def _print_optical_debug(self, current_sim_nanos, attitude, commands,
                             acs_mode):
        """Throttled console line: centroid, body RW_tau and body rate.

        Purpose-built to answer "which way is it actually pushing": recomputes
        x_cm/y_cm from the same frame the DLL just consumed, and shows the
        body-frame torque alongside the body rate that fed the damping term.
        Enable with ``ADCS_OPTICAL_DEBUG=1``.
        """
        time_s = current_sim_nanos * macros.NANO2SEC
        if (self._last_optical_debug_s is not None
                and time_s - self._last_optical_debug_s < OPTICAL_DEBUG_INTERVAL_S):
            return
        self._last_optical_debug_s = time_s

        frame = (self.camera_feed.frame()
                 if self.camera_feed is not None else None)
        x_cm, y_cm, valid = optical_centroid(frame)
        self._last_optical_centroid = (x_cm, y_cm, valid)
        body = commands["rw_torque_body"]
        omega = np.asarray(attitude.omega_BN_B)

        def fmt(vec):
            return "(" + ", ".join(f"{v:+.4e}" for v in vec) + ")"

        cm_text = (f"x_cm={x_cm:+7.2f} px  y_cm={y_cm:+7.2f} px" if valid
                   else "no usable frame")
        print(f"t={time_s:8.2f} s  acs={acs_mode}  {cm_text}  "
              f"RW_tau_body={fmt(body)}  "
              f"omega_BN_B={fmt(omega)}")
