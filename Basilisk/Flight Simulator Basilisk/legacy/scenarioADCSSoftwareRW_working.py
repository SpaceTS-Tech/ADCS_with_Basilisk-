"""Run Basilisk with the generated ADCS_Software_v1 reaction-wheel controller.

The generated controller starts with mission command 4 (nadir pointing).
Use ``--mode live`` for an indefinite two-way Vizard socket stream, or
``--mode file`` for a finite run that writes ``test2.bin`` for later playback.
The reaction wheels start at their momentum limit by default; use
``--rw-omega-rpm`` or ``--rw-momentum`` to select another initial state.
"""

import argparse
import ctypes
import time
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from Basilisk.architecture import messaging, sysModel
from Basilisk.simulation import (MtbEffector, extForceTorque,
                                 magneticFieldCenteredDipole,
                                 reactionWheelStateEffector, simpleNav,
                                 simSynch, spacecraft, svIntegrators)
from Basilisk.utilities import (RigidBodyKinematics as rbk, SimulationBaseClass,
                                macros, orbitalMotion, simIncludeGravBody,
                                simHelpers, simIncludeRW,
                                simSetPlanetEnvironment, vizSupport)

MISSION_CMD = 4
SIMULATION_SECONDS = 5000.0
CONTROLLER_STEP_SECONDS = 0.05  # Generated model base sample time
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[2]  # legacy/ -> ADCS_software_test/ -> Basilisk/
ADCS_DIR = REPO_ROOT / "ADCS_Software_v1_grt_rtw"
ADCS_LIBRARY = ADCS_DIR / "ADCS_Software_v1.dll"
VIZ_FILE = SCRIPT_DIR / "test2.bin"
PLOT_SAMPLE_SECONDS = 1.0
CONTROLLER_I_RW = 0.07957747154594767  # Honeywell HR16 Js [kg m^2]
RW_PHYSICAL_H_MAX = 50.0               # Honeywell HR16 momentum limit [N m s]
CONTROLLER_H_MAX = 50.0                # ADCS h_max parameter [N m s]
RW_MAX_MOTOR_TORQUE = 0.2              # Honeywell HR16 u_max [N m]
# Temporary ideal-thruster torque limit. Replace this with the torque obtained
# from the flight thruster force and moment-arm geometry.
THR_MAX_BODY_TORQUE = 0.05             # Per body axis [N m]
DEFAULT_INITIAL_RW_MOMENTUM_NMS = np.full(3, RW_PHYSICAL_H_MAX)
# Replace these two ratings with the values from the flight torque-rod data
# sheet. The generated ADCS output is a voltage command; Basilisk's built-in
# MtbEffector requires a magnetic dipole command in A m^2.
MTQ_MAX_VOLTAGE = 5.0                  # Applied voltage saturation [V]
MTQ_MAX_DIPOLE = 0.2                   # Dipole at full voltage [A m^2]
MTQ_DIPOLE_PER_VOLT = MTQ_MAX_DIPOLE / MTQ_MAX_VOLTAGE
TARGET_LATITUDE_DEG = 28.522431782788708
TARGET_LONGITUDE_DEG = 77.02428245359896
TARGET_RADIUS_M = 6_378_137.0
SPICE_EPOCH = "2026 JULY 13 06:30:00.0"  # 12:00 IST
CONJUGATE_Q_BI = True
ANGULAR_RATE_SIGN = -1.0
RW_MOTOR_TORQUE_SIGN = 1.0
# Connect explicitly to Vizard's IPv4 loopback server on Windows.  This
# installed Basilisk build acts as the client; ``0.0.0.0`` is not a valid
# remote destination and ``localhost`` can resolve to the IPv6 address ::1.
VIZ_SOCKET_ADDRESS = "127.0.0.1"
VIZ_SOCKET_PORT = "5556"
LIVE_ACCELERATION_FACTOR = 20.0
INITIAL_TUMBLE_RATE_MAGNITUDE_RAD_S = 0
INITIAL_TUMBLE_RATE_B = (INITIAL_TUMBLE_RATE_MAGNITUDE_RAD_S
                         / np.sqrt(3.0) * np.ones(3))


def initial_rw_speed_rpm(initial_omega_rpm=None, initial_momentum_nms=None):
    """Return the three initial wheel speeds in the RPM expected by Basilisk."""
    if initial_omega_rpm is not None and initial_momentum_nms is not None:
        raise ValueError("Specify initial RW speed or momentum, not both.")

    if initial_omega_rpm is not None:
        omega_rpm = np.asarray(initial_omega_rpm, dtype=float)
    else:
        momentum_nms = (DEFAULT_INITIAL_RW_MOMENTUM_NMS
                        if initial_momentum_nms is None
                        else np.asarray(initial_momentum_nms, dtype=float))
        omega_rad_s = momentum_nms / CONTROLLER_I_RW
        omega_rpm = omega_rad_s / macros.RPM

    if omega_rpm.shape != (3,) or not np.all(np.isfinite(omega_rpm)):
        raise ValueError("Initial RW values must be three finite numbers.")
    return omega_rpm


def target_position_planet_fixed():
    """Return the fixed Indian ground target in Earth-fixed coordinates."""
    latitude = TARGET_LATITUDE_DEG * macros.D2R
    longitude = TARGET_LONGITUDE_DEG * macros.D2R
    cos_latitude = np.cos(latitude)
    return np.array((
        TARGET_RADIUS_M * cos_latitude * np.cos(longitude),
        TARGET_RADIUS_M * cos_latitude * np.sin(longitude),
        TARGET_RADIUS_M * np.sin(latitude),
    ))


def create_plots(attitude_log, trans_log, sun_log, earth_log,
                 rw_speed_log, rw_torque_log,
                 mtq_voltage_log, mtq_torque_log, thr_torque_log, controller,
                 show_plots=False):
    """Save spacecraft, wheel, and generated-controller output histories."""
    time_seconds = attitude_log.times() * macros.NANO2SEC
    sigma_BN = np.asarray(attitude_log.sigma_BN)
    q_BI = np.vstack([rbk.MRP2EP(sigma) for sigma in sigma_BN])

    plot_files = {
        "quaternion": SCRIPT_DIR / "quaternion.png",
        "angular_rate": SCRIPT_DIR / "angular_rate.png",
        "rw_angular_velocity": SCRIPT_DIR / "rw_angular_velocity.png",
        "mtq_voltage": SCRIPT_DIR / "mtq_voltage.png",
        "adcs_torques": SCRIPT_DIR / "adcs_torques.png",
        "rw_tau_after_200s": SCRIPT_DIR / "rw_tau_after_200s.png",
        "earth_pointing": SCRIPT_DIR / "earth_pointing.png",
        "sun_pointing": SCRIPT_DIR / "sun_pointing.png",
        "earth_target_pointing": SCRIPT_DIR / "earth_target_pointing.png",
        "target_sunlight": SCRIPT_DIR / "target_sunlight.png",
        "adcs_modes": SCRIPT_DIR / "adcs_modes.png",
    }

    fig, ax = plt.subplots()
    for index, label in enumerate((r"$q_0$", r"$q_1$", r"$q_2$", r"$q_3$")):
        ax.plot(time_seconds, q_BI[:, index], label=label)
    ax.set(xlabel="Time [s]", ylabel=r"Quaternion $q_{BI}$",
           title="Spacecraft attitude quaternion")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_files["quaternion"], dpi=200)

    fig, ax = plt.subplots()
    body_rates = np.asarray(attitude_log.omega_BN_B)
    body_rate_norm = np.linalg.norm(body_rates, axis=1)
    for index, label in enumerate((r"$\omega_x$", r"$\omega_y$", r"$\omega_z$")):
        ax.plot(time_seconds, body_rates[:, index], label=label)
    ax.set(xlabel="Time [s]", ylabel="Angular rate [rad/s]",
           title="Spacecraft body angular rate")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_files["angular_rate"], dpi=200)

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 7))
    rw_time_seconds = rw_speed_log.times() * macros.NANO2SEC
    wheel_speeds = np.asarray(rw_speed_log.wheelSpeeds)[:, :3]
    for index in range(3):
        axes[0].plot(rw_time_seconds, wheel_speeds[:, index], label=f"RW{index + 1}")
        axes[1].plot(rw_time_seconds,
                     wheel_speeds[:, index] - wheel_speeds[0, index],
                     label=f"RW{index + 1}")
    axes[0].set(ylabel="Angular velocity [rad/s]",
                title="Reaction-wheel angular velocities")
    axes[1].set(xlabel="Time [s]", ylabel=r"Change $\Delta\Omega$ [rad/s]")
    for ax in axes:
        ax.grid(True)
        ax.legend()
    fig.tight_layout()
    fig.savefig(plot_files["rw_angular_velocity"], dpi=200)

    voltage_time_seconds = mtq_voltage_log.times() * macros.NANO2SEC
    mtq_voltages = np.asarray(mtq_voltage_log.torqueRequestBody)
    fig, ax = plt.subplots(figsize=(9, 5))
    for index, component in enumerate(("x", "y", "z")):
        ax.plot(voltage_time_seconds, mtq_voltages[:, index], label=component)
    ax.axhline(MTQ_MAX_VOLTAGE, color="black", linestyle="--", linewidth=1,
               label="voltage limits")
    ax.axhline(-MTQ_MAX_VOLTAGE, color="black", linestyle="--", linewidth=1)
    ax.set(xlabel="Time [s]", ylabel="Applied voltage [V]",
           title="Magnetic torque-rod voltage commands")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_files["mtq_voltage"], dpi=200)

    torque_time_seconds = rw_torque_log.times() * macros.NANO2SEC
    torque_sets = (
        (np.asarray(rw_torque_log.motorTorque)[:, :3], r"RW_tau", "RW motor torque [N m]"),
        (np.asarray(mtq_torque_log.mtbNetTorque_B), r"MTQ_tau", "MTQ torque [N m]"),
        (np.asarray(thr_torque_log.torqueRequestBody), r"THR_tau", "Thruster torque [N m]"),
    )
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 9))
    component_labels = ("x", "y", "z")
    for ax, (torques, title, ylabel) in zip(axes, torque_sets):
        has_finite_sample = False
        nonfinite_count = np.count_nonzero(~np.isfinite(torques))
        for index, component in enumerate(component_labels):
            finite = np.isfinite(torques[:, index])
            if np.any(finite):
                has_finite_sample = True
                ax.plot(torque_time_seconds[finite], torques[finite, index],
                        label=component)
        ax.set(title=title, ylabel=ylabel)
        ax.grid(True)
        if has_finite_sample:
            ax.legend()
        if not has_finite_sample:
            ax.text(0.5, 0.5, "No finite samples (controller output is NaN)",
                    ha="center", va="center", transform=ax.transAxes,
                    color="crimson")
        elif nonfinite_count:
            ax.text(0.01, 0.95, f"{nonfinite_count} non-finite samples",
                    ha="left", va="top", transform=ax.transAxes,
                    color="crimson")
    axes[-1].set_xlabel("Time [s]")
    fig.tight_layout()
    fig.savefig(plot_files["adcs_torques"], dpi=200)

    rw_torques = torque_sets[0][0]
    post_200 = torque_time_seconds >= 200.0
    fig, ax = plt.subplots(figsize=(9, 5))
    for index, component in enumerate(component_labels):
        ax.plot(torque_time_seconds[post_200], rw_torques[post_200, index],
                label=component)
    ax.set(xlabel="Time [s]", ylabel="RW motor torque [N m]",
           title=r"RW_tau after 200 seconds")
    ax.ticklabel_format(axis="y", style="sci", scilimits=(0, 0))
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_files["rw_tau_after_200s"], dpi=200)

    positions_N = np.asarray(trans_log.r_BN_N)
    earth_direction_N = -positions_N / np.linalg.norm(positions_N, axis=1)[:, None]
    earth_direction_B = np.empty_like(earth_direction_N)
    for index, sigma in enumerate(sigma_BN):
        earth_direction_B[index] = rbk.MRP2C(sigma) @ earth_direction_N[index]

    fig, ax = plt.subplots(figsize=(9, 5))
    for index, component in enumerate(component_labels):
        ax.plot(time_seconds, earth_direction_B[:, index], label=f"Body {component}")
    ax.set(xlabel="Time [s]", ylabel="Earth-direction component in body frame",
           title="Earth-pointing alignment (ideal fixed body component = +/-1)")
    ax.set_ylim(-1.05, 1.05)
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_files["earth_pointing"], dpi=200)

    sun_positions_N = np.asarray(sun_log.PositionVector)
    sun_direction_N = sun_positions_N - positions_N
    sun_direction_N /= np.linalg.norm(sun_direction_N, axis=1)[:, None]
    sun_direction_B = np.empty_like(sun_direction_N)
    for index, sigma in enumerate(sigma_BN):
        sun_direction_B[index] = rbk.MRP2C(sigma) @ sun_direction_N[index]

    fig, ax = plt.subplots(figsize=(9, 5))
    for index, component in enumerate(component_labels):
        ax.plot(time_seconds, sun_direction_B[:, index], label=f"Body {component}")
    ax.set(xlabel="Time [s]", ylabel="Sun-direction component in body frame",
           title="Sun-pointing alignment (controller aligns body +Z with Sun)")
    ax.set_ylim(-1.05, 1.05)
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_files["sun_pointing"], dpi=200)

    target_position_P = target_position_planet_fixed()
    earth_dcm_PN = np.asarray(earth_log.J20002Pfix).reshape((-1, 3, 3))
    target_positions_N = np.einsum(
        "nij,j->ni", np.transpose(earth_dcm_PN, (0, 2, 1)), target_position_P)
    target_direction_N = target_positions_N - positions_N
    target_direction_N /= np.linalg.norm(target_direction_N, axis=1)[:, None]
    target_direction_B = np.empty_like(target_direction_N)
    for index, sigma in enumerate(sigma_BN):
        target_direction_B[index] = rbk.MRP2C(sigma) @ target_direction_N[index]

    fig, ax = plt.subplots(figsize=(9, 5))
    for index, component in enumerate(component_labels):
        ax.plot(time_seconds, target_direction_B[:, index],
                label=f"Body {component}")
    ax.set(xlabel="Time [s]", ylabel="Target-direction component in body frame",
           title="Earth-target alignment (controller aligns body +Z with target)")
    ax.set_ylim(-1.05, 1.05)
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_files["earth_target_pointing"], dpi=200)

    target_to_sun_N = sun_positions_N - target_positions_N
    target_to_sun_N /= np.linalg.norm(target_to_sun_N, axis=1)[:, None]
    target_normals_N = target_positions_N / np.linalg.norm(
        target_positions_N, axis=1)[:, None]
    solar_elevation_deg = np.arcsin(np.clip(
        np.sum(target_normals_N * target_to_sun_N, axis=1), -1.0, 1.0
    )) * macros.R2D
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(time_seconds, solar_elevation_deg)
    ax.axhline(0.0, color="black", linestyle="--", label="Horizon")
    ax.set(xlabel="Time [s]", ylabel="Solar elevation at target [deg]",
           title=f"Sunlight at Indian target ({TARGET_LATITUDE_DEG:.4f} deg N, "
                 f"{TARGET_LONGITUDE_DEG:.4f} deg E)")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_files["target_sunlight"], dpi=200)

    mode_time = np.asarray(controller.mode_time_seconds)
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
    axes[0].step(mode_time, controller.desired_acs_history, where="post",
                 label="desired ACS")
    axes[0].step(mode_time, controller.actual_acs_history, where="post",
                 label="actual ACS")
    axes[0].set_ylabel("ACS mode")
    axes[1].step(mode_time, controller.desired_frame_history, where="post",
                 label="desired frame")
    axes[1].step(mode_time, controller.actual_frame_history, where="post",
                 label="actual frame")
    axes[1].set_ylabel("Frame mode")
    axes[2].step(mode_time, controller.mode_degraded_history, where="post",
                 label="mode degraded")
    axes[2].step(mode_time, controller.desired_ads_history, where="post",
                 label="desired ADS")
    axes[2].step(mode_time, controller.actual_ads_history, where="post",
                 label="actual ADS")
    axes[2].set(xlabel="Time [s]", ylabel="Status")
    for ax in axes:
        ax.grid(True)
        ax.legend()
    fig.suptitle("Generated ADCS manager states")
    fig.tight_layout()
    fig.savefig(plot_files["adcs_modes"], dpi=200)

    print("\nSimulation telemetry summary")
    print(f"  Body-rate magnitude: initial={body_rate_norm[0]:.9e}, "
          f"at 200 s={body_rate_norm[np.searchsorted(time_seconds, 200.0)]:.9e}, "
          f"final={body_rate_norm[-1]:.9e} rad/s")
    initial_momentum_metric = CONTROLLER_I_RW * np.max(np.abs(wheel_speeds[0]))
    desat_entry_threshold = 0.95 * CONTROLLER_H_MAX
    desat_exit_threshold = 0.25 * CONTROLLER_H_MAX
    comparison = ">=" if initial_momentum_metric >= desat_entry_threshold else "<"
    print(f"  Controller saturation metric: {initial_momentum_metric:.6e} "
          f"{comparison} entry={desat_entry_threshold:.6e} N m s")
    print(f"  Controller h_max={CONTROLLER_H_MAX:.6e}, "
          f"DESAT exit threshold={desat_exit_threshold:.6e} N m s")
    for index in range(3):
        change = wheel_speeds[-1, index] - wheel_speeds[0, index]
        excursion = np.ptp(wheel_speeds[:, index])
        print(f"  RW{index + 1}: delta={change:+.6e} rad/s, "
              f"peak-to-peak={excursion:.6e} rad/s")
    for torques, title, _ in torque_sets:
        finite_values = torques[np.isfinite(torques)]
        nonfinite_count = np.count_nonzero(~np.isfinite(torques))
        if finite_values.size:
            print(f"  {title}: max(abs finite)={np.max(np.abs(finite_values)):.6e} "
                  f"N m, non-finite samples={nonfinite_count}")
        else:
            print(f"  {title}: no finite samples (all NaN)")
    print("  RW_tau statistics after 200 s:")
    for index, component in enumerate(component_labels):
        values = rw_torques[post_200, index]
        print(f"    {component}: max(abs)={np.max(np.abs(values)):.9e}, "
              f"RMS={np.sqrt(np.mean(values * values)):.9e}, "
              f"final={values[-1]:+.9e} N m")
    print("  Earth direction in body frame after 200 s (mean +/- max error):")
    for index, component in enumerate(component_labels):
        values = earth_direction_B[time_seconds >= 200.0, index]
        print(f"    {component}: mean={np.mean(values):+.9f}, "
              f"range=[{np.min(values):+.9f}, {np.max(values):+.9f}]")
    print("  Sun direction in body frame after 200 s:")
    for index, component in enumerate(component_labels):
        values = sun_direction_B[time_seconds >= 200.0, index]
        print(f"    {component}: mean={np.mean(values):+.9f}, "
              f"range=[{np.min(values):+.9f}, {np.max(values):+.9f}]")
    print("  Earth-target direction in body frame after 200 s:")
    for index, component in enumerate(component_labels):
        values = target_direction_B[time_seconds >= 200.0, index]
        print(f"    {component}: mean={np.mean(values):+.9f}, "
              f"range=[{np.min(values):+.9f}, {np.max(values):+.9f}]")
    print(f"  Target solar elevation: min={np.min(solar_elevation_deg):.6f} deg, "
          f"max={np.max(solar_elevation_deg):.6f} deg")

    if show_plots:
        plt.show()
    else:
        plt.close("all")
    return plot_files


class ADCSInputs(ctypes.Structure):
    """ctypes mirror of ExtU_ADCS_Software_v1_T."""

    _fields_ = [
        ("q_BI", ctypes.c_double * 4),
        ("omega", ctypes.c_double * 3),
        ("B", ctypes.c_double * 3),
        ("RW_omega", ctypes.c_double * 3),
        ("v_sat_i", ctypes.c_double * 3),
        ("r_sat_i", ctypes.c_double * 3),
        ("r_p_i", ctypes.c_double * 3),
        ("v_sun_i", ctypes.c_double * 3),
        ("r_sun_i", ctypes.c_double * 3),
    ]


class ADCSOutputs(ctypes.Structure):
    """ctypes mirror of ExtY_ADCS_Software_v1_T."""

    _fields_ = [
        ("actual_ephem", ctypes.c_uint8),
        ("actual_ads", ctypes.c_uint8),
        ("mode_degraded", ctypes.c_bool),
        ("RW_tau", ctypes.c_double * 3),
        ("THR_tau", ctypes.c_double * 3),
        ("MTQ_tau", ctypes.c_double * 3),
    ]


class ADCSBlockSignals(ctypes.Structure):
    """ctypes mirror of B_ADCS_Software_v1_T for mode diagnostics."""

    _fields_ = [
        ("desired_ephem", ctypes.c_uint8),
        ("desired_ads", ctypes.c_uint8),
        ("desired_frame", ctypes.c_uint8),
        ("desired_acs", ctypes.c_uint8),
        ("actual_frame", ctypes.c_uint8),
        ("actual_acs", ctypes.c_uint8),
        ("Compare", ctypes.c_bool),
        ("Compare_i", ctypes.c_bool),
        ("NOT", ctypes.c_bool),
    ]


class GeneratedADCSController(sysModel.SysModel):
    """Basilisk message adapter around the generated Simulink C controller."""

    def __init__(self, library_path, wheel_inertia, max_wheel_momentum):
        super().__init__()
        library_path = Path(library_path)
        if not library_path.is_file():
            raise FileNotFoundError(
                f"Generated ADCS library not found: {library_path}\n"
                "Build ADCS_Software_v1.dll from ADCS_Software_v1_grt_rtw first."
            )

        self.ModelTag = "ADCS_Software_v1"
        self._library = ctypes.CDLL(str(library_path))
        self._library.ADCS_set_mission_cmd.argtypes = [ctypes.c_uint8]
        self._library.ADCS_set_rw_inertia.argtypes = [ctypes.c_double]
        self._library.ADCS_set_h_max.argtypes = [ctypes.c_double]
        self._library.ADCS_get_h_max.restype = ctypes.c_double
        self._library.ADCS_get_rw_saturated.restype = ctypes.c_uint8
        self._library.ADCS_get_desat_complete.restype = ctypes.c_uint8
        self._library.ADCS_set_ads_fine_available.argtypes = [ctypes.c_uint8]
        self._inputs = ADCSInputs.in_dll(self._library, "ADCS_Software_v1_U")
        self._outputs = ADCSOutputs.in_dll(self._library, "ADCS_Software_v1_Y")
        self._block_signals = ADCSBlockSignals.in_dll(
            self._library, "ADCS_Software_v1_B")
        self._wheel_inertia = wheel_inertia
        self._max_wheel_momentum = max_wheel_momentum
        self.current_mission_cmd = MISSION_CMD
        self.record_diagnostics = True
        self.acs_state_counts = {}
        self.ephem_state_counts = {}
        self.max_rw_tau = 0.0
        self.mode_time_seconds = []
        self.actual_acs_history = []
        self.desired_acs_history = []
        self.actual_frame_history = []
        self.desired_frame_history = []
        self.actual_ads_history = []
        self.desired_ads_history = []
        self.mode_degraded_history = []

        self.attNavInMsg = messaging.NavAttMsgReader()
        self.transNavInMsg = messaging.NavTransMsgReader()
        self.rwSpeedsInMsg = messaging.RWSpeedMsgReader()
        self.magFieldInMsg = messaging.MagneticFieldMsgReader()
        self.sunStateInMsg = messaging.SpicePlanetStateMsgReader()
        self.earthStateInMsg = messaging.SpicePlanetStateMsgReader()
        self.rwMotorTorqueOutMsg = messaging.ArrayMotorTorqueMsg()
        self.mtbDipoleOutMsg = messaging.MTBCmdMsg()
        # CmdTorqueBodyMsg is used only as a three-component voltage telemetry
        # container. It is not connected to a dynamic torque effector.
        self.mtqVoltageOutMsg = messaging.CmdTorqueBodyMsg()
        self.thrTorqueOutMsg = messaging.CmdTorqueBodyMsg()

    def Reset(self, current_sim_nanos):
        del current_sim_nanos
        self._library.ADCS_Software_v1_initialize()
        self._library.ADCS_set_mission_cmd(self.current_mission_cmd)
        self._library.ADCS_set_rw_inertia(self._wheel_inertia)
        self._library.ADCS_set_h_max(self._max_wheel_momentum)
        configured_h_max = self._library.ADCS_get_h_max()
        if not np.isclose(configured_h_max, self._max_wheel_momentum):
            raise RuntimeError(
                f"Failed to configure generated h_max: {configured_h_max}")
        self._library.ADCS_set_ads_fine_available(1)
        self.acs_state_counts.clear()
        self.ephem_state_counts.clear()
        self.max_rw_tau = 0.0
        self.mode_time_seconds.clear()
        self.actual_acs_history.clear()
        self.desired_acs_history.clear()
        self.actual_frame_history.clear()
        self.desired_frame_history.clear()
        self.actual_ads_history.clear()
        self.desired_ads_history.clear()
        self.mode_degraded_history.clear()

    def set_mission_command(self, mission_cmd):
        """Change the generated controller mission mode while it is running."""
        mission_cmd = int(mission_cmd)
        if not 0 <= mission_cmd <= 11:
            raise ValueError("Mission command must be between 0 and 11")
        self.current_mission_cmd = mission_cmd
        self._library.ADCS_set_mission_cmd(mission_cmd)

    def UpdateState(self, current_sim_nanos):
        att = self.attNavInMsg()
        trans = self.transNavInMsg()
        rw = self.rwSpeedsInMsg()
        mag = self.magFieldInMsg()
        sun = self.sunStateInMsg()
        earth_state = self.earthStateInMsg()

        q_BI = rbk.MRP2EP(att.sigma_BN)
        if CONJUGATE_Q_BI:
            q_BI[1:] *= -1.0
        dcm_BN = rbk.MRP2C(att.sigma_BN)
        magnetic_field_B = dcm_BN @ np.asarray(mag.magField_N)
        # The generated LVLH block outputs -h/|r|^2 in ECI components and
        # subtracts it directly from this input.  Convert Basilisk's body-rate
        # measurement to ECI and reverse its sign to match that convention.
        omega_BN_N = dcm_BN.T @ np.asarray(att.omega_BN_B)
        for index in range(4):
            self._inputs.q_BI[index] = q_BI[index]
        for index in range(3):
            self._inputs.omega[index] = ANGULAR_RATE_SIGN * omega_BN_N[index]
            self._inputs.RW_omega[index] = rw.wheelSpeeds[index]
            # With Earth configured as Basilisk's central body, these are the
            # requested Earth-centered inertial position and velocity vectors.
            self._inputs.r_sat_i[index] = trans.r_BN_N[index]
            self._inputs.v_sat_i[index] = trans.v_BN_N[index]
            self._inputs.B[index] = magnetic_field_B[index]
            # With the SPICE interface zero-based at Earth, these are the
            # Sun's Earth-centered J2000/ECI position and velocity vectors.
            self._inputs.r_sun_i[index] = sun.PositionVector[index]
            self._inputs.v_sun_i[index] = sun.VelocityVector[index]

        dcm_PN = np.asarray(earth_state.J20002Pfix).reshape((3, 3))
        target_position_N = dcm_PN.T @ target_position_planet_fixed()
        for index in range(3):
            self._inputs.r_p_i[index] = target_position_N[index]
        self._library.ADCS_Software_v1_step()
        # The generated controller's attitude error and rate reference are in
        # ECI axes, so its torque vectors are ECI spacecraft torques.  Rotate
        # them into body axes.  A reaction-wheel motor torque produces the
        # opposite torque on the spacecraft, hence the additional minus sign.
        generated_rw_motor_torque_B = (
            RW_MOTOR_TORQUE_SIGN
            * (-dcm_BN @ np.asarray(self._outputs.RW_tau))
        )
        # Apply the same HR16 limit at the controller boundary as well as in
        # ReactionWheelStateEffector. This prevents an unstable/generated
        # command from entering the coupled spacecraft-wheel dynamics.
        rw_motor_torque_B = np.clip(
            np.nan_to_num(generated_rw_motor_torque_B, nan=0.0,
                          posinf=RW_MAX_MOTOR_TORQUE,
                          neginf=-RW_MAX_MOTOR_TORQUE),
            -RW_MAX_MOTOR_TORQUE, RW_MAX_MOTOR_TORQUE)
        generated_mtq_voltage = np.asarray(self._outputs.MTQ_tau)
        applied_mtq_voltage = np.clip(generated_mtq_voltage,
                                      -MTQ_MAX_VOLTAGE, MTQ_MAX_VOLTAGE)
        mtb_dipole_cmd = applied_mtq_voltage * MTQ_DIPOLE_PER_VOLT
        generated_thr_torque_B = dcm_BN @ np.asarray(self._outputs.THR_tau)
        # The generated DESAT law currently returns roughly -RW_omega, which
        # is about 628 N m at 6000 RPM. ExtForceTorque would apply that value
        # literally and destabilize the integration. Bound the ideal torque
        # to the available actuator authority until a physical thruster model
        # (force, location and firing logic) replaces this interface.
        thr_torque_B = np.clip(
            np.nan_to_num(generated_thr_torque_B, nan=0.0,
                          posinf=THR_MAX_BODY_TORQUE,
                          neginf=-THR_MAX_BODY_TORQUE),
            -THR_MAX_BODY_TORQUE, THR_MAX_BODY_TORQUE)
        actual_acs = int(self._block_signals.actual_acs)
        actual_ephem = int(self._outputs.actual_ephem)
        self.acs_state_counts[actual_acs] = self.acs_state_counts.get(actual_acs, 0) + 1
        self.ephem_state_counts[actual_ephem] = self.ephem_state_counts.get(actual_ephem, 0) + 1
        self.max_rw_tau = max(self.max_rw_tau,
                              max(abs(value) for value in self._outputs.RW_tau))
        if self.record_diagnostics:
            self.mode_time_seconds.append(current_sim_nanos * macros.NANO2SEC)
            self.actual_acs_history.append(actual_acs)
            self.desired_acs_history.append(int(self._block_signals.desired_acs))
            self.actual_frame_history.append(int(self._block_signals.actual_frame))
            self.desired_frame_history.append(int(self._block_signals.desired_frame))
            self.actual_ads_history.append(int(self._outputs.actual_ads))
            self.desired_ads_history.append(int(self._block_signals.desired_ads))
            self.mode_degraded_history.append(int(self._outputs.mode_degraded))

        command = messaging.ArrayMotorTorqueMsgPayload()
        # Assign the complete SWIG property. Slice assignment only mutates a
        # temporary Python list and silently leaves the message payload zero.
        command.motorTorque = rw_motor_torque_B.tolist()
        self.rwMotorTorqueOutMsg.write(command, current_sim_nanos, self.moduleID)

        voltage_telemetry = messaging.CmdTorqueBodyMsgPayload()
        voltage_telemetry.torqueRequestBody = applied_mtq_voltage.tolist()
        self.mtqVoltageOutMsg.write(voltage_telemetry, current_sim_nanos,
                                    self.moduleID)

        mtb_command = messaging.MTBCmdMsgPayload()
        mtb_command.mtbDipoleCmds = mtb_dipole_cmd.tolist()
        self.mtbDipoleOutMsg.write(mtb_command, current_sim_nanos,
                                   self.moduleID)

        thr_command = messaging.CmdTorqueBodyMsgPayload()
        thr_command.torqueRequestBody = thr_torque_B.tolist()
        self.thrTorqueOutMsg.write(thr_command, current_sim_nanos, self.moduleID)


def run(show_plots=False, live_stream=False, initial_omega_rpm=None,
        initial_momentum_nms=None):
    """Run either a finite recorded simulation or an indefinite live stream."""
    rw_omega_rpm = initial_rw_speed_rpm(initial_omega_rpm,
                                        initial_momentum_nms)
    rw_momentum_nms = CONTROLLER_I_RW * rw_omega_rpm * macros.RPM
    print("Initial reaction-wheel state:")
    for index, (omega, momentum) in enumerate(zip(rw_omega_rpm,
                                                  rw_momentum_nms), start=1):
        saturation = "SATURATED" if abs(momentum) >= RW_PHYSICAL_H_MAX else ""
        print(f"  RW{index}: {omega:+.6f} RPM, "
              f"h={momentum:+.6f} N m s {saturation}".rstrip())

    task_name = "simTask"
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("simProcess")
    process.addTask(sim.CreateNewTask(task_name, macros.sec2nano(CONTROLLER_STEP_SECONDS)))

    sc = spacecraft.Spacecraft()
    sc.ModelTag = "bsk-Sat"
    # Satellite inertia P.I0 supplied by the vehicle model: L/2 [kg m^2].
    inertia = (np.array([
        202.6910, 0.6765, 0.2248,
        0.6765, 115.7325, -16.5856,
        0.2248, -16.5856, 161.4387,
    ]) / 2.0).tolist()
    sc.hub.mHub = 750.0
    sc.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(inertia)
    # The long DESAT-to-pointing maneuver includes actuator saturation and
    # controller mode changes. Use an adaptive integrator so rejected trial
    # stages cannot create enormous, nonphysical RW accelerations inside a
    # fixed 0.05 s RK4 step. The controller still executes at exactly 0.05 s.
    integrator = svIntegrators.svIntegratorRKF45(sc)
    integrator.relTol = 1.0e-8
    integrator.absTol = 1.0e-10
    sc.setIntegrator(integrator)
    sim.AddModelToTask(task_name, sc, 1)

    grav_factory = simIncludeGravBody.gravBodyFactory()
    grav_bodies = grav_factory.createBodies("earth", "sun")
    earth = grav_bodies["earth"]
    earth.isCentralBody = True
    spice_object = grav_factory.createSpiceInterface(
        time=SPICE_EPOCH, epochInMsg=True)
    spice_object.zeroBase = "Earth"
    sim.AddModelToTask(task_name, spice_object, 3)
    grav_factory.addBodiesTo(sc)

    # The generated ADCS provides torque-rod voltages. MtbEffector applies the
    # corresponding physical m x B torque after voltage-to-dipole conversion.
    mtb_effector = MtbEffector.MtbEffector()
    mtb_effector.ModelTag = "MTQ_effector"
    sc.addDynamicEffector(mtb_effector)
    sim.AddModelToTask(task_name, mtb_effector)

    mtb_config = messaging.MTBArrayConfigMsgPayload()
    mtb_config.numMTB = 3
    mtb_config.GtMatrix_B = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ]
    mtb_config.maxMtbDipoles = [MTQ_MAX_DIPOLE] * 3
    mtb_config_msg = messaging.MTBArrayConfigMsg().write(mtb_config)

    thr_effector = extForceTorque.ExtForceTorque()
    thr_effector.ModelTag = "Thruster_torque_effector"
    sc.addDynamicEffector(thr_effector)
    sim.AddModelToTask(task_name, thr_effector)

    magnetic_field = magneticFieldCenteredDipole.MagneticFieldCenteredDipole()
    magnetic_field.ModelTag = "CenteredDipoleMagneticField"
    magnetic_field.addSpacecraftToModel(sc.scStateOutMsg)
    simSetPlanetEnvironment.centeredDipoleMagField(magnetic_field, "earth")
    sim.AddModelToTask(task_name, magnetic_field)

    rw_factory = simIncludeRW.rwFactory()
    rw_factory.create("Honeywell_HR16", [1, 0, 0],
                      maxMomentum=RW_PHYSICAL_H_MAX,
                      Omega=rw_omega_rpm[0], useMaxTorque=True,
                      u_max=RW_MAX_MOTOR_TORQUE)
    rw_factory.create("Honeywell_HR16", [0, 1, 0],
                      maxMomentum=RW_PHYSICAL_H_MAX,
                      Omega=rw_omega_rpm[1], useMaxTorque=True,
                      u_max=RW_MAX_MOTOR_TORQUE)
    rw_factory.create("Honeywell_HR16", [0, 0, 1],
                      maxMomentum=RW_PHYSICAL_H_MAX,
                      Omega=rw_omega_rpm[2], useMaxTorque=True,
                      u_max=RW_MAX_MOTOR_TORQUE)
    rw_effector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rw_effector.ModelTag = "RW_cluster"
    rw_factory.addToSpacecraft(sc.ModelTag, rw_effector, sc)
    sim.AddModelToTask(task_name, rw_effector, 2)

    nav = simpleNav.SimpleNav()
    nav.ModelTag = "SimpleNavigation"
    sim.AddModelToTask(task_name, nav)

    controller = GeneratedADCSController(ADCS_LIBRARY, CONTROLLER_I_RW,
                                         CONTROLLER_H_MAX)
    controller.record_diagnostics = not live_stream
    sim.AddModelToTask(task_name, controller)

    oe = orbitalMotion.ClassicElements()
    oe.a = 10_000_000.0
    oe.e = 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    r_N, v_N = orbitalMotion.elem2rv(earth.mu, oe)
    sc.hub.r_CN_NInit = r_N
    sc.hub.v_CN_NInit = v_N
    r_hat_N = r_N / np.linalg.norm(r_N)
    h_N = np.cross(r_N, v_N)
    h_hat_N = h_N / np.linalg.norm(h_N)
    z_hat_N = -r_hat_N
    y_hat_N = -h_hat_N
    x_hat_N = np.cross(y_hat_N, z_hat_N)
    dcm_BN_initial = np.vstack((x_hat_N, y_hat_N, z_hat_N))
    sigma_BN_initial = rbk.C2MRP(dcm_BN_initial)
    omega_LVLH_N = h_N / np.dot(r_N, r_N)
    omega_LVLH_B = dcm_BN_initial @ omega_LVLH_N
    sc.hub.sigma_BNInit = sigma_BN_initial.reshape(3, 1)
    # Three-axis tumble with exactly 10 rad/s total angular-rate magnitude.
    sc.hub.omega_BN_BInit = INITIAL_TUMBLE_RATE_B.reshape(3, 1)

    nav.scStateInMsg.subscribeTo(sc.scStateOutMsg)
    controller.attNavInMsg.subscribeTo(nav.attOutMsg)
    controller.transNavInMsg.subscribeTo(nav.transOutMsg)
    controller.rwSpeedsInMsg.subscribeTo(rw_effector.rwSpeedOutMsg)
    controller.magFieldInMsg.subscribeTo(magnetic_field.envOutMsgs[0])
    # createBodies("earth", "sun") preserves this order in the SPICE output.
    controller.sunStateInMsg.subscribeTo(spice_object.planetStateOutMsgs[1])
    controller.earthStateInMsg.subscribeTo(spice_object.planetStateOutMsgs[0])
    rw_effector.rwMotorCmdInMsg.subscribeTo(controller.rwMotorTorqueOutMsg)
    mtb_effector.mtbCmdInMsg.subscribeTo(controller.mtbDipoleOutMsg)
    mtb_effector.mtbParamsInMsg.subscribeTo(mtb_config_msg)
    mtb_effector.magInMsg.subscribeTo(magnetic_field.envOutMsgs[0])
    thr_effector.cmdTorqueInMsg.subscribeTo(controller.thrTorqueOutMsg)

    if not live_stream:
        sampling_time = macros.sec2nano(PLOT_SAMPLE_SECONDS)
        actuator_sampling_time = macros.sec2nano(CONTROLLER_STEP_SECONDS)
        attitude_log = nav.attOutMsg.recorder(sampling_time)
        trans_log = nav.transOutMsg.recorder(sampling_time)
        sun_log = spice_object.planetStateOutMsgs[1].recorder(sampling_time)
        earth_log = spice_object.planetStateOutMsgs[0].recorder(sampling_time)
        rw_speed_log = rw_effector.rwSpeedOutMsg.recorder(sampling_time)
        rw_torque_log = controller.rwMotorTorqueOutMsg.recorder(
            actuator_sampling_time)
        mtq_voltage_log = controller.mtqVoltageOutMsg.recorder(
            actuator_sampling_time)
        mtq_torque_log = mtb_effector.mtbOutMsg.recorder(
            actuator_sampling_time)
        thr_torque_log = controller.thrTorqueOutMsg.recorder(
            actuator_sampling_time)
        for recorder in (attitude_log, trans_log, sun_log, earth_log,
                         rw_speed_log, rw_torque_log, mtq_voltage_log,
                         mtq_torque_log, thr_torque_log):
            sim.AddModelToTask(task_name, recorder)

    if not vizSupport.vizFound:
        raise RuntimeError("Basilisk was built without Vizard support; test2.bin cannot be created.")
    if live_stream:
        clock_sync = simSynch.ClockSynch()
        clock_sync.accelFactor = LIVE_ACCELERATION_FACTOR
        sim.AddModelToTask(task_name, clock_sync)

    viz = vizSupport.enableUnityVisualization(
        sim, task_name, sc,
        saveFile=None if live_stream else str(VIZ_FILE),
        rwEffectorList=rw_effector,
        liveStream=live_stream,
    )
    if live_stream:
        viz.settings.keyboardLiveInput = "0123456789abpq"
        viz.reqComProtocol = "tcp"
        viz.reqComAddress = VIZ_SOCKET_ADDRESS
        viz.reqPortNumber = VIZ_SOCKET_PORT

    sim.InitializeSimulation()
    if live_stream:
        step_nanos = macros.sec2nano(CONTROLLER_STEP_SECONDS)
        stop_time = 0
        previous_keys = ""
        mission_keys = {str(value): value for value in range(10)}
        mission_keys.update({"a": 10, "b": 11})
        print(f"Live Vizard stream connected on tcp://{VIZ_SOCKET_ADDRESS}:"
              f"{VIZ_SOCKET_PORT}")
        print("Mission keys: 0-9 => commands 0-9, A => 10, B => 11")
        print("Controls: P => pause/resume, Q => quit")
        print(f"Starting mission command {controller.current_mission_cmd}")
        try:
            while True:
                if vizSupport.pauseFlag:
                    viz.UpdateState(stop_time)
                    clock_sync.Reset(0)
                    time.sleep(0.02)
                else:
                    stop_time += step_nanos
                    sim.ConfigureStopTime(stop_time)
                    sim.ExecuteSimulation()

                keys = viz.userInputMsg.read().keyboardInput.lower()
                new_keys = set(keys) - set(previous_keys)
                for key in new_keys:
                    if key in mission_keys:
                        controller.set_mission_command(mission_keys[key])
                        print(f"t={stop_time * macros.NANO2SEC:.2f} s: "
                              f"mission command -> {mission_keys[key]}")
                    elif key == "p":
                        vizSupport.pauseFlag = not vizSupport.pauseFlag
                        print("Simulation paused" if vizSupport.pauseFlag
                              else "Simulation resumed")
                    elif key == "q":
                        raise KeyboardInterrupt
                previous_keys = keys
        except KeyboardInterrupt:
            print("Stopping live simulation.")
            viz.liveSettings.terminateVizard = True
            viz.UpdateState(stop_time)
        return

    sim.ConfigureStopTime(macros.sec2nano(SIMULATION_SECONDS))
    sim.ExecuteSimulation()
    print(f"Generated ACS state counts: {controller.acs_state_counts}")
    print(f"Generated ephemeris state counts: {controller.ephem_state_counts}")
    print(f"Generated peak RW_tau at 0.05 s rate: {controller.max_rw_tau:.6e} N m")
    plot_files = create_plots(attitude_log, trans_log, sun_log, earth_log,
                              rw_speed_log, rw_torque_log,
                              mtq_voltage_log, mtq_torque_log, thr_torque_log,
                              controller, show_plots)
    print(f"Vizard data written to {VIZ_FILE}")
    for plot_file in plot_files.values():
        print(f"Plot written to {plot_file}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Run the generated ADCS controller in Basilisk.")
    parser.add_argument(
        "--mode", choices=("file", "live"), default="file",
        help=(f"file: run for {SIMULATION_SECONDS:g} s and generate test2.bin "
              "(default); live: stream indefinitely to Vizard over the socket"))
    parser.add_argument(
        "--show-plots", action="store_true",
        help="display plots after file mode completes")
    initial_rw_group = parser.add_mutually_exclusive_group()
    initial_rw_group.add_argument(
        "--rw-omega-rpm", nargs=3, type=float, metavar=("RW1", "RW2", "RW3"),
        help=("initial RW1/RW2/RW3 speeds in RPM; default is 6000 RPM "
              "on each wheel (the 50 N m s saturation limit)"))
    initial_rw_group.add_argument(
        "--rw-momentum", nargs=3, type=float, metavar=("RW1", "RW2", "RW3"),
        help=("initial RW1/RW2/RW3 momenta in N m s; default is "
              "50 N m s on each wheel"))
    arguments = parser.parse_args()
    run(show_plots=arguments.show_plots,
        live_stream=arguments.mode == "live",
        initial_omega_rpm=arguments.rw_omega_rpm,
        initial_momentum_nms=arguments.rw_momentum)
