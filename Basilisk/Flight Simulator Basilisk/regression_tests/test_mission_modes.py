"""Mission-mode regression scenarios for the generated ADCS controller.

Runs each mission command as an independent headless simulation, then checks
quantitative criteria against the recorded telemetry.  Nothing here touches
Vizard, so the whole suite is deterministic and runnable in CI.

Modes covered
-------------
=====  ==========================  =====================================
Cmd    Mode                        Expected ACS state
=====  ==========================  =====================================
1      ``M_DETUMBLE``              1 ``ACS_DETUMBLE``
4      ``M_POINT_NADIR``           2 ``ACS_POINT``   (frame LVLH)
5      ``M_POINT_SUN``             2 ``ACS_POINT``   (frame INERTIAL)
6      ``M_POINT_EARTH_TARGET``    2 ``ACS_POINT``   (frame EARTH_TARGET)
9      ``M_INTERSAT_TRACK``        8 ``ACS_OPTICAL`` (frame INERTIAL)
=====  ==========================  =====================================

The mapping above was read out of the generated ``ADCS_Software_v1.c``
mission dispatch, **not** from ``v1/build_adcs_fsm.m`` -- that file is stale
(it still places ``M_POINT_MOON`` at command 6 and spins about LVLH).

Saturated-start cases
---------------------
Every mode is also run with the wheels started at their momentum limit.  The
controller must then enter ``ACS_DESAT`` first, shed momentum, and only
afterwards settle into the commanded mode.  That ordering is the actual
assertion -- see :func:`check_desat_then_mode`.

Command 9 and the camera
------------------------
Optical tracking consumes rendered frames, which normally come from Vizard.
For a headless deterministic test :class:`SyntheticCameraFeed` substitutes an
analytically projected blob at the companion satellite's true line of sight.

.. warning::
   That exercises the flight software's centroid and control law, and the
   bridge that feeds them, but **not** Vizard's renderer or the PNG
   discovery/decode path in :mod:`adcs_sim.flightsoftware.camera_feed`.  A
   green command-9 case here does not prove the real imaging chain works; run
   ``scenarioADCSSoftwareRW.py --mode live`` for that.

Usage
-----
.. code-block:: text

    python test_mission_modes.py                 # every case
    python test_mission_modes.py --cases 1 4     # only these mission commands
    python test_mission_modes.py --no-desat      # skip the saturated-start set
    python test_mission_modes.py --verbose       # per-case metric detail

Exit status is non-zero if any case fails, so it can gate a build.
"""

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

#: ``adcs_sim`` lives beside the scenario entry point in ``../simulator``.
_SCENARIO_DIR = Path(__file__).resolve().parents[1] / "simulator"
if str(_SCENARIO_DIR) not in sys.path:
    sys.path.insert(0, str(_SCENARIO_DIR))

from Basilisk.utilities import RigidBodyKinematics as rbk  # noqa: E402
from Basilisk.utilities import macros  # noqa: E402

from adcs_sim.analysis.derived import derive  # noqa: E402
from adcs_sim.config import actuators as act_cfg  # noqa: E402
from adcs_sim.config import simulation as sim_cfg  # noqa: E402
from adcs_sim.config import visualization as viz_cfg  # noqa: E402
from adcs_sim.flightsoftware import structs  # noqa: E402
from adcs_sim.flightsoftware.camera_feed import FrameStatus  # noqa: E402
from adcs_sim.models import reaction_wheels  # noqa: E402
from adcs_sim.scenario import builder, recorders  # noqa: E402

# ---------------------------------------------------------------------------
# Generated-controller ACS state numbering
# ---------------------------------------------------------------------------
ACS_IDLE = 0
ACS_DETUMBLE = 1
ACS_POINT = 2
ACS_HOLD = 3
ACS_SLEW_SLOW = 4
ACS_SLEW_FAST = 5
ACS_SPIN = 6
ACS_DESAT = 7
ACS_OPTICAL = 8
ACS_SAFE = 9

ACS_NAMES = {
    ACS_IDLE: "IDLE", ACS_DETUMBLE: "DETUMBLE", ACS_POINT: "POINT",
    ACS_HOLD: "HOLD", ACS_SLEW_SLOW: "SLEW_SLOW", ACS_SLEW_FAST: "SLEW_FAST",
    ACS_SPIN: "SPIN", ACS_DESAT: "DESAT", ACS_OPTICAL: "OPTICAL",
    ACS_SAFE: "SAFE",
}

#: Payload boresight in body axes.  ``plots.py`` documents the pointing modes
#: as aligning body +Z with the Sun / the ground target.
BORESIGHT_B = np.array([0.0, 0.0, 1.0])

#: Per-wheel momentum that starts a run inside the desaturation band.
#: ``DESAT_ENTRY_FRACTION`` is 0.95 of ``CONTROLLER_H_MAX``.
SATURATED_MOMENTUM_NMS = act_cfg.CONTROLLER_H_MAX

#: Centroid radius the optical loop must hold the target inside [px].
#: 25 px is 2.0 deg at the 703 px/rad focal length.
OPTICAL_HOLD_TOLERANCE_PX = 25.0

#: Centroid radius that counts as losing control, well inside the 256 px
#: half-frame so a divergence is caught before the target leaves the image.
OPTICAL_MAX_EXCURSION_PX = 150.0


# ---------------------------------------------------------------------------
# Synthetic camera feed (command 9, headless)
# ---------------------------------------------------------------------------
class SyntheticCameraFeed:
    """Render the companion satellite analytically instead of via Vizard.

    Duck-types :class:`~adcs_sim.flightsoftware.camera_feed.CameraFeedReader`
    -- ``reset``/``update``/``frame``/``describe`` -- so the controller uses it
    without modification.

    The projection matches the pinhole model measured in
    :mod:`adcs_sim.flightsoftware.structs`: increasing **column** is ``+X_body``
    and increasing **row** is ``+Y_body``, with focal length
    ``f = (width/2) / tan(FOV/2)``.  For the configured 40 deg / 512 px that is
    703.4 px/rad, against the 705.8 / 665.0 px/rad measured from real frames.
    """

    def __init__(self, bundle, blob_radius_px=6, fov_deg=None):
        self._sc_msg = bundle.spacecraft.scStateOutMsg
        if bundle.companion is None:
            raise ValueError(
                "Command 9 needs the companion satellite; it is disabled in "
                "adcs_sim/config/companion.py.")
        self._target_msg = bundle.companion.spacecraft.scStateOutMsg

        self.width = structs.CAMERA_FEED_WIDTH
        self.height = structs.CAMERA_FEED_HEIGHT
        fov_deg = (viz_cfg.INSTRUMENT_CAMERA_FIELD_OF_VIEW_DEG
                   if fov_deg is None else fov_deg)
        self.focal_px = (self.width / 2.0) / np.tan(np.radians(fov_deg) / 2.0)
        self.blob_radius_px = int(blob_radius_px)
        self.reset()

    # -- CameraFeedReader interface ---------------------------------------

    def reset(self):
        self._frame = None
        self.frames_loaded = 0
        self.decode_failures = 0
        #: ``(row, col)`` of the last rendered blob, or ``None`` if off-frame.
        self.last_pixel = None
        #: ``(time_s, row, col)`` per step; ``row``/``col`` are NaN off-frame.
        #: This is the independent evidence that the optical loop converges --
        #: it is the true projected position, not anything the control law
        #: computed, so a sign error shows up as a growing offset rather than
        #: being masked by the law under test.
        self.pixel_history = []

    def update(self, time_s):
        self._frame, self.last_pixel = self._render()
        self.frames_loaded += 1
        if self.last_pixel is None:
            self.pixel_history.append((time_s, np.nan, np.nan))
        else:
            self.pixel_history.append(
                (time_s, self.last_pixel[0], self.last_pixel[1]))
        return FrameStatus(updated=True, stale=False, path="<synthetic>",
                           age_s=0.0)

    def centroid_offsets(self):
        """Return ``(t, x_cm, y_cm)`` arrays in pixels from frame centre."""
        if not self.pixel_history:
            return (np.empty(0),) * 3
        history = np.asarray(self.pixel_history, dtype=float)
        time_s = history[:, 0]
        y_cm = history[:, 1] - self.height / 2.0
        x_cm = history[:, 2] - self.width / 2.0
        return time_s, x_cm, y_cm

    def frame(self):
        return self._frame

    def describe(self):
        return [f"  camera feed: SYNTHETIC ({self.frames_loaded} frames, "
                f"f={self.focal_px:.1f} px/rad, blob r={self.blob_radius_px})"]

    # -- projection --------------------------------------------------------

    def line_of_sight_B(self):
        """Companion direction in body axes, un-normalised.

        .. note::
           The companion is registered **after** the controller in the task
           (``builder.build_simulation`` does this deliberately to preserve the
           established model order), so reading its message from here returns a
           position that is one controller step stale.  At 6.3 km/s that lag is
           ~316 m -- five times the 60 m separation being measured, and on its
           own enough to place the target behind the camera from the second
           step onward.

           Propagating the companion forward by one step removes it.  The
           residual from neglecting orbital acceleration over 0.05 s is about
           5 mm, four orders below the separation.

           This matters beyond the test: any onboard code that reads the
           companion state message from the controller sees the same lag.
        """
        own = self._sc_msg.read()
        target = self._target_msg.read()
        lag_s = (0.0 if self.frames_loaded == 0
                 else sim_cfg.CONTROLLER_STEP_SECONDS)
        r_target_N = (np.asarray(target.r_BN_N)
                      + np.asarray(target.v_BN_N) * lag_s)
        separation_N = r_target_N - np.asarray(own.r_BN_N)
        dcm_BN = rbk.MRP2C(np.asarray(own.sigma_BN))
        return dcm_BN @ separation_N

    def _render(self):
        """Return ``(flat_frame, (row, col))``; blank frame if out of view."""
        blank = np.zeros(self.width * self.height, dtype=np.uint8)
        los_B = self.line_of_sight_B()
        if los_B[2] <= 0.0:            # behind the camera
            return blank, None

        column = self.width / 2.0 + 0.5 + self.focal_px * los_B[0] / los_B[2]
        row = self.height / 2.0 + 0.5 + self.focal_px * los_B[1] / los_B[2]
        row_i, col_i = int(round(row)), int(round(column))
        radius = self.blob_radius_px
        if not (radius <= row_i < self.height - radius
                and radius <= col_i < self.width - radius):
            return blank, None         # outside the field of view

        image = np.zeros((self.height, self.width), dtype=np.uint8)
        image[row_i - radius:row_i + radius + 1,
              col_i - radius:col_i + radius + 1] = 255
        return np.ascontiguousarray(image.flatten(order="F")), (row, column)


# ---------------------------------------------------------------------------
# Case definition
# ---------------------------------------------------------------------------
@dataclass
class MissionCase:
    """One mission command run under one set of initial conditions."""

    name: str
    mission_cmd: int
    seconds: float
    expect_acs: int
    #: Body rate at t=0 [rad/s].
    initial_rate_rad_s: tuple = (0.0, 0.0, 0.0)
    #: Per-wheel momentum at t=0 [N m s].
    initial_momentum_nms: float = 0.0
    #: Direction the +Z boresight should acquire, for reporting.
    pointing_target: str = None
    #: Treat the boresight angle as pass/fail rather than informational.
    assert_pointing: bool = False
    max_pointing_error_deg: float = 15.0
    needs_camera: bool = False
    #: Rotate the initial attitude off its nominal RAM alignment [deg], about
    #: ``offset_axis`` (1=X, 2=Y, 3=Z).  Used to start the optical case
    #: off-boresight so acquisition is exercised; keep it inside the camera
    #: half-FOV (20 deg) or the target starts outside the frame.
    initial_attitude_offset_deg: float = 0.0
    offset_axis: int = 1
    #: Require the centroid to halve.  Only set on cases that start offset.
    expect_acquisition: bool = False

    @property
    def starts_saturated(self):
        return self.initial_momentum_nms >= (
            act_cfg.DESAT_ENTRY_FRACTION * act_cfg.CONTROLLER_H_MAX)


@dataclass
class CaseResult:
    """Outcome of one case."""

    case: MissionCase
    metrics: dict = field(default_factory=dict)
    checks: list = field(default_factory=list)   # (label, ok, detail)
    error: str = None
    #: Derived telemetry of the completed run, for callers that want to plot
    #: it (``run_test_cases.py``).  ``None`` if the case raised.
    telemetry: object = None
    #: The synthetic camera feed, when the case used one.
    feed: object = None

    @property
    def passed(self):
        return self.error is None and all(ok for _, ok, _ in self.checks)


# ---------------------------------------------------------------------------
# Execution
# ---------------------------------------------------------------------------
def run_case(case, verbose=False):
    """Build, run and evaluate a single case."""
    result = CaseResult(case=case)
    try:
        momenta = np.full(3, case.initial_momentum_nms, dtype=float)
        rw_omega_rpm = reaction_wheels.initial_speed_rpm(
            initial_momentum_nms=momenta)

        bundle = builder.build_simulation(rw_omega_rpm, record_history=True)
        bundle.controller.current_mission_cmd = case.mission_cmd

        feed = None
        if case.needs_camera:
            feed = SyntheticCameraFeed(bundle)
            bundle.controller.camera_feed = feed

        rate = np.asarray(case.initial_rate_rad_s, dtype=float)
        bundle.spacecraft.hub.omega_BN_BInit = rate.reshape(3, 1)

        if case.initial_attitude_offset_deg:
            apply_attitude_offset(bundle, case.initial_attitude_offset_deg,
                                  case.offset_axis)

        run_recorders = recorders.attach_recorders(bundle)
        bundle.sim.InitializeSimulation()
        bundle.sim.ConfigureStopTime(macros.sec2nano(case.seconds))
        bundle.sim.ExecuteSimulation()

        telemetry = derive(run_recorders, bundle.controller)
        result.telemetry = telemetry
        result.feed = feed
        evaluate(case, telemetry, result, feed=feed)
    except Exception as error:            # noqa: BLE001 - reported, not raised
        result.error = f"{type(error).__name__}: {error}"
        if verbose:
            import traceback
            traceback.print_exc()
    return result


def apply_attitude_offset(bundle, angle_deg, axis):
    """Rotate the initial attitude off its nominal alignment.

    Applied on top of whatever ``apply_initial_state`` already set, so the
    nominal RAM alignment stays the reference and the offset is a deliberate
    pointing error at t=0.
    """
    sigma0 = np.asarray(bundle.spacecraft.hub.sigma_BNInit).flatten()
    dcm_offset = rbk.Mi(np.radians(angle_deg), int(axis))
    dcm_new = dcm_offset @ rbk.MRP2C(sigma0)
    bundle.spacecraft.hub.sigma_BNInit = \
        np.asarray(rbk.C2MRP(dcm_new)).reshape(3, 1)


def boresight_angle_deg(direction_B):
    """Angle between the +Z boresight and a body-frame direction [deg]."""
    unit = direction_B / np.linalg.norm(direction_B, axis=1, keepdims=True)
    return np.degrees(np.arccos(np.clip(unit @ BORESIGHT_B, -1.0, 1.0)))


def best_axis_alignment(direction_B_row):
    """Return the body axis best aligned with a direction, for diagnosis."""
    unit = direction_B_row / np.linalg.norm(direction_B_row)
    labels = ("+X", "-X", "+Y", "-Y", "+Z", "-Z")
    components = (unit[0], -unit[0], unit[1], -unit[1], unit[2], -unit[2])
    index = int(np.argmax(components))
    return labels[index], np.degrees(np.arccos(np.clip(components[index],
                                                       -1.0, 1.0)))


def acs_occupancy(telemetry):
    """Return ``{acs_state: fraction_of_steps}``."""
    history = np.asarray(telemetry.diagnostics.actual_acs_history)
    if history.size == 0:
        return {}
    states, counts = np.unique(history, return_counts=True)
    return {int(s): c / history.size for s, c in zip(states, counts)}


def evaluate(case, telemetry, result, feed=None):
    """Fill ``result.metrics`` and ``result.checks`` for a completed run."""
    history = np.asarray(telemetry.diagnostics.actual_acs_history)
    occupancy = acs_occupancy(telemetry)
    rate_norm = telemetry.body_rate_norm
    wheel_momentum = act_cfg.CONTROLLER_I_RW * np.abs(telemetry.wheel_speeds)

    tail = max(1, len(rate_norm) // 10)
    result.metrics.update({
        "acs_occupancy": {ACS_NAMES.get(s, s): f"{f:.0%}"
                          for s, f in sorted(occupancy.items())},
        "rate_initial_rad_s": float(rate_norm[0]),
        "rate_final_rad_s": float(rate_norm[-1]),
        "wheel_h_initial_nms": float(np.max(wheel_momentum[0])),
        "wheel_h_final_nms": float(np.max(wheel_momentum[-1])),
    })

    # -- the commanded mode must actually be reached ----------------------
    reached = occupancy.get(case.expect_acs, 0.0)
    result.checks.append((
        f"reaches ACS {ACS_NAMES[case.expect_acs]}",
        reached > 0.10,
        f"{reached:.0%} of steps"))

    # -- saturated start: DESAT first, commanded mode afterwards -----------
    if case.starts_saturated:
        check_desat_then_mode(case, telemetry, history, result)

    # -- detumble: the rate must actually come down ------------------------
    if case.mission_cmd == 1:
        initial, final = rate_norm[0], rate_norm[-1]
        result.checks.append((
            "body rate reduced >=80%",
            final < 0.2 * initial,
            f"{initial:.3e} -> {final:.3e} rad/s"))

    # -- pointing modes: settle, and optionally acquire the boresight ------
    direction = {
        "nadir": telemetry.earth_direction_B,
        "sun": telemetry.sun_direction_B,
        "target": telemetry.target_direction_B,
    }.get(case.pointing_target)

    if direction is not None:
        angle = boresight_angle_deg(direction)
        axis, axis_angle = best_axis_alignment(direction[-1])
        result.metrics.update({
            "boresight_err_initial_deg": float(angle[0]),
            "boresight_err_final_deg": float(angle[-1]),
            "best_aligned_axis": f"{axis} ({axis_angle:.1f} deg)",
        })
        settled = float(np.std(angle[-tail:]))
        result.metrics["boresight_std_tail_deg"] = settled
        result.checks.append((
            "pointing settled (tail std < 2 deg)",
            settled < 2.0,
            f"{settled:.2f} deg"))
        if case.assert_pointing:
            result.checks.append((
                f"boresight within {case.max_pointing_error_deg:g} deg",
                angle[-1] < case.max_pointing_error_deg,
                f"{angle[-1]:.1f} deg"))

    # -- optical: the centroid must converge toward frame centre -----------
    if case.needs_camera and feed is not None:
        check_optical_convergence(feed, result)

    # -- nothing should diverge -------------------------------------------
    result.checks.append((
        "body rate bounded (<0.5 rad/s)",
        bool(np.all(np.isfinite(rate_norm)) and np.max(rate_norm) < 0.5),
        f"peak {np.max(rate_norm):.3e} rad/s"))


def check_desat_then_mode(case, telemetry, history, result):
    """Assert DESAT is entered first and the commanded mode follows it."""
    desat_steps = np.flatnonzero(history == ACS_DESAT)
    mode_steps = np.flatnonzero(history == case.expect_acs)

    result.checks.append((
        "enters ACS DESAT",
        desat_steps.size > 0,
        f"{desat_steps.size} steps"))

    ordered = (desat_steps.size > 0 and mode_steps.size > 0
               and desat_steps[0] < mode_steps[0])
    detail = "mode never reached" if mode_steps.size == 0 else (
        f"first DESAT step {desat_steps[0] if desat_steps.size else '-'}, "
        f"first {ACS_NAMES[case.expect_acs]} step {mode_steps[0]}")
    result.checks.append(("DESAT precedes the commanded mode", ordered, detail))

    momentum = act_cfg.CONTROLLER_I_RW * np.abs(telemetry.wheel_speeds)
    exit_band = act_cfg.DESAT_EXIT_FRACTION * act_cfg.CONTROLLER_H_MAX
    shed = float(np.max(momentum[0]) - np.max(momentum[-1]))
    result.checks.append((
        "wheel momentum shed",
        shed > 0.0,
        f"{np.max(momentum[0]):.1f} -> {np.max(momentum[-1]):.1f} N m s "
        f"(exit band {exit_band:.1f})"))


def check_optical_convergence(feed, result):
    """Assert the tracked blob is driven toward the frame centre.

    Uses the feed's own record of the **true projected** blob position, which
    is independent of the control law -- so an inverted gain shows up as a
    growing offset rather than being hidden by the law under test.
    """
    time_s, x_cm, y_cm = feed.centroid_offsets()
    visible = np.isfinite(x_cm)
    seen = int(np.count_nonzero(visible))

    result.metrics["frames_target_visible"] = f"{seen}/{len(x_cm)}"
    result.checks.append((
        "target visible in the optical frame at t=0",
        bool(visible.size and visible[0]),
        "in frame" if visible.size and visible[0] else "NOT in frame"))

    if seen < 2:
        result.checks.append(
            ("optical centroid converges", False, "target never tracked"))
        return

    radius = np.hypot(x_cm[visible], y_cm[visible])
    times = time_s[visible]
    head = radius[:max(1, len(radius) // 10)]
    tail = radius[-max(1, len(radius) // 10):]
    result.metrics.update({
        "centroid_r_initial_px": float(np.mean(head)),
        "centroid_r_final_px": float(np.mean(tail)),
    })

    del times
    result.checks.append((
        f"target held within {OPTICAL_HOLD_TOLERANCE_PX:g} px of centre",
        float(np.mean(tail)) < OPTICAL_HOLD_TOLERANCE_PX,
        f"{np.mean(tail):.1f} px at end"))
    result.checks.append((
        f"centroid bounded (<{OPTICAL_MAX_EXCURSION_PX:g} px)",
        float(np.max(radius)) < OPTICAL_MAX_EXCURSION_PX,
        f"peak {np.max(radius):.1f} px"))
    result.checks.append((
        "target stays in the optical frame",
        seen == len(x_cm),
        f"lost for {len(x_cm) - seen} step(s)"))

    # Only meaningful when the run deliberately starts off-boresight; a case
    # that begins centred has nothing to converge and would fail on noise.
    if result.case.expect_acquisition:
        result.checks.append((
            "centroid acquired (halved from start)",
            float(np.mean(tail)) < 0.5 * float(np.mean(head)),
            f"{np.mean(head):.1f} -> {np.mean(tail):.1f} px"))


# ---------------------------------------------------------------------------
# Case list
# ---------------------------------------------------------------------------
def build_cases(include_desat=True):
    """Return the full ordered case list."""
    nominal = [
        MissionCase(
            name="cmd1 detumble",
            mission_cmd=1, seconds=900.0, expect_acs=ACS_DETUMBLE,
            initial_rate_rad_s=(0.03, -0.02, 0.04)),
        # +Z -> nadir is now confirmed, not assumed: the nominal run settles at
        # 0.06 deg with +Z the best-aligned axis, so this is a hard assertion.
        MissionCase(
            name="cmd4 nadir pointing",
            mission_cmd=4, seconds=900.0, expect_acs=ACS_POINT,
            pointing_target="nadir", assert_pointing=True,
            max_pointing_error_deg=5.0),
        MissionCase(
            name="cmd5 sun pointing",
            mission_cmd=5, seconds=900.0, expect_acs=ACS_POINT,
            pointing_target="sun", assert_pointing=True),
        MissionCase(
            name="cmd6 earth-target tracking",
            mission_cmd=6, seconds=900.0, expect_acs=ACS_POINT,
            pointing_target="target", assert_pointing=True),
        # Nominal: RAM-aligned at t=0 with command 9 already active, so the
        # companion projects on-boresight from the first step -- the optical
        # law only ever has to HOLD it.
        MissionCase(
            name="cmd9 intersat tracking (target centred at t=0)",
            mission_cmd=9, seconds=600.0, expect_acs=ACS_OPTICAL,
            needs_camera=True),
        # Acquisition: same start, but rolled 8 deg off boresight.  The target
        # is still inside the 20 deg half-FOV, so the loop can see it and must
        # pull it back in -- this is what actually exercises the sign of the
        # optical law.
        MissionCase(
            name="cmd9 intersat acquisition (8 deg offset)",
            mission_cmd=9, seconds=600.0, expect_acs=ACS_OPTICAL,
            needs_camera=True, initial_attitude_offset_deg=8.0,
            offset_axis=1, expect_acquisition=True),
    ]
    if not include_desat:
        return nominal

    saturated = [
        MissionCase(
            name=f"{case.name} [saturated start]",
            mission_cmd=case.mission_cmd,
            # Desaturation has to finish before the mode gets its turn.
            seconds=case.seconds + 600.0,
            expect_acs=case.expect_acs,
            initial_rate_rad_s=case.initial_rate_rad_s,
            initial_momentum_nms=SATURATED_MOMENTUM_NMS,
            pointing_target=case.pointing_target,
            # Pointing accuracy is not asserted after a desat: the run spends
            # its first phase shedding momentum, not pointing.
            assert_pointing=False,
            needs_camera=case.needs_camera)
        for case in nominal
    ]
    return nominal + saturated


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------
def print_result(result, verbose=False):
    """Print one case's verdict, and its detail when asked or on failure."""
    case = result.case
    status = "PASS" if result.passed else "FAIL"
    print(f"[{status}] {case.name}  (cmd {case.mission_cmd}, "
          f"{case.seconds:g} s)")

    if result.error:
        print(f"    ERROR  {result.error}")
        return

    for label, ok, detail in result.checks:
        if verbose or not ok:
            print(f"    {'ok  ' if ok else 'FAIL'}  {label}: {detail}")
    if verbose:
        for key, value in result.metrics.items():
            print(f"          {key} = {value}")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Mission-mode regression scenarios for the generated ADCS.")
    parser.add_argument("--cases", type=int, nargs="*", metavar="CMD",
                        help="only run these mission commands")
    parser.add_argument("--no-desat", action="store_true",
                        help="skip the saturated-start cases")
    parser.add_argument("--verbose", action="store_true",
                        help="print every check and metric")
    args = parser.parse_args(argv)

    cases = build_cases(include_desat=not args.no_desat)
    if args.cases:
        wanted = set(args.cases)
        cases = [c for c in cases if c.mission_cmd in wanted]
    if not cases:
        print("No cases selected.")
        return 2

    print(f"Running {len(cases)} mission-mode case(s)\n")
    results = []
    for case in cases:
        result = run_case(case, verbose=args.verbose)
        results.append(result)
        print_result(result, verbose=args.verbose)
        print()

    failed = [r for r in results if not r.passed]
    print("=" * 64)
    print(f"{len(results) - len(failed)}/{len(results)} passed")
    for result in failed:
        print(f"  FAILED: {result.case.name}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
