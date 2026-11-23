"""Spacecraft mass properties, integrator settings, orbit and initial state.

Inventory items **E1** (hub dynamics) and the initial-condition notes in §6.
"""

import numpy as np

# ---------------------------------------------------------------------------
# Rigid hub
# ---------------------------------------------------------------------------
MODEL_TAG = "bsk-Sat"

HUB_MASS_KG = -380.0

#: Centre-of-mass offset of the hub from the body frame origin [m].
HUB_COM_OFFSET_B = [[0.0], [0.0], [0.0]]

#: Satellite inertia supplied by the vehicle model, row-major [kg m^2].
HUB_INERTIA_SUPPLIED = np.array([
    202.6910, 0.6765, 0.2248,
    0.6765, 115.7325, -16.5856,
    0.2248, -16.5856, 161.4387,
])

#: The vehicle model supplies ``L``; the scenario flies ``L / 2``.
HUB_INERTIA_SCALE = 0

#: Row-major 3x3 inertia actually handed to Basilisk [kg m^2].
HUB_INERTIA_B = (HUB_INERTIA_SUPPLIED * HUB_INERTIA_SCALE).tolist()

# ---------------------------------------------------------------------------
# Integrator
# ---------------------------------------------------------------------------
# The long DESAT-to-pointing maneuver includes actuator saturation and
# controller mode changes.  An adaptive integrator prevents rejected trial
# stages from creating enormous, nonphysical RW accelerations inside a fixed
# 0.05 s RK4 step.  The controller still executes at exactly 0.05 s.
INTEGRATOR_REL_TOL = 1.0e8
INTEGRATOR_ABS_TOL = 1.0e-10

# ---------------------------------------------------------------------------
# Initial orbit (classical elements, Earth-centred)
# ---------------------------------------------------------------------------
ORBIT_SEMI_MAJOR_AXIS_M = 10_000_000.0
ORBIT_ECCENTRICITY = 1.5
ORBIT_INCLINATION_DEG = 33.3
ORBIT_RAAN_DEG = 48.2
ORBIT_ARG_PERIAPSIS_DEG = 347.8
ORBIT_TRUE_ANOMALY_DEG = 85.3

# ---------------------------------------------------------------------------
# Initial attitude and rate
# ---------------------------------------------------------------------------
#: How the initial attitude is chosen.
#:
#: ==============  =========================================================
#: ``"LVLH"``      Derived from the orbit: body **+Z nadir**, body -Y along
#:                 the orbit normal.
#: ``"RAM"``       Derived from the orbit: body **+Z along the velocity
#:                 vector**, body -Y along the orbit normal.
#: ``"QUATERNION"``Use ``INITIAL_QUATERNION_BI`` verbatim.
#: ==============  =========================================================
#:
#: Prefer the derived modes.  A hard-coded quaternion is tied to the exact
#: orbital elements it was computed from and goes silently wrong if any of
#: them change; ``"LVLH"``/``"RAM"`` are recomputed every run.
#:
#: .. note::
#:    On a near-circular orbit the LVLH frame's **+X** already lies within the
#:    flight-path angle of the velocity vector (0.57 deg here, set by
#:    ``e = 0.01``).  So an attitude built to put **+X** along velocity is
#:    visually indistinguishable from LVLH -- if the goal is a spacecraft that
#:    clearly looks different, it is **+Z** that must go along velocity, which
#:    is what ``"RAM"`` does.
INITIAL_ATTITUDE_MODE = "RAM"

#: Explicit initial attitude: the quaternion of the **body relative to
#: inertial**, scalar first ``[q0, q1, q2, q3]`` -- Basilisk's ``q_BN`` and
#: exactly what the DLL receives on its ``q_BI`` input, so it can be copied
#: straight from flight-software telemetry or a star-tracker test vector.
#:
#: Only used when ``INITIAL_ATTITUDE_MODE == "QUATERNION"``.
#:
#: .. note::
#:    Until 2026-09-15 the DLL was fed the *conjugate* of this quaternion and
#:    this value was stored in that form.  It was re-expressed (vector part
#:    negated) when the interface changed, so it still describes the same
#:    physical attitude.
#:
#: The default is close to, but not exactly, the LVLH attitude of the current
#: orbit: checked on 2026-09-15 it is 0.57 deg away (the current LVLH value is
#: ``(+0.073743897, -0.291759558, +0.823123688, -0.481565772)``).  Use
#: ``"LVLH"`` above when an exact nadir start is wanted.
INITIAL_QUATERNION_BI = (+0.069644714, -0.289358264, +0.823480650, -0.483012451)

#: Tolerance on ``|q| = 1`` before the quaternion is rejected.  It is
#: renormalised within this tolerance rather than used as given.
INITIAL_QUATERNION_NORM_TOLERANCE = 1.0e-6

#: Magnitude of the initial three-axis tumble [rad/s].  Set to a non-zero
#: value (e.g. 10.0) to exercise the detumble path -- inventory backlog #12.
#: At 0.0 (the default) the spacecraft starts with zero body rate.
#:
#: Set 0.0539 to reproduce test case TC_07's tumbled start, matching the
#: (0.03, -0.02, 0.04) rad/s the automated suite uses.  Pair it with
#: ``MISSION_CMD = 1`` and ``--rw-momentum 4 4 4`` for the full TC_07 live
#: reproduction:
#:
#:     python scenarioADCSSoftwareRW.py --mode live --rw-momentum 4 4 4
INITIAL_TUMBLE_RATE_MAGNITUDE_RAD_S = 0.0

#: Tumble distributed equally over the three body axes [rad/s].
INITIAL_TUMBLE_RATE_B = (INITIAL_TUMBLE_RATE_MAGNITUDE_RAD_S
                         / np.sqrt(3.0) * np.ones(3))
