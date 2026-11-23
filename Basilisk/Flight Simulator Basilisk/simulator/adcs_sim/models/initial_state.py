"""Orbit and attitude initialisation.

Three ways to set the initial attitude, selected by
:data:`~adcs_sim.config.spacecraft.INITIAL_ATTITUDE_MODE`:

* ``"LVLH"`` -- derived from the orbit: body **+Z** nadir, body **-Y** along
  the orbit normal, body **+X** completing the right-handed set.
* ``"RAM"``  -- derived from the orbit: body **+Z along the velocity vector**,
  body **-Y** along the orbit normal.
* ``"QUATERNION"`` -- :data:`~adcs_sim.config.spacecraft.INITIAL_QUATERNION_BI`,
  the attitude quaternion of the body relative to inertial (Basilisk ``q_BN``).

The derived modes are preferred: a hard-coded quaternion is tied to the exact
orbital elements it was computed from.

The LVLH angular rate is computed and returned in every mode for reference; the
flown initial body rate is the configured tumble (inventory backlog #12).

.. warning::
   This is an **initial condition, not a pointing mode**.  The generated
   controller starts slewing toward whatever ``MISSION_CMD`` demands on the
   first step -- with the default command 4 (nadir) a RAM-pointing start is
   driven back to nadir within seconds.  Holding an attitude requires a
   controller mode that commands it, not an initial condition.

Quaternion conventions
----------------------
Two representations of the same attitude appear here:

===============  ==========================================================
``sigma_BN``     MRP.  The only form ``hub.sigma_BNInit`` accepts.
``q_BI``         What the DLL is fed: the attitude quaternion of the body
                 relative to inertial, scalar first -- identical to
                 Basilisk's ``q_BN`` from ``rbk.MRP2EP``, no conjugation.
===============  ==========================================================

:func:`mrp_bn_from_q_bi` and :func:`q_bi_from_mrp_bn` are the only places that
conversion happens, and they are exact inverses of each other.
"""

from dataclasses import dataclass

import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import macros, orbitalMotion

from adcs_sim.config import spacecraft as sc_cfg


@dataclass
class InitialState:
    """Everything the initialisation computed, for logging and reuse."""

    r_BN_N: np.ndarray
    v_BN_N: np.ndarray
    dcm_BN: np.ndarray
    sigma_BN: np.ndarray
    omega_BN_B: np.ndarray
    #: LVLH orbital angular rate, inertial components [rad/s].
    omega_LVLH_N: np.ndarray
    #: The same rate in body axes -- the rate a perfectly tracking nadir
    #: attitude would hold.  Compare against ``omega_BN_B`` to see how far the
    #: initial condition is from the LVLH reference.
    omega_LVLH_B: np.ndarray
    #: The initial attitude as the DLL will see it, scalar first.
    q_BI: np.ndarray
    #: Which branch produced the attitude, for the run banner.
    attitude_source: str


def orbit_elements():
    """Return the configured classical orbital elements."""
    elements = orbitalMotion.ClassicElements()
    elements.a = sc_cfg.ORBIT_SEMI_MAJOR_AXIS_M
    elements.e = sc_cfg.ORBIT_ECCENTRICITY
    elements.i = sc_cfg.ORBIT_INCLINATION_DEG * macros.D2R
    elements.Omega = sc_cfg.ORBIT_RAAN_DEG * macros.D2R
    elements.omega = sc_cfg.ORBIT_ARG_PERIAPSIS_DEG * macros.D2R
    elements.f = sc_cfg.ORBIT_TRUE_ANOMALY_DEG * macros.D2R
    return elements


def lvlh_nadir_frame(r_N, v_N):
    """Return ``(dcm_BN, omega_LVLH_N)`` for the nadir-pointing LVLH frame."""
    r_hat_N = r_N / np.linalg.norm(r_N)
    h_N = np.cross(r_N, v_N)
    h_hat_N = h_N / np.linalg.norm(h_N)

    z_hat_N = -r_hat_N            # body +Z toward nadir
    y_hat_N = -h_hat_N            # body -Y along the orbit normal
    x_hat_N = np.cross(y_hat_N, z_hat_N)

    dcm_BN = np.vstack((x_hat_N, y_hat_N, z_hat_N))
    omega_LVLH_N = h_N / np.dot(r_N, r_N)
    return dcm_BN, omega_LVLH_N


def ram_pointing_frame(r_N, v_N):
    """Return ``dcm_BN`` with body **+Z along the velocity vector**.

    The remaining freedom -- rotation about the boresight -- is removed with
    the same orbit-normal convention the LVLH frame uses, body -Y along +h, so
    this is a ram-pointing analogue of that frame rather than an arbitrary
    attitude.

    The frame closes exactly: ``h = r x v`` is perpendicular to ``v`` for any
    orbit, eccentric or not, so ``y_hat`` and ``z_hat`` are exactly orthogonal
    and no re-orthogonalisation is needed.
    """
    v_hat_N = v_N / np.linalg.norm(v_N)
    h_N = np.cross(r_N, v_N)
    h_hat_N = h_N / np.linalg.norm(h_N)

    z_hat_N = v_hat_N             # body +Z along velocity
    y_hat_N = -h_hat_N            # body -Y along the orbit normal
    x_hat_N = np.cross(y_hat_N, z_hat_N)

    return np.vstack((x_hat_N, y_hat_N, z_hat_N))


def _unit_quaternion(q_BI):
    """Validate and renormalise a configured quaternion."""
    quaternion = np.asarray(q_BI, dtype=float)
    if quaternion.shape != (4,):
        raise ValueError(
            f"INITIAL_QUATERNION_BI must have 4 components, got {quaternion.shape}")

    norm = np.linalg.norm(quaternion)
    if norm == 0.0:
        raise ValueError("INITIAL_QUATERNION_BI must not be the zero vector")
    if abs(norm - 1.0) > sc_cfg.INITIAL_QUATERNION_NORM_TOLERANCE:
        raise ValueError(
            f"INITIAL_QUATERNION_BI is not a unit quaternion: |q| = {norm:.9f}. "
            "A non-unit quaternion usually means a typo or a scalar-last "
            "ordering; fix it rather than relying on renormalisation.")
    return quaternion / norm


def mrp_bn_from_q_bi(q_BI):
    """Convert the DLL's ``q_BI`` to the MRP ``hub.sigma_BNInit`` needs.

    ``q_BI`` is Basilisk's ``q_BN`` unchanged (body relative to inertial), so
    this is a plain Euler-parameter to MRP conversion.
    """
    return rbk.EP2MRP(_unit_quaternion(q_BI))


def q_bi_from_mrp_bn(sigma_BN):
    """Convert an MRP attitude to the ``q_BI`` the DLL will be fed.

    Mirrors ``GeneratedADCSController._write_inputs`` exactly, so the reported
    value is what the flight code actually receives.
    """
    return rbk.MRP2EP(np.asarray(sigma_BN, dtype=float))


def apply_initial_state(hub_spacecraft, central_body_mu):
    """Set the hub's initial translational and rotational state.

    Returns
    -------
    InitialState
        The computed initial condition, for logging and cross-checks.
    """
    r_N, v_N = orbitalMotion.elem2rv(central_body_mu, orbit_elements())
    hub_spacecraft.hub.r_CN_NInit = r_N
    hub_spacecraft.hub.v_CN_NInit = v_N

    # The LVLH frame is computed either way: its rate is the reference the
    # reported initial condition is compared against.
    dcm_lvlh_BN, omega_LVLH_N = lvlh_nadir_frame(r_N, v_N)

    mode = str(sc_cfg.INITIAL_ATTITUDE_MODE).upper()
    if mode == "LVLH":
        dcm_BN = dcm_lvlh_BN
        sigma_BN = rbk.C2MRP(dcm_BN)
        attitude_source = "LVLH-aligned (+Z nadir)"
    elif mode == "RAM":
        dcm_BN = ram_pointing_frame(r_N, v_N)
        sigma_BN = rbk.C2MRP(dcm_BN)
        attitude_source = "RAM-aligned (+Z along velocity)"
    elif mode == "QUATERNION":
        sigma_BN = mrp_bn_from_q_bi(sc_cfg.INITIAL_QUATERNION_BI)
        dcm_BN = rbk.MRP2C(sigma_BN)
        attitude_source = "explicit q_BI from config"
    else:
        raise ValueError(
            f"INITIAL_ATTITUDE_MODE must be 'LVLH', 'RAM' or 'QUATERNION', "
            f"got {sc_cfg.INITIAL_ATTITUDE_MODE!r}")

    omega_LVLH_B = dcm_lvlh_BN @ omega_LVLH_N

    omega_BN_B = np.asarray(sc_cfg.INITIAL_TUMBLE_RATE_B, dtype=float)
    hub_spacecraft.hub.sigma_BNInit = sigma_BN.reshape(3, 1)
    hub_spacecraft.hub.omega_BN_BInit = omega_BN_B.reshape(3, 1)

    return InitialState(r_BN_N=r_N, v_BN_N=v_N, dcm_BN=dcm_BN,
                        sigma_BN=sigma_BN, omega_BN_B=omega_BN_B,
                        omega_LVLH_N=omega_LVLH_N, omega_LVLH_B=omega_LVLH_B,
                        q_BI=q_bi_from_mrp_bn(sigma_BN),
                        attitude_source=attitude_source)


def describe(initial_state):
    """Return printable lines describing the initial condition."""
    tumble = np.linalg.norm(initial_state.omega_BN_B)
    lvlh_rate = np.linalg.norm(initial_state.omega_LVLH_B)
    q_BI = initial_state.q_BI
    sigma = initial_state.sigma_BN
    return [
        "Initial spacecraft state:",
        f"  |r_BN_N| = {np.linalg.norm(initial_state.r_BN_N):.3f} m, "
        f"|v_BN_N| = {np.linalg.norm(initial_state.v_BN_N):.3f} m/s",
        f"  attitude: {initial_state.attitude_source}",
        f"    q_BI (as fed to the DLL) = "
        f"[{q_BI[0]:+.9f}, {q_BI[1]:+.9f}, {q_BI[2]:+.9f}, {q_BI[3]:+.9f}]",
        f"    sigma_BN (MRP in Basilisk) = "
        f"[{sigma[0]:+.9f}, {sigma[1]:+.9f}, {sigma[2]:+.9f}]",
        f"  initial body rate = {tumble:.6e} rad/s "
        f"(LVLH reference rate = {lvlh_rate:.6e} rad/s)",
    ]
