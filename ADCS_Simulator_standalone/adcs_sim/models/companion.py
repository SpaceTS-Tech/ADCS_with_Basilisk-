"""Passive companion satellite in close proximity to the main spacecraft.

A second ``Spacecraft`` with mass properties and orbital dynamics but no
controller, actuators or sensors.  Present so the formation can be visualised.

Independence
------------
Basilisk integrates each ``Spacecraft`` separately in the central body's
gravity field; there is no inter-spacecraft attraction.  Adding this therefore
leaves the main satellite's trajectory bit-for-bit unchanged -- verified by
comparing the main spacecraft's telemetry with and without the companion.

Placement
---------
The companion flies the **same orbit** as the main spacecraft with a small
offset in true anomaly.  Same semi-major axis means same period, so the
along-track separation is stable rather than drifting.  See
:mod:`adcs_sim.config.companion` for why radial/cross-track offsets are not
offered.
"""

from dataclasses import dataclass

import numpy as np
from Basilisk.simulation import spacecraft
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import orbitalMotion, simHelpers

from adcs_sim.config import companion as comp_cfg
from adcs_sim.models import initial_state


@dataclass
class Companion:
    """The companion spacecraft and its computed initial state."""

    spacecraft: object
    r_BN_N: np.ndarray
    v_BN_N: np.ndarray
    #: Along-track separation actually achieved at t=0 [m].
    separation_m: float


def _offset_elements(central_body_mu):
    """Return the companion's orbital elements and the true-anomaly offset.

    The along-track arc length ``s`` at orbit radius ``r`` corresponds to a
    true-anomaly offset of ``s / r``.
    """
    elements = initial_state.orbit_elements()
    r_main, _ = orbitalMotion.elem2rv(central_body_mu, elements)
    radius = np.linalg.norm(r_main)

    delta_true_anomaly = comp_cfg.COMPANION_ALONG_TRACK_OFFSET_M / radius
    elements.f += delta_true_anomaly
    return elements, delta_true_anomaly


def add_companion(sim, task_name, central_body_mu, gravity_factory):
    """Create, place and register the companion, or return ``None``.

    Parameters
    ----------
    gravity_factory : simIncludeGravBody.gravBodyFactory
        The same factory used for the main spacecraft.  The companion must be
        attached to it or it will fly in a straight line.
    """
    if not comp_cfg.ENABLE_COMPANION_SATELLITE:
        return None

    body = spacecraft.Spacecraft()
    body.ModelTag = comp_cfg.COMPANION_MODEL_TAG
    body.hub.mHub = comp_cfg.COMPANION_MASS_KG
    body.hub.r_BcB_B = comp_cfg.COMPANION_COM_OFFSET_B
    body.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(
        comp_cfg.COMPANION_INERTIA_B)
    # No integrator is set: Basilisk's default is fine for a passive body, and
    # each Spacecraft owns its own integrator anyway.

    elements, _ = _offset_elements(central_body_mu)
    r_N, v_N = orbitalMotion.elem2rv(central_body_mu, elements)
    body.hub.r_CN_NInit = r_N
    body.hub.v_CN_NInit = v_N

    if comp_cfg.COMPANION_ALIGN_INITIAL_ATTITUDE_TO_LVLH:
        dcm_BN, _ = initial_state.lvlh_nadir_frame(r_N, v_N)
        body.hub.sigma_BNInit = rbk.C2MRP(dcm_BN).reshape(3, 1)

    rate = np.asarray(comp_cfg.COMPANION_INITIAL_RATE_B, dtype=float)
    body.hub.omega_BN_BInit = rate.reshape(3, 1)

    # Must be attached to the gravity factory, or it flies in a straight line.
    gravity_factory.addBodiesTo(body)
    sim.AddModelToTask(task_name, body)

    main_r, _ = orbitalMotion.elem2rv(
        central_body_mu, initial_state.orbit_elements())
    return Companion(spacecraft=body, r_BN_N=r_N, v_BN_N=v_N,
                     separation_m=float(np.linalg.norm(r_N - main_r)))


def describe(companion):
    """Return printable lines describing the companion, if there is one."""
    if companion is None:
        return ["Companion satellite: none"]

    model = comp_cfg.COMPANION_MODEL_PATH
    appearance = ("Vizard built-in satellite" if model is None
                  else f"model '{model}'")
    rate = np.linalg.norm(comp_cfg.COMPANION_INITIAL_RATE_B)
    return [
        f"Companion satellite: '{comp_cfg.COMPANION_MODEL_TAG}' "
        f"({appearance})",
        f"  along-track offset = "
        f"{comp_cfg.COMPANION_ALONG_TRACK_OFFSET_M:+g} m "
        f"-> initial separation {companion.separation_m:.3f} m",
        f"  mass = {comp_cfg.COMPANION_MASS_KG:g} kg, "
        f"initial spin = {rate:.4g} rad/s",
    ]
