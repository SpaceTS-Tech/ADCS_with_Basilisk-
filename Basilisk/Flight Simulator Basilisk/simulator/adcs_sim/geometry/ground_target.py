"""Ground-target position in planet-fixed and inertial coordinates.

Inventory item **S7**.  See :mod:`adcs_sim.config.target` for the spherical
Earth caveat.
"""

import numpy as np
from Basilisk.utilities import macros

from adcs_sim.config import target as target_cfg


def position_planet_fixed():
    """Return the fixed ground target in Earth-fixed coordinates [m].

    Uses a sphere of radius ``TARGET_RADIUS_M``; no WGS-84 flattening.
    """
    latitude = target_cfg.TARGET_LATITUDE_DEG * macros.D2R
    longitude = target_cfg.TARGET_LONGITUDE_DEG * macros.D2R
    cos_latitude = np.cos(latitude)
    return np.array((
        target_cfg.TARGET_RADIUS_M * cos_latitude * np.cos(longitude),
        target_cfg.TARGET_RADIUS_M * cos_latitude * np.sin(longitude),
        target_cfg.TARGET_RADIUS_M * np.sin(latitude),
    ))


def position_inertial(dcm_PN):
    """Rotate the target into ECI given one planet-fixed-from-inertial DCM.

    Parameters
    ----------
    dcm_PN : array_like, shape (3, 3)
        SPICE ``J20002Pfix`` rotation for the sample of interest.
    """
    dcm_PN = np.asarray(dcm_PN).reshape((3, 3))
    return dcm_PN.T @ position_planet_fixed()


def position_inertial_history(dcm_PN_history):
    """Vectorised :func:`position_inertial` over a recorded DCM history.

    Parameters
    ----------
    dcm_PN_history : array_like
        Flat ``(n, 9)`` or already-shaped ``(n, 3, 3)`` history of the SPICE
        ``J20002Pfix`` rotation.

    Returns
    -------
    numpy.ndarray, shape (n, 3)
        Target position in ECI at each sample [m].
    """
    dcm_PN = np.asarray(dcm_PN_history).reshape((-1, 3, 3))
    return np.einsum("nij,j->ni",
                     np.transpose(dcm_PN, (0, 2, 1)),
                     position_planet_fixed())
