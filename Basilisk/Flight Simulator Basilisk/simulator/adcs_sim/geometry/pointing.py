"""Pointing-vector helpers shared by the controller and the analysis code."""

import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import macros


def unit(vectors):
    """Normalise a single vector or an ``(n, 3)`` stack of vectors."""
    vectors = np.asarray(vectors, dtype=float)
    if vectors.ndim == 1:
        return vectors / np.linalg.norm(vectors)
    return vectors / np.linalg.norm(vectors, axis=1)[:, None]


def direction_in_body(sigma_BN_history, direction_N_history):
    """Rotate an inertial direction history into body axes sample by sample.

    Parameters
    ----------
    sigma_BN_history : array_like, shape (n, 3)
        Recorded MRP attitude.
    direction_N_history : array_like, shape (n, 3)
        Inertial direction at each sample; need not be normalised.

    Returns
    -------
    numpy.ndarray, shape (n, 3)
        Unit direction expressed in body axes.  For a perfectly pointed
        spacecraft one component sits at +/-1 and the others at 0.
    """
    directions_N = unit(direction_N_history)
    directions_B = np.empty_like(directions_N)
    for index, sigma in enumerate(np.asarray(sigma_BN_history)):
        directions_B[index] = rbk.MRP2C(sigma) @ directions_N[index]
    return directions_B


def nadir_direction(positions_N):
    """Return the unit Earth-pointing (nadir) direction in ECI."""
    return unit(-np.asarray(positions_N, dtype=float))


def solar_elevation_deg(surface_positions_N, sun_positions_N):
    """Solar elevation above the local horizon at a surface point [deg].

    Positive means the Sun is above the horizon at that location.
    """
    surface_positions_N = np.asarray(surface_positions_N, dtype=float)
    to_sun = unit(np.asarray(sun_positions_N, dtype=float) - surface_positions_N)
    local_normal = unit(surface_positions_N)
    cosine = np.sum(local_normal * to_sun, axis=1)
    return np.arcsin(np.clip(cosine, -1.0, 1.0)) * macros.R2D
