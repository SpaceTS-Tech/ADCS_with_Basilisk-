"""Navigation sensor: attitude, body rate, position and velocity.

Inventory item **S1**.  ``SimpleNav`` stands in for the star tracker, gyro and
GPS simultaneously.  It supports a Gauss-Markov noise plus bounded random-walk
error model, but with ``NAV_ERROR_ENABLED = False`` the error matrices are left
unset and the module is a pass-through of the *true* spacecraft state.

Discrete replacements (all Basilisk built-ins, none wired up yet): see
inventory items **S2**-**S5** and the switches in
:mod:`adcs_sim.config.sensors`.
"""

from Basilisk.simulation import simpleNav

from adcs_sim.config import sensors as sensor_cfg


def add_navigation(sim, task_name):
    """Create and register the navigation sensor.

    Subscribe its ``scStateInMsg`` afterwards -- wiring lives in
    :mod:`adcs_sim.scenario.builder`, not here.
    """
    nav = simpleNav.SimpleNav()
    nav.ModelTag = sensor_cfg.NAV_MODEL_TAG

    if sensor_cfg.NAV_ERROR_ENABLED:
        # Only touch the error model when it is switched on: leaving PMatrix
        # and walkBounds at their defaults is what makes this a truth sensor.
        nav.PMatrix = sensor_cfg.NAV_P_MATRIX
        nav.walkBounds = sensor_cfg.NAV_WALK_BOUNDS

    sim.AddModelToTask(task_name, nav)
    return nav


def error_model_enabled():
    """Whether this run's navigation output carries sensor error."""
    return bool(sensor_cfg.NAV_ERROR_ENABLED)
