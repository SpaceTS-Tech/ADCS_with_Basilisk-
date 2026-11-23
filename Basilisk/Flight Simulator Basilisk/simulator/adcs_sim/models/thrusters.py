"""Reaction-wheel desaturation actuator -- PLACEHOLDER.

Inventory item **A3**, status ``[ ]``.  There is no thruster model here.
``ExtForceTorque`` applies the commanded body torque directly to the hub: no
thrust magnitude, no nozzle locations or moment arms, no on/off firing, no
thrust transients, no propellant depletion, and no translational delta-V.

To replace it (inventory backlog #6) the physics is all built-in ---
``thrusterDynamicEffector`` or ``thrusterStateEffector`` plus
``simIncludeThruster`` geometry and ``fuelTank``, driven by
``thrForceMapping`` and ``thrFiringSchmitt`` --- but custom glue is still
required, because the generated DLL emits a **torque vector** while a thruster
effector needs per-thruster on-times.
"""

from Basilisk.simulation import extForceTorque

import numpy as np

from adcs_sim.config import actuators as act_cfg


def limit_body_torque(torque_B):
    """Bound the generated desaturation torque to available authority [N m].

    The generated DESAT law currently returns roughly ``-RW_omega``, about
    628 N m at 6000 RPM.  ``ExtForceTorque`` would apply that literally and
    destabilise the integration, so the command is clamped until a physical
    thruster model replaces this interface.
    """
    limit = act_cfg.THR_MAX_BODY_TORQUE
    return np.clip(
        np.nan_to_num(torque_B, nan=0.0, posinf=limit, neginf=-limit),
        -limit, limit)


def add_thruster_torque_effector(sim, task_name, hub_spacecraft):
    """Attach and register the ideal-torque desaturation placeholder."""
    effector = extForceTorque.ExtForceTorque()
    effector.ModelTag = act_cfg.THR_MODEL_TAG
    hub_spacecraft.addDynamicEffector(effector)
    sim.AddModelToTask(task_name, effector)
    return effector
