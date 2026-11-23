"""Magnetic torque rods: 3x orthogonal, body-aligned.

Inventory item **A2**.  ``MtbEffector`` applies the true physical
``tau_B = (Gt m) x B_B`` with per-rod dipole saturation, so authority correctly
collapses when the commanded dipole is parallel to **B**.

What is *not* modelled -- and would have to be written by hand, as Basilisk has
no module for it:

* coil electrical dynamics (R, L, current rise time) -- the dipole responds
  instantly,
* core hysteresis and remanence,
* the rods' self-field corrupting a magnetometer,
* duty-cycling / rod-versus-magnetometer time multiplexing.
"""

from dataclasses import dataclass

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.simulation import MtbEffector

from adcs_sim.config import actuators as act_cfg


@dataclass
class Magnetorquers:
    """The torque-rod effector and its standalone configuration message."""

    effector: object
    #: Retained: the effector subscribes to this standalone message, so it
    #: must not be garbage-collected while the simulation runs.
    config_msg: object


# ---------------------------------------------------------------------------
# Command conditioning (inventory item A2 / C3)
# ---------------------------------------------------------------------------

def limit_voltage(voltage):
    """Clamp a generated torque-rod command to the drive-voltage rating [V]."""
    limit = act_cfg.MTQ_MAX_VOLTAGE
    return np.clip(np.asarray(voltage, dtype=float), -limit, limit)


def voltage_to_dipole(applied_voltage):
    """Convert applied rod voltage [V] to magnetic dipole [A m^2].

    A single linear gain, because the generated ADCS emits volts while
    ``MtbEffector`` needs a dipole.  Both endpoints of this gain are
    placeholders -- see the warning in :mod:`adcs_sim.config.actuators`.
    """
    return np.asarray(applied_voltage, dtype=float) * act_cfg.MTQ_DIPOLE_PER_VOLT


# ---------------------------------------------------------------------------
# Construction
# ---------------------------------------------------------------------------

def _config_payload():
    """Build the torque-rod array configuration message payload."""
    config = messaging.MTBArrayConfigMsgPayload()
    config.numMTB = act_cfg.MTQ_COUNT
    config.GtMatrix_B = act_cfg.MTQ_ALIGNMENT_MATRIX_B
    config.maxMtbDipoles = [act_cfg.MTQ_MAX_DIPOLE] * act_cfg.MTQ_COUNT
    return config


def add_magnetorquers(sim, task_name, hub_spacecraft):
    """Create the torque-rod effector, attach it to the hub, and register it."""
    effector = MtbEffector.MtbEffector()
    effector.ModelTag = act_cfg.MTQ_MODEL_TAG
    hub_spacecraft.addDynamicEffector(effector)
    sim.AddModelToTask(task_name, effector)

    config_msg = messaging.MTBArrayConfigMsg().write(_config_payload())
    return Magnetorquers(effector=effector, config_msg=config_msg)
