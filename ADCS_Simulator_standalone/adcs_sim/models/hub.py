"""Rigid-body spacecraft hub and numerical integrator.

Inventory item **E1**.  Status: implemented and adequate.  Flexible
appendages and fuel slosh would be added here via
``hingedRigidBodyStateEffector``, ``sphericalPendulum`` or
``linearSpringMassDamper``.
"""

from dataclasses import dataclass

from Basilisk.simulation import spacecraft, svIntegrators
from Basilisk.utilities import simHelpers

from adcs_sim.config import simulation as sim_cfg
from adcs_sim.config import spacecraft as sc_cfg


@dataclass
class Hub:
    """The spacecraft object and the integrator it holds a raw pointer to."""

    spacecraft: object
    #: Retained only to keep the C++ integrator alive; ``setIntegrator`` does
    #: not take ownership, so dropping this reference dangles the pointer.
    integrator: object


def add_hub(sim, task_name):
    """Create, configure and register the rigid spacecraft hub."""
    hub_object = spacecraft.Spacecraft()
    hub_object.ModelTag = sc_cfg.MODEL_TAG
    hub_object.hub.mHub = sc_cfg.HUB_MASS_KG
    hub_object.hub.r_BcB_B = sc_cfg.HUB_COM_OFFSET_B
    hub_object.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(
        sc_cfg.HUB_INERTIA_B)

    integrator = svIntegrators.svIntegratorRKF45(hub_object)
    integrator.relTol = sc_cfg.INTEGRATOR_REL_TOL
    integrator.absTol = sc_cfg.INTEGRATOR_ABS_TOL
    hub_object.setIntegrator(integrator)

    sim.AddModelToTask(task_name, hub_object, sim_cfg.HUB_PRIORITY)
    return Hub(spacecraft=hub_object, integrator=integrator)
