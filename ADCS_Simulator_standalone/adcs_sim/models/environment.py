"""Gravity, SPICE ephemeris and the Earth magnetic field.

Inventory items **E2** (point-mass gravity, no J2), **E3** (SPICE) and
**E4** (tilted centred dipole).  Disturbance effectors **E5**-**E9** are not
instantiated; when they are added, give each one an ``add_*`` function here.
"""

from dataclasses import dataclass

from Basilisk.simulation import magneticFieldCenteredDipole
from Basilisk.utilities import simIncludeGravBody, simSetPlanetEnvironment

from adcs_sim.config import environment as env_cfg
from adcs_sim.config import simulation as sim_cfg


@dataclass
class GravityEnvironment:
    """Gravity bodies, the SPICE interface, and their owning factory."""

    #: Retained: the factory owns the gravity-body objects handed to the hub.
    factory: object
    bodies: dict
    central_body: object
    spice: object

    @property
    def central_body_mu(self):
        """Gravitational parameter of the integration centre [m^3/s^2]."""
        return self.central_body.mu

    def planet_state_msg(self, planet_name):
        """Return the SPICE state message for a body created by the factory."""
        return self.spice.planetStateOutMsgs[planet_state_index(planet_name)]


def planet_state_index(planet_name):
    """Index of a body inside ``spiceInterface.planetStateOutMsgs``.

    ``gravBodyFactory.createBodies`` preserves the order of
    :data:`adcs_sim.config.environment.GRAVITY_BODIES` in the SPICE output, so
    resolve indices from that tuple instead of hard-coding them.
    """
    return list(env_cfg.GRAVITY_BODIES).index(planet_name.lower())


def add_gravity_and_ephemeris(sim, task_name, hub_spacecraft):
    """Create the gravity bodies and SPICE interface, and attach them."""
    factory = simIncludeGravBody.gravBodyFactory()
    bodies = factory.createBodies(*env_cfg.GRAVITY_BODIES)

    central_body = bodies[env_cfg.CENTRAL_BODY]
    central_body.isCentralBody = True
    if env_cfg.USE_SPHERICAL_HARMONICS:
        raise NotImplementedError(
            "Spherical-harmonics gravity is not wired up yet; see inventory "
            "backlog #5. Load a coefficient file via "
            "simIncludeGravBody.loadGravFromFile before enabling this."
        )

    spice_object = factory.createSpiceInterface(
        time=env_cfg.SPICE_EPOCH, epochInMsg=True)
    spice_object.zeroBase = env_cfg.SPICE_ZERO_BASE
    sim.AddModelToTask(task_name, spice_object, sim_cfg.SPICE_PRIORITY)

    factory.addBodiesTo(hub_spacecraft)
    return GravityEnvironment(factory=factory, bodies=bodies,
                              central_body=central_body, spice=spice_object)


def add_magnetic_field(sim, task_name, hub_spacecraft):
    """Create and register the Earth centred-dipole magnetic field model."""
    if env_cfg.USE_WMM_MAGNETIC_FIELD:
        raise NotImplementedError(
            "WMM magnetic field is not wired up yet; see inventory backlog #9. "
            "Swap in Basilisk.simulation.magneticFieldWMM here."
        )

    field = magneticFieldCenteredDipole.MagneticFieldCenteredDipole()
    field.ModelTag = env_cfg.MAGNETIC_FIELD_MODEL_TAG
    field.addSpacecraftToModel(hub_spacecraft.scStateOutMsg)
    simSetPlanetEnvironment.centeredDipoleMagField(
        field, env_cfg.MAGNETIC_FIELD_PLANET)
    sim.AddModelToTask(task_name, field)
    return field
