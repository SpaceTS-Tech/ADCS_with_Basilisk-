"""Simulated sensor, actuator, environment and dynamics models.

One module per physical subsystem, matching the inventory sections so that a
model's status, configuration and construction code are always one hop apart.

=========================  =========  ==========================================
Module                     Inventory  Contents
=========================  =========  ==========================================
:mod:`~.hub`               E1         Rigid body + adaptive integrator.
:mod:`~.environment`       E2-E4      Gravity, SPICE ephemeris, magnetic field.
:mod:`~.reaction_wheels`   A1, S6     HR16 cluster, initial state, torque limit.
:mod:`~.magnetorquers`     A2         Torque rods + voltage->dipole conversion.
:mod:`~.thrusters`         A3         Ideal-torque placeholder for desaturation.
:mod:`~.navigation`        S1         ``SimpleNav`` and its error model.
:mod:`~.solar_panels`      P1, P2     Solar array, eclipse, power monitor.
:mod:`~.initial_state`     --         Orbit and LVLH attitude initialisation.
=========================  =========  ==========================================

Convention for every ``add_*`` function in this package:

* it creates the Basilisk objects, configures them from :mod:`adcs_sim.config`,
  registers them with the task, and
* it returns **every** Python object whose lifetime must outlast the call.
  Basilisk stores raw C++ pointers, so an integrator, factory or standalone
  message that gets garbage-collected leaves a dangling reference.
"""

__all__ = [
    "companion",
    "environment",
    "hub",
    "initial_state",
    "magnetorquers",
    "navigation",
    "reaction_wheels",
    "solar_panels",
    "thrusters",
]
