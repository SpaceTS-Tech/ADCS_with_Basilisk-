"""Scenario configuration.

Every tunable number, switch and file location lives in one of the modules
below.  Nothing in this package imports Basilisk simulation modules or calls
into them, so configuration can be inspected and unit-tested on its own.

=========================  =================================================
Module                     Contents
=========================  =================================================
:mod:`~.paths`             Repository, DLL and output-artifact locations.
:mod:`~.simulation`        Run length, controller rate, task priorities.
:mod:`~.spacecraft`        Mass properties, integrator, orbit, initial state.
:mod:`~.actuators`         Reaction wheel, magnetorquer, thruster ratings.
:mod:`~.sensors`           Navigation error model and future sensor switches.
:mod:`~.environment`       Gravity bodies, SPICE epoch, magnetic field.
:mod:`~.power`             Solar-array geometry, eclipse, power monitor.
:mod:`~.target`            Ground-target location.
:mod:`~.visualization`     Vizard socket and live-stream settings.
=========================  =================================================

Import the specific module you need (``from adcs_sim.config import actuators``)
rather than re-exporting names here, so that there is exactly one import path
for every constant.
"""

__all__ = [
    "actuators",
    "companion",
    "environment",
    "paths",
    "power",
    "sensors",
    "simulation",
    "spacecraft",
    "target",
    "visualization",
]
