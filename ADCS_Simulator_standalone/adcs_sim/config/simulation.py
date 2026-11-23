"""Simulation timing, run length and task execution order."""

#: Mission command loaded into the generated controller at reset.
#: 0-11 are valid; 4 is nadir pointing.
#:
#: Set to 1 to reproduce test case TC_07 (detumble from a saturated-wheel
#: state) live.  Pair it with a non-zero
#: ``spacecraft.INITIAL_TUMBLE_RATE_MAGNITUDE_RAD_S`` and ``--rw-momentum``
#: at the wheel's capacity so the run starts both tumbling and saturated:
#:
#:     python scenarioADCSSoftwareRW.py --mode live --rw-momentum 4 4 4
#:
#: Use 4 instead for TC_08 (nadir from saturated wheels).
MISSION_CMD = 1

#: Wall-clock length of a ``--mode file`` run [s].
SIMULATION_SECONDS = 5000.0

#: Base sample time of the generated Simulink model [s].  The Basilisk task
#: runs at this rate so the controller executes at exactly its design rate.
CONTROLLER_STEP_SECONDS = 0.05

#: Recording interval for slow-moving states (attitude, orbit, ephemeris) [s].
#: Actuator commands are always recorded at ``CONTROLLER_STEP_SECONDS``.
PLOT_SAMPLE_SECONDS = 1.0

#: Analysis window start: statistics are reported for ``t >= `` this [s].
SETTLING_TIME_SECONDS = 200.0

# ---------------------------------------------------------------------------
# Task execution order
# ---------------------------------------------------------------------------
# Within one Basilisk task, models with a *higher* priority execute first.
# Models registered with the default priority (-1) execute in the order they
# were added.  These values reproduce the original monolithic scenario:
#
#   SPICE (3) -> reaction wheels (2) -> hub (1) -> then, in registration
#   order: magnetorquers, thrusters, magnetic field, navigation, controller,
#   recorders / clock sync, Vizard interface.
#
# Changing them changes which measurements a model sees within a step.
SPICE_PRIORITY = 3
REACTION_WHEEL_PRIORITY = 2
HUB_PRIORITY = 1

#: Names used for the single process/task the scenario creates.
PROCESS_NAME = "simProcess"
TASK_NAME = "simTask"
