"""Assemble the simulation: instantiate every model, then wire the messages.

.. important::
   **Registration order is part of the model.**  Within one Basilisk task,
   models with a higher priority run first, and models sharing the default
   priority run in the order they were added.  The sequence in
   :func:`build_simulation` reproduces the original monolithic scenario:

   .. code-block:: text

       SPICE (3) -> reaction wheels (2) -> hub (1)
         -> magnetorquers -> thrusters -> magnetic field
         -> navigation -> star tracker camera -> controller
         -> recorders / clock sync -> Vizard interface

   Reordering these calls changes which measurement each model sees within a
   step, so treat the order as load-bearing.
"""

from dataclasses import dataclass, field

from Basilisk.utilities import SimulationBaseClass, macros

from adcs_sim.config import actuators as act_cfg
from adcs_sim.config import environment as env_cfg
from adcs_sim.config import paths
from adcs_sim.config import power as power_cfg
from adcs_sim.config import simulation as sim_cfg
from adcs_sim.config import star_tracker_camera as str_cfg
from adcs_sim.flightsoftware.controller import GeneratedADCSController
from adcs_sim.models import (companion, environment, hub, initial_state,
                             magnetorquers, navigation, reaction_wheels,
                             solar_panels, star_tracker_camera, thrusters)
from adcs_sim.scenario.camera import star_tracker_dcm_cb


@dataclass
class SimulationBundle:
    """Every object a built scenario needs to keep alive and reach into."""

    sim: object
    task_name: str
    hub: hub.Hub
    gravity: environment.GravityEnvironment
    magnetic_field: object
    reaction_wheels: reaction_wheels.ReactionWheels
    magnetorquers: magnetorquers.Magnetorquers
    thruster: object
    navigation: object
    controller: GeneratedADCSController
    initial_state: initial_state.InitialState
    #: Passive proximity satellite, or ``None`` when disabled.
    companion: object = None
    #: Solar array, eclipse model and power monitor, or ``None`` when the
    #: power model is disabled.  Telemetry only -- see
    #: :mod:`adcs_sim.models.solar_panels`.
    power: object = None
    #: In-simulation star tracker camera feeding ``STR 1``, or ``None`` when
    #: frames come from Vizard (``config/star_tracker_camera.IMAGE_SOURCE``).
    star_tracker_camera: object = None
    #: Anything else that must not be garbage-collected mid-run.
    retained: list = field(default_factory=list)

    @property
    def spacecraft(self):
        """The rigid-body hub object."""
        return self.hub.spacecraft

    @property
    def visualized_spacecraft(self):
        """Spacecraft to draw, **main one first**.

        Order matters: ``enableUnityVisualization`` takes ``scList[0]`` as
        Vizard's ``firstSpacecraftName``, the default target for the camera and
        the custom model.
        """
        bodies = [self.spacecraft]
        if self.companion is not None:
            bodies.append(self.companion.spacecraft)
        return bodies

    @property
    def rw_effector_list(self):
        """Per-spacecraft RW effectors, aligned with :attr:`visualized_spacecraft`.

        ``enableUnityVisualization`` requires this list to match ``scList`` in
        length, with ``None`` for a spacecraft that has no wheels.
        """
        return [self.reaction_wheels.effector] + \
               [None] * (len(self.visualized_spacecraft) - 1)

    @property
    def sun_state_msg(self):
        return self.gravity.planet_state_msg("sun")

    @property
    def earth_state_msg(self):
        return self.gravity.planet_state_msg("earth")


def build_simulation(rw_omega_rpm, record_history=True,
                     library_path=paths.ADCS_LIBRARY, link=None):
    """Create the process, every model, and the message wiring.

    Parameters
    ----------
    rw_omega_rpm : array_like, shape (3,)
        Initial reaction-wheel speeds [RPM].
    record_history : bool
        Passed to the controller's diagnostics; False for live runs.
    library_path : str or pathlib.Path or None
        Location of the generated ADCS shared library.  ``None`` builds the
        simulator with no flight software: every actuator command is zero.
    link : str, pathlib.Path or LinkDocument, optional
        Connector link file (``*.aocslink``) that wires the simulator to the
        flight software.  ``None`` uses the hand-written bridge.  The DLL is
        still ``library_path``; :func:`adcs_sim.scenario.runner.run` takes it
        from the link file unless one was given explicitly.
    """
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess(sim_cfg.PROCESS_NAME)
    process.addTask(sim.CreateNewTask(
        sim_cfg.TASK_NAME,
        macros.sec2nano(sim_cfg.CONTROLLER_STEP_SECONDS)))
    task_name = sim_cfg.TASK_NAME

    # -- models, in registration order ------------------------------------
    hub_bundle = hub.add_hub(sim, task_name)
    spacecraft_object = hub_bundle.spacecraft

    gravity = environment.add_gravity_and_ephemeris(
        sim, task_name, spacecraft_object)
    mtq = magnetorquers.add_magnetorquers(sim, task_name, spacecraft_object)
    thruster = thrusters.add_thruster_torque_effector(
        sim, task_name, spacecraft_object)
    magnetic_field = environment.add_magnetic_field(
        sim, task_name, spacecraft_object)
    wheels = reaction_wheels.add_reaction_wheels(
        sim, task_name, spacecraft_object, rw_omega_rpm)
    nav = navigation.add_navigation(sim, task_name)

    # Renders STR 1 frames from the true state.  Registered just before the
    # controller, so the frame a step hands the flight software is that
    # step's; it only reads messages, so nothing registered earlier changes.
    str_camera = None
    if str_cfg.IMAGE_SOURCE == "basilisk":
        str_camera = star_tracker_camera.add_star_tracker_camera(
            sim, task_name, star_tracker_dcm_cb())
    elif str_cfg.IMAGE_SOURCE != "vizard":
        raise ValueError(f"star_tracker_camera.IMAGE_SOURCE must be "
                         f"'basilisk' or 'vizard', got {str_cfg.IMAGE_SOURCE!r}")

    controller = GeneratedADCSController(
        library_path,
        act_cfg.CONTROLLER_I_RW,
        act_cfg.CONTROLLER_H_MAX,
        mission_cmd=sim_cfg.MISSION_CMD,
        record_history=record_history,
        star_tracker_camera=str_camera,
    )
    sim.AddModelToTask(task_name, controller)

    # Registered last so the established model order above is untouched.  The
    # companion is dynamically independent -- no messages to or from anything
    # else -- so its position in the task only affects when its own state is
    # integrated.
    proximity_satellite = companion.add_companion(
        sim, task_name, gravity.central_body_mu, gravity.factory)

    # Power models are pure telemetry: they read the spacecraft state and the
    # Sun ephemeris and write power messages, with no effector and nothing fed
    # back to the controller.  Registered last so every model above sees the
    # same measurements it saw before the array existed.
    solar_array = solar_panels.add_solar_array(
        sim, task_name, spacecraft_object,
        {body: gravity.planet_state_msg(body)
         for body in power_cfg.ECLIPSE_OCCULTING_BODIES})

    # -- initial conditions ------------------------------------------------
    start_state = initial_state.apply_initial_state(
        spacecraft_object, gravity.central_body_mu)

    bundle = SimulationBundle(
        sim=sim,
        task_name=task_name,
        hub=hub_bundle,
        gravity=gravity,
        magnetic_field=magnetic_field,
        reaction_wheels=wheels,
        magnetorquers=mtq,
        thruster=thruster,
        navigation=nav,
        controller=controller,
        initial_state=start_state,
        companion=proximity_satellite,
        power=solar_array,
        star_tracker_camera=str_camera,
        retained=[process],
    )
    connect_messages(bundle)
    if link is not None:
        attach_link(bundle, link)
    return bundle


def attach_link(bundle, link):
    """Let a connector link file drive the bridge instead of the hand code.

    Runs after :func:`connect_messages`: the link re-subscribes the simulator
    inputs it drives, and inputs it leaves unconnected are written as zeros.
    """
    from adcs_sim.flightsoftware import link_runtime

    doc = link if hasattr(link, "wires") else link_runtime.load_link(link)
    runtime = link_runtime.LinkRuntime(doc)
    runtime.bind(bundle, bundle.controller.library)
    bundle.controller.link = runtime
    bundle.retained.append(runtime)
    return runtime


def connect_messages(bundle):
    """Wire the whole message graph.

    Kept separate from construction so the data flow can be read in one place:

    .. code-block:: text

        hub --> navigation --> controller <-- RW speeds
                                        <-- magnetic field
                                        <-- SPICE (Sun, Earth)
        controller --> RW motor torque  --> RW effector
                   --> MTB dipole       --> MTB effector
                   --> THR body torque  --> ExtForceTorque

        hub + SPICE Sun --> eclipse --> solar panels --> power monitor
                                                     (telemetry only)
    """
    controller = bundle.controller
    nav = bundle.navigation
    field_msg = bundle.magnetic_field.envOutMsgs[0]

    nav.scStateInMsg.subscribeTo(bundle.spacecraft.scStateOutMsg)

    controller.attNavInMsg.subscribeTo(nav.attOutMsg)
    controller.transNavInMsg.subscribeTo(nav.transOutMsg)
    controller.rwSpeedsInMsg.subscribeTo(
        bundle.reaction_wheels.effector.rwSpeedOutMsg)
    controller.magFieldInMsg.subscribeTo(field_msg)
    controller.sunStateInMsg.subscribeTo(bundle.sun_state_msg)
    controller.earthStateInMsg.subscribeTo(bundle.earth_state_msg)

    bundle.reaction_wheels.effector.rwMotorCmdInMsg.subscribeTo(
        controller.rwMotorTorqueOutMsg)

    mtb_effector = bundle.magnetorquers.effector
    mtb_effector.mtbCmdInMsg.subscribeTo(controller.mtbDipoleOutMsg)
    mtb_effector.mtbParamsInMsg.subscribeTo(bundle.magnetorquers.config_msg)
    mtb_effector.magInMsg.subscribeTo(field_msg)

    bundle.thruster.cmdTorqueInMsg.subscribeTo(controller.thrTorqueOutMsg)

    # Star tracker camera: true attitude/position (an image sensor sees the
    # real sky, not the navigation estimate) and the Sun for blinding.
    if bundle.star_tracker_camera is not None:
        bundle.star_tracker_camera.scStateInMsg.subscribeTo(
            bundle.spacecraft.scStateOutMsg)
        bundle.star_tracker_camera.sunStateInMsg.subscribeTo(
            bundle.sun_state_msg)

    # Power telemetry: reads the hub state and the Sun, feeds nothing back.
    solar_panels.connect_solar_array(
        bundle.power, bundle.sun_state_msg, bundle.spacecraft.scStateOutMsg)


def describe(bundle):
    """Return printable lines summarising how this run is configured."""
    lines = list(initial_state.describe(bundle.initial_state))
    library = bundle.controller.library
    lines.append("Flight software: " + (
        str(library.path) if bundle.controller.has_flight_software
        else "NONE -- open loop, no control torque or force"))
    if bundle.controller.link is not None:
        lines += bundle.controller.link.describe()
    else:
        lines.append("Wiring: hand-written bridge (controller.py)")
    lines.append("Model fidelity switches:")
    lines.append(f"  navigation error model: "
                 f"{'ON' if navigation.error_model_enabled() else 'OFF (truth state)'}")
    lines += reaction_wheels.describe_friction()
    deadband = ("ON u_min=%.6g N m" % act_cfg.RW_MIN_MOTOR_TORQUE
                if act_cfg.RW_USE_MIN_TORQUE else "OFF")
    lines.append(f"  RW min-torque deadband: {deadband}")
    disturbances = [
        name for name, enabled in (
            ("drag", env_cfg.DRAG_ENABLED),
            ("SRP", env_cfg.SOLAR_RADIATION_PRESSURE_ENABLED),
            ("gravity gradient", env_cfg.GRAVITY_GRADIENT_ENABLED),
            ("residual dipole", env_cfg.RESIDUAL_DIPOLE_ENABLED),
            ("eclipse", env_cfg.ECLIPSE_ENABLED),
        ) if enabled
    ]
    lines.append(f"  disturbance torques: "
                 f"{', '.join(disturbances) if disturbances else 'none'}")
    lines.append("  star tracker image (STR 1): " + (
        "rendered in the simulation" if bundle.star_tracker_camera is not None
        else "Vizard frames from disk"))
    if bundle.star_tracker_camera is not None:
        lines += bundle.star_tracker_camera.sky_warning()
    lines += companion.describe(bundle.companion)
    lines += solar_panels.describe(bundle.power)
    return lines
