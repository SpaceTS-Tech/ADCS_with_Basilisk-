"""Message recorders for a file-mode run.

Two sample rates: slow-moving states (attitude, orbit, ephemeris, wheel speeds)
at ``PLOT_SAMPLE_SECONDS``, and actuator commands at the full controller rate so
saturation and chatter are not aliased away.
"""

from dataclasses import dataclass

from Basilisk.utilities import macros

from adcs_sim.config import simulation as sim_cfg


@dataclass
class Recorders:
    """Every recorder attached to a file-mode run."""

    attitude: object
    translation: object
    sun: object
    earth: object
    rw_speed: object
    rw_torque: object
    mtq_voltage: object
    mtq_torque: object
    thr_torque: object
    #: One recorder per solar panel; empty when the power model is disabled.
    panel_power: tuple = ()
    #: Power monitor and eclipse recorders, or ``None`` when disabled.
    power_monitor: object = None
    eclipse: object = None

    def all(self):
        """Return every recorder, in registration order."""
        recorders = [self.attitude, self.translation, self.sun, self.earth,
                     self.rw_speed, self.rw_torque, self.mtq_voltage,
                     self.mtq_torque, self.thr_torque]
        recorders += list(self.panel_power)
        recorders += [recorder for recorder in
                      (self.power_monitor, self.eclipse)
                      if recorder is not None]
        return recorders


def attach_recorders(bundle):
    """Create and register the recorders for ``bundle``.

    Returns
    -------
    Recorders
        The recorder set, which must be kept alive for the whole run.
    """
    state_interval = macros.sec2nano(sim_cfg.PLOT_SAMPLE_SECONDS)
    actuator_interval = macros.sec2nano(sim_cfg.CONTROLLER_STEP_SECONDS)

    nav = bundle.navigation
    controller = bundle.controller

    recorders = Recorders(
        attitude=nav.attOutMsg.recorder(state_interval),
        translation=nav.transOutMsg.recorder(state_interval),
        sun=bundle.sun_state_msg.recorder(state_interval),
        earth=bundle.earth_state_msg.recorder(state_interval),
        rw_speed=bundle.reaction_wheels.effector.rwSpeedOutMsg.recorder(
            state_interval),
        rw_torque=controller.rwMotorTorqueOutMsg.recorder(actuator_interval),
        mtq_voltage=controller.mtqVoltageOutMsg.recorder(actuator_interval),
        mtq_torque=bundle.magnetorquers.effector.mtbOutMsg.recorder(
            actuator_interval),
        thr_torque=controller.thrTorqueOutMsg.recorder(actuator_interval),
    )

    # Power varies on the orbit timescale, so the state rate is plenty.
    array = bundle.power
    if array is not None:
        recorders.panel_power = tuple(
            panel.nodePowerOutMsg.recorder(state_interval)
            for panel in array.panels)
        recorders.power_monitor = array.monitor.batPowerOutMsg.recorder(
            state_interval)
        if array.eclipse is not None:
            recorders.eclipse = array.eclipse.eclipseOutMsgs[0].recorder(
                state_interval)

    for recorder in recorders.all():
        bundle.sim.AddModelToTask(bundle.task_name, recorder)
    return recorders
