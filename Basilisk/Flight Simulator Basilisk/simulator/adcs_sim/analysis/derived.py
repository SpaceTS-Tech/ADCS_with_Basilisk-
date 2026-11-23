"""Turn raw recorder output into analysis-ready arrays.

Everything the plots and the printed summary need is computed once, here, so
that the two consumers can never disagree about what they are showing.  No
matplotlib import, so this module is safe to use in tests.
"""

from collections import namedtuple
from dataclasses import dataclass

import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import macros

from adcs_sim.config import power as power_cfg
from adcs_sim.config import simulation as sim_cfg
from adcs_sim.geometry import ground_target, pointing

#: Component labels used everywhere in plots and printed output.
COMPONENT_LABELS = ("x", "y", "z")

#: One actuator torque history, with the labels the plots and summary share.
TorqueSeries = namedtuple("TorqueSeries", "values title ylabel")


@dataclass
class DerivedTelemetry:
    """Analysis-ready arrays for one completed run."""

    # -- spacecraft state (state sample rate) ------------------------------
    time_seconds: np.ndarray
    sigma_BN: np.ndarray
    q_BI: np.ndarray
    body_rates: np.ndarray
    body_rate_norm: np.ndarray
    positions_N: np.ndarray

    # -- wheel state (state sample rate) -----------------------------------
    rw_time_seconds: np.ndarray
    wheel_speeds: np.ndarray

    # -- actuator commands (controller rate) -------------------------------
    torque_time_seconds: np.ndarray
    torque_series: tuple
    voltage_time_seconds: np.ndarray
    mtq_voltages: np.ndarray

    # -- pointing geometry (state sample rate) -----------------------------
    earth_direction_B: np.ndarray
    sun_positions_N: np.ndarray
    sun_direction_B: np.ndarray
    target_positions_N: np.ndarray
    target_direction_B: np.ndarray
    solar_elevation_deg: np.ndarray

    # -- generated-controller mode history ---------------------------------
    mode_time_seconds: np.ndarray
    diagnostics: object
    #: ``(n, 4)`` attitude estimate ``q_BI_est`` from the flight software, on
    #: ``mode_time_seconds``.  Shape ``(0, 4)`` when no history was recorded.
    q_BI_est: np.ndarray

    # -- analysis window ---------------------------------------------------
    settling_time_seconds: float
    #: Mask over ``torque_time_seconds`` for ``t >= settling_time_seconds``.
    post_settling_actuator: np.ndarray
    #: Mask over ``time_seconds`` for ``t >= settling_time_seconds``.
    post_settling_state: np.ndarray

    # -- solar-array power (state sample rate) -----------------------------
    # All ``None`` when the power model was disabled for the run; every
    # consumer checks ``total_power_w`` before using any of them.
    power_time_seconds: np.ndarray = None
    #: ``(n, n_panels)`` generation per panel [W].
    panel_powers_w: np.ndarray = None
    panel_names: tuple = ()
    total_power_w: np.ndarray = None
    net_power_w: np.ndarray = None
    stored_energy_wh: np.ndarray = None
    #: 1 in full sun, 0 in umbra; ``None`` when eclipse is not modelled.
    illumination_factor: np.ndarray = None

    @property
    def peak_array_power_w(self):
        """Array output at normal incidence, 1 AU, full sun [W]."""
        return sum(power_cfg.SOLAR_CONSTANT_1AU_W_M2 * panel.area_m2
                   * panel.efficiency for panel in power_cfg.SOLAR_PANELS)

    @property
    def rw_torques(self):
        """Reaction-wheel motor torque history [N m]."""
        return self.torque_series[0].values


def _power_arrays(recorders):
    """Return the power telemetry fields, or ``{}`` when none was recorded."""
    panel_recorders = getattr(recorders, "panel_power", ())
    if not panel_recorders:
        return {}

    panel_powers = np.column_stack(
        [np.asarray(recorder.netPower) for recorder in panel_recorders])
    fields = {
        "power_time_seconds": panel_recorders[0].times() * macros.NANO2SEC,
        "panel_powers_w": panel_powers,
        "panel_names": tuple(panel.name for panel in power_cfg.SOLAR_PANELS),
        "total_power_w": panel_powers.sum(axis=1),
    }

    monitor = getattr(recorders, "power_monitor", None)
    if monitor is not None:
        fields["net_power_w"] = np.asarray(monitor.currentNetPower)
        # The monitor stores energy in W s; W h is the unit a power budget
        # is written in.
        fields["stored_energy_wh"] = np.asarray(monitor.storageLevel) / 3600.0

    shadow = getattr(recorders, "eclipse", None)
    if shadow is not None:
        # Basilisk 2.11.1 is mid-rename: the sunlit fraction is
        # ``illuminationFactor`` on some objects and ``shadowFactor`` on
        # others.  Both are 1 in full sun and 0 in umbra.
        sunlit = getattr(shadow, "illuminationFactor", None)
        if sunlit is None:
            sunlit = shadow.shadowFactor
        fields["illumination_factor"] = np.asarray(sunlit)
    return fields


def derive(recorders, controller,
           settling_time_seconds=sim_cfg.SETTLING_TIME_SECONDS):
    """Build a :class:`DerivedTelemetry` from a completed run's recorders."""
    time_seconds = recorders.attitude.times() * macros.NANO2SEC
    sigma_BN = np.asarray(recorders.attitude.sigma_BN)
    q_BI = np.vstack([rbk.MRP2EP(sigma) for sigma in sigma_BN])

    body_rates = np.asarray(recorders.attitude.omega_BN_B)
    body_rate_norm = np.linalg.norm(body_rates, axis=1)

    positions_N = np.asarray(recorders.translation.r_BN_N)
    sun_positions_N = np.asarray(recorders.sun.PositionVector)
    target_positions_N = ground_target.position_inertial_history(
        recorders.earth.J20002Pfix)

    earth_direction_B = pointing.direction_in_body(
        sigma_BN, pointing.nadir_direction(positions_N))
    sun_direction_B = pointing.direction_in_body(
        sigma_BN, sun_positions_N - positions_N)
    target_direction_B = pointing.direction_in_body(
        sigma_BN, target_positions_N - positions_N)
    solar_elevation = pointing.solar_elevation_deg(
        target_positions_N, sun_positions_N)

    torque_time_seconds = recorders.rw_torque.times() * macros.NANO2SEC
    torque_series = (
        TorqueSeries(np.asarray(recorders.rw_torque.motorTorque)[:, :3],
                     "RW_tau", "RW motor torque [N m]"),
        TorqueSeries(np.asarray(recorders.mtq_torque.mtbNetTorque_B),
                     "MTQ_tau", "MTQ torque [N m]"),
        TorqueSeries(np.asarray(recorders.thr_torque.torqueRequestBody),
                     "THR_tau", "Thruster torque [N m]"),
    )

    return DerivedTelemetry(
        time_seconds=time_seconds,
        sigma_BN=sigma_BN,
        q_BI=q_BI,
        body_rates=body_rates,
        body_rate_norm=body_rate_norm,
        positions_N=positions_N,
        rw_time_seconds=recorders.rw_speed.times() * macros.NANO2SEC,
        wheel_speeds=np.asarray(recorders.rw_speed.wheelSpeeds)[:, :3],
        torque_time_seconds=torque_time_seconds,
        torque_series=torque_series,
        voltage_time_seconds=recorders.mtq_voltage.times() * macros.NANO2SEC,
        mtq_voltages=np.asarray(recorders.mtq_voltage.torqueRequestBody),
        earth_direction_B=earth_direction_B,
        sun_positions_N=sun_positions_N,
        sun_direction_B=sun_direction_B,
        target_positions_N=target_positions_N,
        target_direction_B=target_direction_B,
        solar_elevation_deg=solar_elevation,
        mode_time_seconds=np.asarray(controller.diagnostics.mode_time_seconds),
        diagnostics=controller.diagnostics,
        q_BI_est=np.asarray(controller.diagnostics.q_est_history,
                            dtype=float).reshape(-1, 4),
        settling_time_seconds=settling_time_seconds,
        post_settling_actuator=torque_time_seconds >= settling_time_seconds,
        post_settling_state=time_seconds >= settling_time_seconds,
        **_power_arrays(recorders),
    )
