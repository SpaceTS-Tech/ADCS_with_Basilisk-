"""Printed end-of-run telemetry summary.

Returns lines rather than printing directly wherever practical, so the summary
can be captured in a regression test as well as shown on the console.
"""

import numpy as np

from adcs_sim.analysis.derived import COMPONENT_LABELS
from adcs_sim.config import actuators as act_cfg


def _window_note(telemetry):
    """Return a note if the run ended before the analysis window opened."""
    if np.any(telemetry.post_settling_state):
        return None
    return (f"  (run is shorter than the {telemetry.settling_time_seconds:g} s "
            f"analysis window; post-settling statistics omitted)")


def _body_rate_lines(telemetry):
    norm = telemetry.body_rate_norm
    settling = telemetry.settling_time_seconds
    # Clip: a run shorter than the analysis window has no sample at `settling`.
    settling_index = min(np.searchsorted(telemetry.time_seconds, settling),
                         norm.size - 1)
    return [f"  Body-rate magnitude: initial={norm[0]:.9e}, "
            f"at {telemetry.time_seconds[settling_index]:g} s="
            f"{norm[settling_index]:.9e}, "
            f"final={norm[-1]:.9e} rad/s"]


def _saturation_lines(telemetry):
    """Compare the initial wheel state against the controller's DESAT bands."""
    metric = act_cfg.CONTROLLER_I_RW * np.max(np.abs(telemetry.wheel_speeds[0]))
    entry = act_cfg.DESAT_ENTRY_FRACTION * act_cfg.CONTROLLER_H_MAX
    exit_threshold = act_cfg.DESAT_EXIT_FRACTION * act_cfg.CONTROLLER_H_MAX
    comparison = ">=" if metric >= entry else "<"
    return [
        f"  Controller saturation metric: {metric:.6e} "
        f"{comparison} entry={entry:.6e} N m s",
        f"  Controller h_max={act_cfg.CONTROLLER_H_MAX:.6e}, "
        f"DESAT exit threshold={exit_threshold:.6e} N m s",
    ]


def _wheel_excursion_lines(telemetry):
    speeds = telemetry.wheel_speeds
    lines = []
    for index in range(speeds.shape[1]):
        change = speeds[-1, index] - speeds[0, index]
        excursion = np.ptp(speeds[:, index])
        lines.append(f"  RW{index + 1}: delta={change:+.6e} rad/s, "
                     f"peak-to-peak={excursion:.6e} rad/s")
    return lines


def _torque_lines(telemetry):
    lines = []
    for series in telemetry.torque_series:
        finite_values = series.values[np.isfinite(series.values)]
        nonfinite_count = np.count_nonzero(~np.isfinite(series.values))
        if finite_values.size:
            lines.append(
                f"  {series.title}: "
                f"max(abs finite)={np.max(np.abs(finite_values)):.6e} N m, "
                f"non-finite samples={nonfinite_count}")
        else:
            lines.append(f"  {series.title}: no finite samples (all NaN)")

    settled = telemetry.post_settling_actuator
    if not np.any(settled):
        return lines
    lines.append(f"  RW_tau statistics after "
                 f"{telemetry.settling_time_seconds:g} s:")
    for index, component in enumerate(COMPONENT_LABELS):
        values = telemetry.rw_torques[settled, index]
        lines.append(f"    {component}: max(abs)={np.max(np.abs(values)):.9e}, "
                     f"RMS={np.sqrt(np.mean(values * values)):.9e}, "
                     f"final={values[-1]:+.9e} N m")
    return lines


def _direction_lines(telemetry, values, label):
    settled = telemetry.post_settling_state
    if not np.any(settled):
        return []
    lines = [f"  {label} after {telemetry.settling_time_seconds:g} s:"]
    for index, component in enumerate(COMPONENT_LABELS):
        column = values[settled, index]
        lines.append(f"    {component}: mean={np.mean(column):+.9f}, "
                     f"range=[{np.min(column):+.9f}, {np.max(column):+.9f}]")
    return lines


def _power_lines(telemetry):
    """Array generation over the run, or nothing when power was not modelled."""
    power = telemetry.total_power_w
    if power is None:
        return []

    lines = [f"  Solar array: mean={np.mean(power):.1f} W, "
             f"peak={np.max(power):.1f} W, min={np.min(power):.1f} W "
             f"(normal incidence would give "
             f"{telemetry.peak_array_power_w:.0f} W)"]
    for index, name in enumerate(telemetry.panel_names):
        column = telemetry.panel_powers_w[:, index]
        lines.append(f"    {name}: mean={np.mean(column):.1f} W, "
                     f"peak={np.max(column):.1f} W")
    if telemetry.stored_energy_wh is not None:
        duration = telemetry.power_time_seconds[-1]
        lines.append(f"  Energy generated: "
                     f"{telemetry.stored_energy_wh[-1]:.2f} W h over "
                     f"{duration:.0f} s")
    if telemetry.illumination_factor is not None:
        shadowed = np.mean(telemetry.illumination_factor < 1.0)
        lines.append(f"  Time in Earth's shadow: {shadowed:.1%} of the run")
    return lines


def summary_lines(telemetry):
    """Return the whole telemetry summary as printable lines."""
    lines = ["", "Simulation telemetry summary"]
    window_note = _window_note(telemetry)
    if window_note:
        lines.append(window_note)
    lines += _body_rate_lines(telemetry)
    lines += _saturation_lines(telemetry)
    lines += _wheel_excursion_lines(telemetry)
    lines += _torque_lines(telemetry)
    lines += _direction_lines(telemetry, telemetry.earth_direction_B,
                              "Earth direction in body frame")
    lines += _direction_lines(telemetry, telemetry.sun_direction_B,
                              "Sun direction in body frame")
    lines += _direction_lines(telemetry, telemetry.target_direction_B,
                              "Earth-target direction in body frame")
    elevation = telemetry.solar_elevation_deg
    lines.append(f"  Target solar elevation: min={np.min(elevation):.6f} deg, "
                 f"max={np.max(elevation):.6f} deg")
    lines += _power_lines(telemetry)
    return lines


def print_summary(telemetry):
    """Print :func:`summary_lines` to the console."""
    for line in summary_lines(telemetry):
        print(line)
