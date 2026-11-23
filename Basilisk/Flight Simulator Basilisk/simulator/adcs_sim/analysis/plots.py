"""Run figures: one function per plot, collected in a registry.

To add a figure, write a ``plot_*(telemetry, output_dir) -> Path`` function and
add one entry to :data:`PLOT_BUILDERS`.  Nothing else needs to change.

.. note::
   Import :mod:`adcs_sim.analysis.backend` and call ``select()`` before
   importing this module -- it imports ``matplotlib.pyplot`` at module level.
"""

import matplotlib.pyplot as plt
import numpy as np

from adcs_sim.analysis.derived import COMPONENT_LABELS
from adcs_sim.config import actuators as act_cfg
from adcs_sim.config import power as power_cfg
from adcs_sim.config import target as target_cfg

#: Resolution of every saved figure.
FIGURE_DPI = 200


def _save(fig, output_dir, name):
    """Save ``fig`` as ``<output_dir>/<name>.png`` and return the path."""
    path = output_dir / f"{name}.png"
    fig.tight_layout()
    fig.savefig(path, dpi=FIGURE_DPI)
    return path


def _plot_body_components(telemetry, values, title, ylabel, output_dir, name):
    """Shared layout for the three body-frame alignment plots."""
    fig, ax = plt.subplots(figsize=(9, 5))
    for index, component in enumerate(COMPONENT_LABELS):
        ax.plot(telemetry.time_seconds, values[:, index],
                label=f"Body {component}")
    ax.set(xlabel="Time [s]", ylabel=ylabel, title=title)
    ax.set_ylim(-1.05, 1.05)
    ax.grid(True)
    ax.legend()
    return _save(fig, output_dir, name)


# ---------------------------------------------------------------------------
# Spacecraft state
# ---------------------------------------------------------------------------

def plot_quaternion(telemetry, output_dir):
    fig, ax = plt.subplots()
    for index, label in enumerate((r"$q_0$", r"$q_1$", r"$q_2$", r"$q_3$")):
        ax.plot(telemetry.time_seconds, telemetry.q_BI[:, index], label=label)
    ax.set(xlabel="Time [s]", ylabel=r"Quaternion $q_{BI}$",
           title="Spacecraft attitude quaternion")
    ax.grid(True)
    ax.legend()
    return _save(fig, output_dir, "quaternion")


def plot_q_est(telemetry, output_dir):
    """The flight software's attitude estimate against the true attitude.

    ``q_BI_est`` is sampled at the controller rate and the true attitude at the
    slower state rate, so the truth is interpolated onto the estimate's time
    base before the two are compared.  The error is the rotation angle between
    them, taken through ``|q_est . q_true|`` because ``q`` and ``-q`` are the
    same attitude.

    Returns ``None`` when the run kept no estimate history, as a live run does.
    """
    q_est = telemetry.q_BI_est
    mode_time = telemetry.mode_time_seconds
    if q_est.size == 0 or mode_time.size == 0:
        return None

    truth = np.column_stack([np.interp(mode_time, telemetry.time_seconds,
                                       telemetry.q_BI[:, index])
                             for index in range(4)])
    truth /= np.linalg.norm(truth, axis=1, keepdims=True)

    # The outport sits at zero until the model's estimator writes it, and a
    # zero quaternion has no attitude to compare against.
    norms = np.linalg.norm(q_est, axis=1)
    usable = norms > 1e-9

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(9, 7))
    for index, label in enumerate((r"$q_0$", r"$q_1$", r"$q_2$", r"$q_3$")):
        line, = axes[0].plot(mode_time, q_est[:, index], label=f"{label} est")
        axes[0].plot(mode_time, truth[:, index], linestyle="--", linewidth=0.8,
                     color=line.get_color(), label=f"{label} true")
    axes[0].set(ylabel=r"Quaternion $q_{BI}$",
                title="Flight software attitude estimate (dashed: true attitude)")
    axes[0].legend(ncol=4, fontsize="small")

    if usable.any():
        alignment = np.abs(np.sum(q_est[usable] / norms[usable, None]
                                  * truth[usable], axis=1))
        error_deg = 2.0 * np.degrees(np.arccos(np.clip(alignment, -1.0, 1.0)))
        axes[1].plot(mode_time[usable], error_deg, color="tab:red")
        axes[1].set(ylabel="Estimate error [deg]")
    else:
        axes[1].text(0.5, 0.5,
                     "No estimate to compare: the generated model leaves\n"
                     r"$q_{BI,est}$ at zero in this build",
                     ha="center", va="center", transform=axes[1].transAxes)
        axes[1].set(ylabel="Estimate error [deg]", ylim=(0.0, 1.0))
    axes[1].set(xlabel="Time [s]")

    for ax in axes:
        ax.grid(True)
    return _save(fig, output_dir, "q_est")


def plot_angular_rate(telemetry, output_dir):
    fig, ax = plt.subplots()
    labels = (r"$\omega_x$", r"$\omega_y$", r"$\omega_z$")
    for index, label in enumerate(labels):
        ax.plot(telemetry.time_seconds, telemetry.body_rates[:, index],
                label=label)
    ax.set(xlabel="Time [s]", ylabel="Angular rate [rad/s]",
           title="Spacecraft body angular rate")
    ax.grid(True)
    ax.legend()
    return _save(fig, output_dir, "angular_rate")


# ---------------------------------------------------------------------------
# Actuators
# ---------------------------------------------------------------------------

def plot_rw_angular_velocity(telemetry, output_dir):
    """Absolute wheel speeds, and the change from each wheel's start."""
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 7))
    speeds = telemetry.wheel_speeds
    for index in range(speeds.shape[1]):
        axes[0].plot(telemetry.rw_time_seconds, speeds[:, index],
                     label=f"RW{index + 1}")
        axes[1].plot(telemetry.rw_time_seconds,
                     speeds[:, index] - speeds[0, index],
                     label=f"RW{index + 1}")
    axes[0].set(ylabel="Angular velocity [rad/s]",
                title="Reaction-wheel angular velocities")
    axes[1].set(xlabel="Time [s]", ylabel=r"Change $\Delta\Omega$ [rad/s]")
    for ax in axes:
        ax.grid(True)
        ax.legend()
    return _save(fig, output_dir, "rw_angular_velocity")


def plot_mtq_voltage(telemetry, output_dir):
    fig, ax = plt.subplots(figsize=(9, 5))
    for index, component in enumerate(COMPONENT_LABELS):
        ax.plot(telemetry.voltage_time_seconds,
                telemetry.mtq_voltages[:, index], label=component)
    ax.axhline(act_cfg.MTQ_MAX_VOLTAGE, color="black", linestyle="--",
               linewidth=1, label="voltage limits")
    ax.axhline(-act_cfg.MTQ_MAX_VOLTAGE, color="black", linestyle="--",
               linewidth=1)
    ax.set(xlabel="Time [s]", ylabel="Applied voltage [V]",
           title="Magnetic torque-rod voltage commands")
    ax.grid(True)
    ax.legend()
    return _save(fig, output_dir, "mtq_voltage")


def plot_adcs_torques(telemetry, output_dir):
    """All three actuator torques, annotated where samples are non-finite."""
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 9))
    for ax, series in zip(axes, telemetry.torque_series):
        torques = series.values
        nonfinite_count = np.count_nonzero(~np.isfinite(torques))
        has_finite_sample = False
        for index, component in enumerate(COMPONENT_LABELS):
            finite = np.isfinite(torques[:, index])
            if np.any(finite):
                has_finite_sample = True
                ax.plot(telemetry.torque_time_seconds[finite],
                        torques[finite, index], label=component)
        ax.set(title=series.title, ylabel=series.ylabel)
        ax.grid(True)
        if has_finite_sample:
            ax.legend()
            if nonfinite_count:
                ax.text(0.01, 0.95, f"{nonfinite_count} non-finite samples",
                        ha="left", va="top", transform=ax.transAxes,
                        color="crimson")
        else:
            ax.text(0.5, 0.5, "No finite samples (controller output is NaN)",
                    ha="center", va="center", transform=ax.transAxes,
                    color="crimson")
    axes[-1].set_xlabel("Time [s]")
    return _save(fig, output_dir, "adcs_torques")


def plot_rw_torque_after_settling(telemetry, output_dir):
    """Reaction-wheel torque once the initial transient has passed."""
    settled = telemetry.post_settling_actuator
    settling = telemetry.settling_time_seconds
    fig, ax = plt.subplots(figsize=(9, 5))
    for index, component in enumerate(COMPONENT_LABELS):
        ax.plot(telemetry.torque_time_seconds[settled],
                telemetry.rw_torques[settled, index], label=component)
    ax.set(xlabel="Time [s]", ylabel="RW motor torque [N m]",
           title=rf"RW_tau after {settling:g} seconds")
    ax.ticklabel_format(axis="y", style="sci", scilimits=(0, 0))
    ax.grid(True)
    ax.legend()
    return _save(fig, output_dir, f"rw_tau_after_{settling:g}s")


# ---------------------------------------------------------------------------
# Pointing
# ---------------------------------------------------------------------------

def plot_earth_pointing(telemetry, output_dir):
    return _plot_body_components(
        telemetry, telemetry.earth_direction_B,
        "Earth-pointing alignment (ideal fixed body component = +/-1)",
        "Earth-direction component in body frame",
        output_dir, "earth_pointing")


def plot_sun_pointing(telemetry, output_dir):
    return _plot_body_components(
        telemetry, telemetry.sun_direction_B,
        "Sun-pointing alignment (controller aligns body +Z with Sun)",
        "Sun-direction component in body frame",
        output_dir, "sun_pointing")


def plot_earth_target_pointing(telemetry, output_dir):
    return _plot_body_components(
        telemetry, telemetry.target_direction_B,
        "Earth-target alignment (controller aligns body +Z with target)",
        "Target-direction component in body frame",
        output_dir, "earth_target_pointing")


def plot_target_sunlight(telemetry, output_dir):
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(telemetry.time_seconds, telemetry.solar_elevation_deg)
    ax.axhline(0.0, color="black", linestyle="--", label="Horizon")
    ax.set(xlabel="Time [s]", ylabel="Solar elevation at target [deg]",
           title=f"Sunlight at Indian target "
                 f"({target_cfg.TARGET_LATITUDE_DEG:.4f} deg N, "
                 f"{target_cfg.TARGET_LONGITUDE_DEG:.4f} deg E)")
    ax.grid(True)
    ax.legend()
    return _save(fig, output_dir, "target_sunlight")


# ---------------------------------------------------------------------------
# Power
# ---------------------------------------------------------------------------

def plot_solar_power(telemetry, output_dir):
    """Array generation per panel and in total, with the energy it collected.

    Returns ``None`` when the run had no power model, which
    :func:`write_plots` drops from its result.
    """
    if telemetry.total_power_w is None:
        return None

    times = telemetry.power_time_seconds
    peak = telemetry.peak_array_power_w
    observed_max = float(np.nanmax(telemetry.total_power_w))
    # Scale to what the array actually produced.  With the Sun well off the
    # panel normals the trace can sit at a tenth of the theoretical ceiling,
    # and an axis pinned to that ceiling flattens every variation in it -- the
    # variation being the whole reason to plot power against a maneuver.  The
    # ceiling only sets the scale when the run came close to reaching it.
    ceiling = (peak * 1.1 if observed_max >= peak * 0.5
               else max(observed_max * 1.25, 1.0))
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(9, 7))

    if telemetry.illumination_factor is not None:
        axes[0].fill_between(times, 0.0, ceiling,
                             where=telemetry.illumination_factor < 1.0,
                             color="0.88", zorder=0, label="Earth's shadow")
    for index, name in enumerate(telemetry.panel_names):
        axes[0].plot(times, telemetry.panel_powers_w[:, index], linewidth=1.0,
                     label=name)
    axes[0].plot(times, telemetry.total_power_w, linewidth=1.8,
                 color="#b26b00", label="array total")
    if ceiling >= peak:
        axes[0].axhline(peak, color="black", linestyle="--", linewidth=1,
                        label=f"normal incidence {peak:.0f} W")
    axes[0].set(ylabel="Power [W]",
                title=f"Solar-array generation "
                      f"({len(power_cfg.SOLAR_PANELS)} panels, "
                      f"{sum(p.area_m2 for p in power_cfg.SOLAR_PANELS):g} "
                      f"m$^2$, {peak:.0f} W at normal incidence)")
    axes[0].set_ylim(0.0, ceiling)

    if telemetry.stored_energy_wh is not None:
        axes[1].plot(times, telemetry.stored_energy_wh, color="#0f7b3d",
                     label="energy collected")
        axes[1].set_ylabel("Energy [W h]")
    else:
        axes[1].plot(times, telemetry.total_power_w, label="array total")
        axes[1].set_ylabel("Power [W]")
    axes[1].set_xlabel("Time [s]")

    for ax in axes:
        ax.grid(True)
        ax.legend(fontsize=8)
    return _save(fig, output_dir, "solar_power")


# ---------------------------------------------------------------------------
# Generated-controller modes
# ---------------------------------------------------------------------------

def plot_adcs_modes(telemetry, output_dir):
    """Desired-vs-actual ADCS manager states from the generated controller."""
    diagnostics = telemetry.diagnostics
    mode_time = telemetry.mode_time_seconds
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 8))

    axes[0].step(mode_time, diagnostics.desired_acs_history, where="post",
                 label="desired ACS")
    axes[0].step(mode_time, diagnostics.actual_acs_history, where="post",
                 label="actual ACS")
    axes[0].set_ylabel("ACS mode")

    axes[1].step(mode_time, diagnostics.desired_frame_history, where="post",
                 label="desired frame")
    axes[1].step(mode_time, diagnostics.actual_frame_history, where="post",
                 label="actual frame")
    axes[1].set_ylabel("Frame mode")

    axes[2].step(mode_time, diagnostics.mode_degraded_history, where="post",
                 label="mode degraded")
    axes[2].step(mode_time, diagnostics.desired_ads_history, where="post",
                 label="desired ADS")
    axes[2].step(mode_time, diagnostics.actual_ads_history, where="post",
                 label="actual ADS")
    axes[2].set(xlabel="Time [s]", ylabel="Status")

    for ax in axes:
        ax.grid(True)
        ax.legend()
    fig.suptitle("Generated ADCS manager states")
    return _save(fig, output_dir, "adcs_modes")


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

#: Figure name -> builder.  Add new figures here.
PLOT_BUILDERS = {
    "quaternion": plot_quaternion,
    "q_est": plot_q_est,
    "angular_rate": plot_angular_rate,
    "rw_angular_velocity": plot_rw_angular_velocity,
    "mtq_voltage": plot_mtq_voltage,
    "adcs_torques": plot_adcs_torques,
    "rw_tau_after_settling": plot_rw_torque_after_settling,
    "earth_pointing": plot_earth_pointing,
    "sun_pointing": plot_sun_pointing,
    "earth_target_pointing": plot_earth_target_pointing,
    "target_sunlight": plot_target_sunlight,
    "solar_power": plot_solar_power,
    "adcs_modes": plot_adcs_modes,
}


def write_plots(telemetry, output_dir, names=None):
    """Build every registered figure (or just ``names``) into ``output_dir``.

    Figures are left open so an interactive run can display them; call
    :func:`finish` afterwards.

    A builder that returns ``None`` -- because the run carries no telemetry for
    it, as with power on a run with the power model off -- is left out of the
    result rather than reported as a written file.
    """
    selected = PLOT_BUILDERS if names is None else {
        name: PLOT_BUILDERS[name] for name in names}
    written = {name: builder(telemetry, output_dir)
               for name, builder in selected.items()}
    return {name: path for name, path in written.items() if path is not None}


def finish(show_plots=False):
    """Display the open figures, or close them all."""
    if show_plots:
        plt.show()
    else:
        plt.close("all")
