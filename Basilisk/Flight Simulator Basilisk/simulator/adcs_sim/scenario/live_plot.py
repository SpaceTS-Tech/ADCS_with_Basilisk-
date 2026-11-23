"""Real-time strip chart of the generated controller's ACS mode.

Opens a matplotlib window alongside Vizard during ``--mode live``.  It shows
which ACS state the flight software is actually in, reaction-wheel momentum
underneath, and -- when the power model is enabled -- solar-array generation
below that.  The three together make a maneuver legible: the mode trace says
what the controller thinks it is doing, the momentum trace says whether a
desaturation is really shedding momentum, and the power trace says what the
attitude the maneuver commands is costing in generation.

The mode is read from ``B.actual_acs`` -- the state the manager really settled
in, not the commanded mission mode.  Those differ exactly when it matters: a
pointing command sits in ``DESAT`` until momentum is shed, and can fall back to
``DETUMBLE`` or ``SAFE`` underneath any command.

The power trace is telemetry: the panels read the spacecraft state and the Sun
ephemeris, and nothing they produce reaches the flight software.

Cost control
------------
An indefinite live run cannot keep every sample or redraw every step, so:

* samples are taken at most every :data:`LIVE_PLOT_SAMPLE_SECONDS` of
  *simulation* time,
* the canvas is redrawn at most every
  :data:`~adcs_sim.config.visualization.LIVE_PLOT_REFRESH_SECONDS` of *wall
  clock* time,
* history is a bounded deque, so memory does not grow without limit.

If the backend cannot open a window the plotter disables itself and the run
continues -- a missing chart must never take the simulation down with it.
"""

import math
import time
from collections import deque

from adcs_sim.config import actuators as act_cfg
from adcs_sim.config import power as power_cfg
from adcs_sim.config import visualization as viz_cfg

#: ACS state numbering of the generated ``ACS_Manager`` -- the ``actual_acs``
#: value each state writes (model 2.95; HOLD, SLEW_SLOW and SLEW_FAST removed).
ACS_STATE_NAMES = {
    0: "IDLE", 1: "DETUMBLE", 2: "POINT", 3: "SPIN", 4: "DESAT",
    5: "OPTICAL", 6: "SAFE",
}

#: Simulation-time interval between samples [s].
LIVE_PLOT_SAMPLE_SECONDS = 1.0


def peak_array_power_w():
    """Array output at normal incidence, 1 AU, full sun [W]."""
    return sum(power_cfg.SOLAR_CONSTANT_1AU_W_M2 * panel.area_m2
               * panel.efficiency for panel in power_cfg.SOLAR_PANELS)


class LiveModePlot:
    """Strip chart of ACS mode, wheel momentum, array power and attitude error.

    Every panel has its own switch in :mod:`adcs_sim.config.visualization`.
    """

    def __init__(self, window_seconds=None, refresh_seconds=None,
                 show_power=None, show_q_est=None, show_mode=None,
                 show_momentum=None):
        self.show_mode = (viz_cfg.SHOW_LIVE_MODE_PLOT
                          if show_mode is None else show_mode)
        self.show_momentum = (viz_cfg.SHOW_LIVE_MOMENTUM_PLOT
                              if show_momentum is None else show_momentum)
        self.window_seconds = (viz_cfg.LIVE_PLOT_WINDOW_SECONDS
                               if window_seconds is None else window_seconds)
        self.refresh_seconds = (viz_cfg.LIVE_PLOT_REFRESH_SECONDS
                                if refresh_seconds is None else refresh_seconds)
        self.show_power = ((viz_cfg.SHOW_LIVE_POWER_PLOT
                            and power_cfg.POWER_MODEL_ENABLED)
                           if show_power is None else show_power)
        self.show_q_est = (viz_cfg.SHOW_LIVE_Q_EST_PLOT
                           if show_q_est is None else show_q_est)
        self.enabled = False

        # Bound the history so an indefinite run cannot exhaust memory.
        depth = (None if not self.window_seconds
                 else int(self.window_seconds / LIVE_PLOT_SAMPLE_SECONDS) + 2)
        self._time = deque(maxlen=depth)
        self._mode = deque(maxlen=depth)
        self._momentum = [deque(maxlen=depth) for _ in range(3)]
        self._power = deque(maxlen=depth)
        self._panel_power = [deque(maxlen=depth)
                             for _ in power_cfg.SOLAR_PANELS]
        self._illumination = deque(maxlen=depth)
        self._energy_wh = 0.0
        #: Error quaternion of the estimate relative to truth, scalar first.
        self._q_err = [deque(maxlen=depth) for _ in range(4)]

        self._next_sample_s = 0.0
        self._last_draw = 0.0
        self._figure = None
        self._sun_line = None

    # -- lifecycle ---------------------------------------------------------

    def open(self):
        """Create the window.  Returns self; disables itself on any failure."""
        try:
            import matplotlib.pyplot as plt

            plt.ion()
            self._plt = plt
            panels = [name for name, shown in (
                ("ACS mode", self.show_mode),
                ("wheel momentum", self.show_momentum),
                ("array power", self.show_power),
                ("q_BI_est error", self.show_q_est),
            ) if shown]
            if not panels:
                print("Live plot: every panel is switched off; not opening.")
                return self
            figure, axes = plt.subplots(
                len(panels), 1, figsize=(7.5, 2.7 * len(panels)),
                sharex=True, squeeze=False)
            axes = list(axes[:, 0])
            figure.canvas.manager.set_window_title(
                "ADCS live -- " + ", ".join(panels))
            self._figure = figure
            self._axes = axes
            panel_axes = iter(axes)

            if self.show_mode:
                self._open_mode_axis(next(panel_axes))
            if self.show_momentum:
                self._open_momentum_axis(next(panel_axes))
            if self.show_power:
                self._open_power_axis(next(panel_axes))
            if self.show_q_est:
                self._q_panel = self._open_q_err_axis(
                    next(panel_axes),
                    "Controller output 7: q_err = q_BI_est (-) q_BI")
            # Time and mission mode head the whole figure.
            self._top_title = figure.suptitle("", fontsize=10)

            axes[-1].set_xlabel("simulation time [s]")
            # Leave headroom for the suptitle and two-line panel titles.
            figure.tight_layout(rect=(0, 0, 1, 0.95), h_pad=2.0)
            figure.show()
            self.enabled = True
        except Exception as error:                       # noqa: BLE001
            print(f"Live ACS plot unavailable ({type(error).__name__}: "
                  f"{error}); continuing without it.")
            self.enabled = False
        return self

    def _open_mode_axis(self, mode_ax):
        """Draw the empty ACS-mode panel."""
        self._mode_ax = mode_ax
        (self._mode_line,) = mode_ax.plot(
            [], [], lw=1.6, color="#1b4f8f", drawstyle="steps-post")
        mode_ax.set_ylabel("ACS state")
        mode_ax.set_ylim(-0.5, max(ACS_STATE_NAMES) + 0.5)
        mode_ax.set_yticks(sorted(ACS_STATE_NAMES))
        mode_ax.set_yticklabels(
            [f"{value} {name}" for value, name in
             sorted(ACS_STATE_NAMES.items())], fontsize=8)
        mode_ax.grid(alpha=.3)

    def _open_momentum_axis(self, momentum_ax):
        """Draw the empty wheel-momentum panel."""
        self._momentum_lines = [
            momentum_ax.plot([], [], lw=1.2, label=f"RW{index + 1}")[0]
            for index in range(3)]
        entry = act_cfg.DESAT_ENTRY_FRACTION * act_cfg.CONTROLLER_H_MAX
        exit_band = act_cfg.DESAT_EXIT_FRACTION * act_cfg.CONTROLLER_H_MAX
        momentum_ax.axhline(entry, ls="--", lw=1, color="#b3261e",
                            label=f"desat entry {entry:g}")
        momentum_ax.axhline(exit_band, ls="--", lw=1, color="#0f7b3d",
                            label=f"desat exit {exit_band:g}")
        momentum_ax.set_ylabel("|h| per wheel [N m s]")
        momentum_ax.set_ylim(0, act_cfg.CONTROLLER_H_MAX * 1.1)
        momentum_ax.grid(alpha=.3)
        momentum_ax.legend(fontsize=7.5, ncol=2, loc="upper right")

    def _open_power_axis(self, power_ax):
        """Draw the empty array-power panel, and its sunlit-fraction twin."""
        self._power_ax = power_ax
        self._panel_lines = [
            power_ax.plot([], [], lw=1.0, ls="--", label=panel.name)[0]
            for panel in power_cfg.SOLAR_PANELS]
        (self._power_line,) = power_ax.plot([], [], lw=1.8, color="#b26b00",
                                            label="array total")
        peak = peak_array_power_w()
        power_ax.set_ylabel("Array power [W]")
        # Starting scale only; _redraw_power rescales to what the array is
        # really producing, which is often far below the ceiling.
        power_ax.set_ylim(0, peak * 1.1)
        power_ax.grid(alpha=.3)
        power_ax.legend(fontsize=7.5, ncol=3, loc="upper right")
        self._power_title = power_ax.set_title("", fontsize=9)

        if power_cfg.POWER_ECLIPSE_ENABLED:
            # Right-hand axis rather than shading: a fill has to be rebuilt on
            # every redraw, a line only has its data replaced.
            sun_ax = power_ax.twinx()
            (self._sun_line,) = sun_ax.plot([], [], lw=1.0, color="#3d7dbf",
                                            alpha=.7)
            sun_ax.set_ylabel("sunlit", fontsize=8)
            sun_ax.set_ylim(-0.05, 1.05)
            sun_ax.tick_params(labelsize=7)

    def _open_q_err_axis(self, q_ax, label):
        """Draw one empty error-quaternion panel and return its handles.

        Plots ``q_err`` with ``C(q_err) = C(q_est) C(q_true)^T`` -- the
        rotation from the true body frame to the estimated one.  A perfect
        estimate is the identity ``[1, 0, 0, 0]``: ``q0`` sits on 1 and the
        vector part on 0.

        All four components share one fixed ``[-1, 1]`` axis, so the unit
        quaternion reads directly.  An arcsecond-level error therefore draws as
        flat lines at 1 and 0; the title carries the exact values and angle.
        """
        colors = ("#1b4f8f", "#b3261e", "#0f7b3d", "#b26b00")
        lines = [
            q_ax.plot([], [], lw=1.4, color=color, label=f"q{index} err")[0]
            for index, color in enumerate(colors)]
        # The zero-error identity, for reference.
        q_ax.axhline(1.0, ls="--", lw=0.8, color="grey")
        q_ax.axhline(0.0, ls="--", lw=0.8, color="grey")
        q_ax.set_ylabel("q_err")
        q_ax.set_ylim(-1.05, 1.05)
        q_ax.grid(alpha=.3)
        q_ax.legend(fontsize=7.5, ncol=4, loc="lower right")
        status = q_ax.set_title(label, fontsize=9)
        return {"lines": lines, "status": status, "label": label}

    def close(self):
        """Close the window, leaving it up if it was never opened."""
        if not self.enabled or self._figure is None:
            return
        try:
            self._plt.close(self._figure)
        except Exception:                                # noqa: BLE001
            pass
        self.enabled = False

    # -- per-step ----------------------------------------------------------

    def update(self, time_s, acs_mode, wheel_momentum, mission_cmd=None,
               power=None, q_est=None, q_true=None):
        """Sample and, if due, redraw.  Safe to call every control step.

        ``power`` is the dict :func:`adcs_sim.scenario.live._power_sample`
        builds, or ``None`` when the power model is off.  A missing sample is
        stored as NaN so the power traces stay aligned with the time axis.
        ``q_est`` / ``q_true`` are the 4-element ``Y.q_BI_est`` and ``U.q_BI``,
        scalar first; missing ones are stored as NaN the same way.
        """
        if not self.enabled or time_s < self._next_sample_s:
            return
        self._next_sample_s = time_s + LIVE_PLOT_SAMPLE_SECONDS

        self._time.append(time_s)
        self._mode.append(int(acs_mode))
        for index in range(3):
            self._momentum[index].append(abs(float(wheel_momentum[index])))
        if self.show_power:
            self._sample_power(power)
        if self.show_q_est:
            q_err = error_quaternion(q_est, q_true)
            for index in range(4):
                self._q_err[index].append(q_err[index])

        now = time.monotonic()
        if now - self._last_draw < self.refresh_seconds:
            return
        self._last_draw = now
        self._redraw(mission_cmd)

    def _sample_power(self, power):
        """Append one power sample, or NaN when there is nothing to read."""
        missing = float("nan")
        self._power.append(missing if power is None
                           else float(power["total_w"]))
        panels = () if power is None else power["panels_w"]
        for index, samples in enumerate(self._panel_power):
            samples.append(float(panels[index]) if index < len(panels)
                           else missing)
        self._illumination.append(missing if power is None
                                  else float(power["illumination"]))
        if power is not None:
            self._energy_wh = float(power["energy_wh"])

    def _redraw(self, mission_cmd):
        try:
            times = list(self._time)
            if self.show_mode:
                self._mode_line.set_data(times, list(self._mode))
            if self.show_momentum:
                for index, line in enumerate(self._momentum_lines):
                    line.set_data(times, list(self._momentum[index]))
            if self.show_power:
                self._redraw_power(times)
            if self.show_q_est:
                self._redraw_q_err(self._q_panel, self._q_err, times)

            if times:
                self._axes[0].set_xlim(times[0], max(times[-1], times[0] + 1))
                heading = ("" if mission_cmd is None
                           else f"mission cmd {mission_cmd}   ")
                if self.show_mode:
                    state = self._mode[-1]
                    heading += (f"ACS = {state} "
                                f"{ACS_STATE_NAMES.get(state, '?')}   ")
                self._top_title.set_text(f"{heading}t = {times[-1]:.0f} s")

            self._figure.canvas.draw_idle()
            self._figure.canvas.flush_events()
        except Exception as error:                       # noqa: BLE001
            # A closed window must not abort the simulation.
            print(f"Live ACS plot stopped ({type(error).__name__}: {error}).")
            self.enabled = False

    def _redraw_power(self, times):
        """Update the array-power traces and their headline figures."""
        self._power_line.set_data(times, list(self._power))
        for index, line in enumerate(self._panel_lines):
            line.set_data(times, list(self._panel_power[index]))
        if self._sun_line is not None:
            self._sun_line.set_data(times, list(self._illumination))

        if not self._power:
            return

        # Rescale to the power actually seen so far.  An axis pinned to the
        # theoretical ceiling draws a flat line along the bottom whenever the
        # Sun is well off the panel normals, hiding the very changes a
        # maneuver causes.  The ceiling sets the scale only once the array
        # gets close to it.
        peak = peak_array_power_w()
        observed = [value for value in self._power if value == value]
        if observed:
            highest = max(observed)
            self._power_ax.set_ylim(
                0, peak * 1.1 if highest >= peak * 0.5
                else max(highest * 1.25, 1.0))

        current = self._power[-1]
        sunlit = self._illumination[-1] if self._illumination else 1.0
        self._power_title.set_text(
            f"array {current:.0f} W of {peak:.0f} W peak   "
            f"energy {self._energy_wh:.2f} W h   sunlit {sunlit:.2f}")


    def _redraw_q_err(self, panel, q_err_history, times):
        """Update one error-quaternion panel and its error angle."""
        for index in range(4):
            panel["lines"][index].set_data(times, list(q_err_history[index]))
        if not q_err_history[0]:
            return

        q_err = [samples[-1] for samples in q_err_history]
        if q_err[0] == q_err[0]:
            # q_err is normalised with q0 >= 0, so this is the shortest
            # rotation between estimate and truth.
            error_arcsec = math.degrees(
                2.0 * math.acos(min(q_err[0], 1.0))) * 3600.0
            error_text = f"error {error_arcsec:.1f} arcsec"
        else:
            error_text = "no estimate"
        values = ", ".join(f"{value:+.6f}" for value in q_err)
        panel["status"].set_text(
            f"{panel['label']}\n[{values}]   {error_text}")


def error_quaternion(q_est, q_true):
    """Error quaternion of ``q_est`` relative to ``q_true``, scalar first.

    ``C(q_err) = C(q_est) C(q_true)^T`` -- Basilisk's ``subEP(q_est, q_true)``
    -- normalised and sign-fixed to ``q0 >= 0`` (``q`` and ``-q`` are the same
    attitude), so zero error is exactly ``[1, 0, 0, 0]``.  Returns four NaNs
    when either input is missing, non-finite or zero.
    """
    missing = [float("nan")] * 4
    if q_est is None or q_true is None:
        return missing
    est = [float(value) for value in q_est]
    true = [float(value) for value in q_true]
    est_norm = math.sqrt(sum(value * value for value in est))
    true_norm = math.sqrt(sum(value * value for value in true))
    if not (est_norm > 0 and true_norm > 0
            and math.isfinite(est_norm) and math.isfinite(true_norm)):
        return missing
    e0, e1, e2, e3 = (value / est_norm for value in est)
    t0, t1, t2, t3 = (value / true_norm for value in true)
    # Same expression as RigidBodyKinematics.subEP(est, true).
    q_err = [
        t0 * e0 + t1 * e1 + t2 * e2 + t3 * e3,
        -t1 * e0 + t0 * e1 + t3 * e2 - t2 * e3,
        -t2 * e0 - t3 * e1 + t0 * e2 + t1 * e3,
        -t3 * e0 + t2 * e1 - t1 * e2 + t0 * e3,
    ]
    if q_err[0] < 0:
        q_err = [-value for value in q_err]
    return q_err


def create():
    """Return an opened plotter, or ``None`` when disabled by config."""
    if not viz_cfg.SHOW_LIVE_ACS_PLOT:
        return None
    return LiveModePlot().open()
