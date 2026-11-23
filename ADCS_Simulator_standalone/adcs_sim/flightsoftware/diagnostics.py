"""Mode-history and saturation bookkeeping for the generated controller.

Two tiers of bookkeeping, deliberately separated:

* **Always on** -- cheap scalar aggregates (state occupancy counts, peak raw
  ``RW_tau``).  These are printed at the end of every run, including live runs.
* **Opt-in** -- per-step mode histories used by the ``adcs_modes`` plot.  These
  grow without bound, so they are disabled for indefinite live streams.
"""

from dataclasses import dataclass, field

from Basilisk.utilities import macros


@dataclass
class ControllerDiagnostics:
    """Collect generated-controller state history during a run.

    Parameters
    ----------
    record_history : bool
        When False only the scalar aggregates are maintained.  Set False for
        ``--mode live``, whose run length is unbounded.
    """

    record_history: bool = True

    # -- always-on aggregates ---------------------------------------------
    acs_state_counts: dict = field(default_factory=dict)
    ephem_state_counts: dict = field(default_factory=dict)
    max_rw_tau: float = 0.0

    # -- opt-in per-step histories ----------------------------------------
    mode_time_seconds: list = field(default_factory=list)
    actual_acs_history: list = field(default_factory=list)
    desired_acs_history: list = field(default_factory=list)
    actual_frame_history: list = field(default_factory=list)
    desired_frame_history: list = field(default_factory=list)
    actual_ads_history: list = field(default_factory=list)
    desired_ads_history: list = field(default_factory=list)
    mode_degraded_history: list = field(default_factory=list)
    #: ``q_BI_est`` per step, as ``(q0, q1, q2, q3)`` tuples.
    q_est_history: list = field(default_factory=list)

    _HISTORY_FIELDS = (
        "mode_time_seconds",
        "actual_acs_history",
        "desired_acs_history",
        "actual_frame_history",
        "desired_frame_history",
        "actual_ads_history",
        "desired_ads_history",
        "mode_degraded_history",
        "q_est_history",
    )

    def reset(self):
        """Clear everything.  Called from the controller's ``Reset``."""
        self.acs_state_counts.clear()
        self.ephem_state_counts.clear()
        self.max_rw_tau = 0.0
        for name in self._HISTORY_FIELDS:
            getattr(self, name).clear()

    def update(self, current_sim_nanos, outputs, block_signals):
        """Sample the generated controller's state for this step.

        Parameters
        ----------
        current_sim_nanos : int
            Simulation time of this step [ns].
        outputs : structs.ADCSOutputs
            Live ``ExtY`` view, already updated by ``step()``.
        block_signals : structs.ADCSBlockSignals
            Live ``B`` view, already updated by ``step()``.
        """
        actual_acs = int(block_signals.actual_acs)
        actual_ephem = int(outputs.actual_ephem)

        self.acs_state_counts[actual_acs] = (
            self.acs_state_counts.get(actual_acs, 0) + 1)
        self.ephem_state_counts[actual_ephem] = (
            self.ephem_state_counts.get(actual_ephem, 0) + 1)
        # Peak of the *raw* controller output, before frame rotation and
        # clipping -- this is what reveals a diverging control law.
        self.max_rw_tau = max(self.max_rw_tau,
                              max(abs(value) for value in outputs.RW_tau))

        if not self.record_history:
            return

        self.mode_time_seconds.append(current_sim_nanos * macros.NANO2SEC)
        self.actual_acs_history.append(actual_acs)
        self.desired_acs_history.append(int(block_signals.desired_acs))
        self.actual_frame_history.append(int(block_signals.actual_frame))
        self.desired_frame_history.append(int(block_signals.desired_frame))
        self.actual_ads_history.append(int(outputs.actual_ads))
        self.desired_ads_history.append(int(block_signals.desired_ads))
        self.mode_degraded_history.append(int(outputs.mode_degraded))
        self.q_est_history.append(tuple(outputs.q_BI_est))

    def summary_lines(self):
        """Return the end-of-run aggregate report as printable lines."""
        return [
            f"Generated ACS state counts: {self.acs_state_counts}",
            f"Generated ephemeris state counts: {self.ephem_state_counts}",
            f"Generated peak RW_tau at controller rate: "
            f"{self.max_rw_tau:.6e} N m",
        ]
