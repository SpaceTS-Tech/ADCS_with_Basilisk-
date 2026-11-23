"""Post-run analysis: derived telemetry, plots and the printed summary.

=========================  =================================================
Module                     Contents
=========================  =================================================
:mod:`~.backend`           Matplotlib backend selection (import order matters).
:mod:`~.derived`           Turn recorder output into analysis-ready arrays.
:mod:`~.plots`             One function per figure, in a registry.
:mod:`~.summary`           The printed telemetry summary.
=========================  =================================================

Call :func:`write_analysis` after a file-mode run; it sequences the three
steps and returns the paths of the figures it wrote.
"""

from adcs_sim.analysis import derived as _derived

__all__ = ["backend", "derived", "plots", "summary", "write_analysis"]


def write_analysis(recorders, controller, output_dir, show_plots=False):
    """Derive telemetry, write every figure, and print the run summary.

    Parameters
    ----------
    recorders : adcs_sim.scenario.recorders.Recorders
        The recorder set from the completed run.
    controller : adcs_sim.flightsoftware.controller.GeneratedADCSController
        Source of the generated-controller mode histories.
    output_dir : pathlib.Path
        Directory the figures are written to.
    show_plots : bool
        Display the figures interactively as well as saving them.

    Returns
    -------
    dict
        Figure name -> written path.
    """
    # The matplotlib backend must be chosen before pyplot is first imported,
    # so select it here and import the plotting module afterwards.
    from adcs_sim.analysis import backend
    backend.select(interactive=show_plots)

    from adcs_sim.analysis import plots, summary

    telemetry = _derived.derive(recorders, controller)
    plot_files = plots.write_plots(telemetry, output_dir)
    summary.print_summary(telemetry)
    plots.finish(show_plots)
    return plot_files
