"""Top-level scenario execution for both run modes."""

import os
from pathlib import Path

from Basilisk.utilities import macros

from adcs_sim.analysis import write_analysis
from adcs_sim.config import paths
from adcs_sim.config import simulation as sim_cfg
from adcs_sim.config import visualization as viz_cfg
from adcs_sim.models import reaction_wheels
from adcs_sim.scenario import (builder, live, recorders, visualization,
                               vizard_app)


#: Default for ``library_path`` and ``link_file``: decide from the
#: environment and the connector's link file (see :func:`resolve_wiring`).
AUTO = "auto"


def resolve_wiring(library_path=AUTO, link_file=AUTO):
    """``(library_path, link document or None)`` for a run.

    ``link_file`` AUTO takes :func:`paths.active_link_file` -- the file the
    connector saved last.  ``None`` uses the hand-written bridge.

    ``library_path`` AUTO takes, in order: ``ADCS_LIBRARY`` if set, the
    software the link file selects (its DLL, or none), the default DLL.
    """
    from adcs_sim.flightsoftware import link_runtime

    if link_file == AUTO:
        link_file = paths.active_link_file()
    doc = link_runtime.load_link(link_file) if link_file else None

    if library_path == AUTO:
        if os.environ.get("ADCS_LIBRARY") is not None:
            library_path = paths.ADCS_LIBRARY
        elif doc is not None:
            library_path = link_runtime.software_library(doc)
            if library_path is not None and not Path(library_path).is_file():
                raise FileNotFoundError(
                    f"The link file {doc.path} selects the flight software "
                    f"{library_path}, which does not exist. Select the DLL in "
                    "the connector and save, or pass --flight-software.")
        else:
            library_path = paths.ADCS_LIBRARY
    return library_path, doc


def run(show_plots=False, live_stream=False, initial_omega_rpm=None,
        initial_momentum_nms=None, output_dir=None,
        simulation_seconds=sim_cfg.SIMULATION_SECONDS,
        launch_vizard=None, vizard_executable=None, live_plot=True,
        library_path=AUTO, link_file=AUTO):
    """Run either a finite recorded simulation or an indefinite live stream.

    Parameters
    ----------
    show_plots : bool
        Display the figures after a file-mode run as well as saving them.
    live_stream : bool
        Stream to Vizard indefinitely instead of recording a fixed run.
    initial_omega_rpm : array_like, shape (3,), optional
        Initial wheel speeds [RPM].  Mutually exclusive with the next argument.
    initial_momentum_nms : array_like, shape (3,), optional
        Initial wheel momenta [N m s].  Defaults to the saturation limit.
    output_dir : str or pathlib.Path, optional
        Destination for plots and the Vizard recording.
    simulation_seconds : float
        Length of a file-mode run [s].
    launch_vizard : bool, optional
        Auto-launch the Vizard viewer in live mode.  Defaults to
        :data:`~adcs_sim.config.visualization.AUTO_LAUNCH_VIZARD`.  Ignored in
        file mode, which only writes a recording.
    vizard_executable : str or pathlib.Path, optional
        Explicit path to ``Vizard.exe``; otherwise auto-discovered.
    library_path : str or pathlib.Path or None
        Flight-software DLL to fly.  ``None`` flies with no flight software
        (zero actuator commands).  If the *default* DLL is missing the run
        falls back to that too, with a warning, so the simulator never
        depends on the ADCS software being present.  ``AUTO`` (default):
        ``ADCS_LIBRARY``, else the software chosen in the link file.
    link_file : str or pathlib.Path or None
        Connector link file that wires simulator and software.  ``AUTO``
        (default): the file the connector saved last.  ``None``: the
        hand-written bridge.

    Returns
    -------
    dict or None
        In file mode, a dict with the Vizard file and figure paths.  In live
        mode, ``None`` once the stream is stopped.
    """
    rw_omega_rpm = reaction_wheels.initial_speed_rpm(
        initial_omega_rpm, initial_momentum_nms)
    for line in reaction_wheels.describe_initial_state(rw_omega_rpm):
        print(line)

    library_path, link_doc = resolve_wiring(library_path, link_file)
    library_path = _resolve_library(library_path)
    bundle = builder.build_simulation(rw_omega_rpm,
                                     record_history=not live_stream,
                                     library_path=library_path,
                                     link=link_doc)
    for line in builder.describe(bundle):
        print(line)

    # Gains are written at Reset, which InitializeSimulation triggers, so they
    # are reported after the run starts -- see below.

    run_recorders = None if live_stream else recorders.attach_recorders(bundle)

    visualization.require_vizard_support()
    clock_sync = (visualization.add_clock_sync(bundle.sim, bundle.task_name)
                  if live_stream else None)

    resolved_output_dir = paths.resolve_output_dir(output_dir)
    viz_file = None if live_stream else paths.viz_file(resolved_output_dir)
    viz = visualization.enable_visualization(bundle, live_stream, viz_file)

    if live_stream:
        # Vizard must be listening before InitializeSimulation, because that is
        # where Basilisk's vizInterface connects out and blocks.
        return _run_live(bundle, viz, clock_sync, launch_vizard,
                         vizard_executable, live_plot)

    bundle.sim.InitializeSimulation()
    for line in bundle.controller.describe_gains():
        print(line)

    bundle.sim.ConfigureStopTime(macros.sec2nano(simulation_seconds))
    bundle.sim.ExecuteSimulation()

    for line in bundle.controller.diagnostics.summary_lines():
        print(line)
    if bundle.star_tracker_camera is not None:
        for line in bundle.star_tracker_camera.describe():
            print(line)

    plot_files = write_analysis(run_recorders, bundle.controller,
                               resolved_output_dir, show_plots)

    print(f"Vizard data written to {viz_file}")
    for plot_file in plot_files.values():
        print(f"Plot written to {plot_file}")
    return {"viz_file": viz_file, "plot_files": plot_files}


def _resolve_library(library_path):
    """The DLL to fly, or ``None`` for no flight software.

    Only the default location falls back, with a warning: a path the user
    asked for explicitly must exist, or the run stops with the usual error.
    """
    if library_path is None:
        return None
    if (not Path(library_path).is_file()
            and paths.ADCS_LIBRARY is not None
            and Path(library_path) == Path(paths.ADCS_LIBRARY)):
        print(f"WARNING: no ADCS flight software found at {library_path}.\n"
              "         Running with NO flight software: no control torque "
              "or force.\n"
              "         Pass --flight-software PATH (or set ADCS_LIBRARY) to "
              "fly a controller.")
        return None
    return library_path


def _run_live(bundle, viz, clock_sync, launch_vizard, vizard_executable,
              live_plot=True):
    """Start Vizard if asked, then initialise and stream indefinitely."""
    if launch_vizard is None:
        launch_vizard = viz_cfg.AUTO_LAUNCH_VIZARD

    if not launch_vizard:
        print(f"Auto-launch disabled. Start Vizard yourself and have it listen "
              f"on tcp://{viz_cfg.VIZ_SOCKET_ADDRESS}:{viz_cfg.VIZ_SOCKET_PORT}.")
        bundle.sim.InitializeSimulation()
        live.run_live_stream(bundle, viz, clock_sync, live_plot=live_plot)
        return None

    session = vizard_app.VizardSession(
        viz_cfg.VIZ_SOCKET_ADDRESS, viz_cfg.VIZ_SOCKET_PORT,
        executable=vizard_executable)
    try:
        session.start()
    except vizard_app.VizardNotFoundError as error:
        # Not fatal: Basilisk waits and retries, so a manually started Vizard
        # can still pick the stream up.
        print(f"{error}\nContinuing without auto-launch.")
        bundle.sim.InitializeSimulation()
        live.run_live_stream(bundle, viz, clock_sync, live_plot=live_plot)
        return None

    try:
        # Deliberately do NOT wait for Vizard's socket here by default. Vizard
        # start-up then overlaps InitializeSimulation, instead of running before
        # it, so Vizard's "waiting for messages" timer does not tick away while
        # Basilisk resets its models. Basilisk blocks at the end of
        # InitializeSimulation until Vizard appears, so it is the side that
        # should be made to wait.
        if viz_cfg.VIZARD_WAIT_FOR_SOCKET:
            session.wait_until_listening()

        # InitializeSimulation blocks in C++ until Vizard connects, and if
        # Vizard timed out first it blocks forever. The watchdog runs on
        # another thread and relaunches Vizard until the stream comes up, so
        # losing the start-up race costs a retry instead of the whole run.
        connected = session.start_watchdog()
        try:
            bundle.sim.InitializeSimulation()
        finally:
            connected.set()

        for line in bundle.controller.describe_gains():
            print(line)
        live.run_live_stream(bundle, viz, clock_sync, live_plot=live_plot)
    finally:
        session.stop()
    return None
