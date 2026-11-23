"""Command-line interface for the ADCS scenario."""

import argparse

from Basilisk.utilities import macros

from adcs_sim.config import actuators as act_cfg
from adcs_sim.config import paths
from adcs_sim.config import simulation as sim_cfg
from adcs_sim.scenario.runner import AUTO, run

#: Default wheel speed implied by the default initial momentum, for help text.
_DEFAULT_RPM = (act_cfg.DEFAULT_INITIAL_RW_MOMENTUM_NMS[0]
                / act_cfg.CONTROLLER_I_RW / macros.RPM)


def build_parser():
    """Return the scenario's argument parser."""
    parser = argparse.ArgumentParser(
        description="Run the generated ADCS controller in Basilisk.")
    parser.add_argument(
        "--mode", choices=("file", "live"), default="file",
        help=(f"file: run for {sim_cfg.SIMULATION_SECONDS:g} s and record a "
              f"Vizard file (default); live: stream indefinitely to Vizard "
              f"over the socket"))
    parser.add_argument(
        "--show-plots", action="store_true",
        help="display plots after file mode completes")
    parser.add_argument(
        "--output-dir", default=None, metavar="DIR",
        help=f"where plots and the Vizard recording are written "
             f"(default: {paths.OUTPUT_DIR})")
    parser.add_argument(
        "--seconds", type=float, default=sim_cfg.SIMULATION_SECONDS,
        metavar="T", help="length of a file-mode run [s]")
    parser.add_argument(
        "--no-vizard", dest="launch_vizard", action="store_false", default=None,
        help=("live mode: do not auto-launch Vizard; start it yourself and "
              "have it listen on the configured socket"))
    parser.add_argument(
        "--vizard-exe", default=None, metavar="PATH",
        help=("path to the Vizard executable; otherwise taken from the "
              "VIZARD_EXECUTABLE environment variable, then config, then a "
              "search of the usual install locations"))
    parser.add_argument(
        "--no-live-plot", dest="live_plot", action="store_false", default=True,
        help=("live mode: do not open the ACS-mode / wheel-momentum strip "
              "chart alongside Vizard"))

    software_group = parser.add_mutually_exclusive_group()
    software_group.add_argument(
        "--flight-software", default=AUTO, metavar="DLL",
        help=("ADCS flight-software DLL to fly (default: ADCS_LIBRARY if set, "
              "else the software selected in the connector's link file, else "
              f"{paths.ADCS_LIBRARY or 'none'}). "
              "'none' flies with no flight software"))
    software_group.add_argument(
        "--no-flight-software", dest="flight_software", action="store_const",
        const=None,
        help=("run the simulator without any ADCS software: no control "
              "torque or force, the spacecraft only feels the environment"))

    link_group = parser.add_mutually_exclusive_group()
    link_group.add_argument(
        "--link", dest="link_file", default=AUTO, metavar="FILE",
        help=("connector link file (.aocslink) wiring simulator and software "
              "(default: ADCS_LINK_FILE if set, else the file the connector "
              f"saved last, else {paths.DEFAULT_LINK_FILE.name})"))
    link_group.add_argument(
        "--no-link", dest="link_file", action="store_const", const=None,
        help="ignore link files and use the hand-written bridge "
             "(controller.py)")

    initial_rw_group = parser.add_mutually_exclusive_group()
    initial_rw_group.add_argument(
        "--rw-omega-rpm", nargs=3, type=float, metavar=("RW1", "RW2", "RW3"),
        help=(f"initial RW1/RW2/RW3 speeds in RPM; default is "
              f"{_DEFAULT_RPM:.0f} RPM on each wheel (the "
              f"{act_cfg.RW_PHYSICAL_H_MAX:g} N m s saturation limit)"))
    initial_rw_group.add_argument(
        "--rw-momentum", nargs=3, type=float, metavar=("RW1", "RW2", "RW3"),
        help=(f"initial RW1/RW2/RW3 momenta in N m s; default is "
              f"{act_cfg.RW_PHYSICAL_H_MAX:g} N m s on each wheel"))
    return parser


def main(argv=None):
    """Parse arguments and run the scenario."""
    arguments = build_parser().parse_args(argv)
    return run(
        show_plots=arguments.show_plots,
        live_stream=arguments.mode == "live",
        initial_omega_rpm=arguments.rw_omega_rpm,
        initial_momentum_nms=arguments.rw_momentum,
        output_dir=arguments.output_dir,
        simulation_seconds=arguments.seconds,
        launch_vizard=arguments.launch_vizard,
        vizard_executable=arguments.vizard_exe,
        live_plot=arguments.live_plot,
        library_path=_library_argument(arguments.flight_software),
        link_file=_link_argument(arguments.link_file),
    )


def _library_argument(value):
    """``None`` for 'none'/'off', AUTO kept, otherwise the path as given."""
    from adcs_sim.flightsoftware.library import is_no_flight_software
    if value == AUTO:
        return AUTO
    return None if is_no_flight_software(value) else value


def _link_argument(value):
    """``None`` for 'none'/'off', AUTO kept, otherwise the path as given."""
    if value is None or value == AUTO:
        return value
    return None if str(value).strip().lower() in ("none", "off", "no", "") \
        else value
