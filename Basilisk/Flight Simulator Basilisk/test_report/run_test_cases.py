"""Execute the test cases declared in ``ADCS_TEST_CASES.html`` and fill it in.

The HTML document is the **source of truth** for what gets run: this script
parses its ``CASES`` array, reconstructs each case's initial conditions from
the declared attitude/wheel state, runs it, produces only the figures that
evidence that case's criteria, and writes the outcome back into the document.

Workflow
--------
.. code-block:: text

    python run_test_cases.py                  # every case, in order
    python run_test_cases.py --case TC_01     # one case
    python run_test_cases.py --from TC_07     # resume from a case onward
    python run_test_cases.py --list           # show what would run

The HTML is rewritten **after each case completes**, so the document can be
kept open in a browser and refreshed to watch results land one at a time.

How the initial state is reconstructed
--------------------------------------
Section 3.0 of the document defines the state labels; this script is the other
half of that contract:

=================  ==========================================================
Label              Applied to the simulation
=================  ==========================================================
``TUMBLED``        ``omega_BN_BInit`` = (0.03, -0.02, 0.04) rad/s
``DETUMBLED``      ``omega_BN_BInit`` = 0
``OFF-POINTING``   detumbled, attitude rotated 8 deg about body X
``SATURATED``      every wheel at ``CONTROLLER_H_MAX`` (50 N m s)
``DESATURATED``    every wheel at rest
=================  ==========================================================

Change a label in the HTML and the run changes with it.

Plot selection
--------------
Each case gets only the figures that evidence its own criteria, not a fixed
set.  A saturated pointing case needs the ACS-mode trace (to show ``DESAT``
runs first) and the momentum trace (to show momentum is shed); a detumble case
needs the body rate.  See :func:`plots_for`.
"""

import argparse
import json
import re
import sys
from datetime import date
from pathlib import Path

import numpy as np

_DIR = Path(__file__).resolve().parent
#: ``adcs_sim`` lives in ``../simulator``, ``test_mission_modes`` in
#: ``../regression_tests``; this script drives both.
for _import_dir in (_DIR, _DIR.parent / "simulator", _DIR.parent / "regression_tests"):
    if str(_import_dir) not in sys.path:
        sys.path.insert(0, str(_import_dir))

import matplotlib                                              # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                # noqa: E402

import test_mission_modes as suite                             # noqa: E402
from adcs_sim.config import actuators as act_cfg               # noqa: E402
from adcs_sim.config import paths as sim_paths                 # noqa: E402

HTML_FILE = _DIR / "ADCS_TEST_CASES.html"
PLOT_ROOT = _DIR / "plots"

#: Body rate applied for a TUMBLED start [rad/s].
TUMBLE_RATE = (0.03, -0.02, 0.04)
#: Attitude offset applied for an OFF-POINTING start [deg], about body X.
OFFPOINT_DEG = 8.0

#: Mission command -> (expected ACS state, boresight reference).
COMMAND_MAP = {
    1: (suite.ACS_DETUMBLE, None),
    4: (suite.ACS_POINT, "nadir"),
    5: (suite.ACS_POINT, "sun"),
    6: (suite.ACS_POINT, "target"),
    9: (suite.ACS_OPTICAL, None),
}

POINTING_LABEL = {"nadir": "nadir", "sun": "Sun", "target": "ground target"}


# ---------------------------------------------------------------------------
# Reading the document
# ---------------------------------------------------------------------------
def parse_cases(html_path=HTML_FILE):
    """Return the case definitions declared in the HTML.

    Reads the fields this script needs -- ``id``, ``cmd``, ``dur``,
    ``attitude``, ``wheels``, ``title`` -- from the ``CASES`` array.  A
    tolerant field-wise scan rather than a JS parser, so reformatting the
    array will not break it as long as the ``key:value`` spellings hold.
    """
    text = html_path.read_text(encoding="utf-8")
    start = text.index("const CASES = [")
    end = text.index("\n];", start)
    block = text[start:end]

    cases = []
    for chunk in re.finditer(r'\{id:"(TC_\d+)"(.*?)(?=\n \{id:"|\Z)',
                             block, re.S):
        case_id, body = chunk.group(1), chunk.group(2)

        def field(name, pattern=r'"([^"]*)"'):
            found = re.search(rf'\b{name}:\s*{pattern}', body)
            return found.group(1) if found else None

        cases.append({
            "id": case_id,
            "title": field("title"),
            "cmd": int(field("cmd", r"(\d+)")),
            "dur": float(field("dur", r"([\d.]+)")),
            "attitude": field("attitude"),
            "wheels": field("wheels"),
        })
    if not cases:
        raise RuntimeError(f"No TC_ definitions found in {html_path}")
    return cases


def to_mission_case(spec):
    """Turn a parsed HTML case into a runnable :class:`suite.MissionCase`."""
    expect_acs, pointing = COMMAND_MAP[spec["cmd"]]
    tumbled = spec["attitude"] == "TUMBLED"
    offpoint = spec["attitude"] == "OFF-POINTING"
    saturated = spec["wheels"] == "SATURATED"

    return suite.MissionCase(
        name=f"{spec['id']} {spec['title']}",
        mission_cmd=spec["cmd"],
        seconds=spec["dur"],
        expect_acs=expect_acs,
        initial_rate_rad_s=TUMBLE_RATE if tumbled else (0.0, 0.0, 0.0),
        initial_momentum_nms=act_cfg.CONTROLLER_H_MAX if saturated else 0.0,
        pointing_target=pointing,
        # A saturated run spends its first phase shedding momentum, so its
        # pointing accuracy is not a pass criterion -- ordering is.
        assert_pointing=bool(pointing) and not saturated,
        max_pointing_error_deg=5.0 if pointing == "nadir" else 15.0,
        needs_camera=spec["cmd"] == 9,
        initial_attitude_offset_deg=OFFPOINT_DEG if offpoint else 0.0,
        offset_axis=1,
        expect_acquisition=offpoint,
    )


# ---------------------------------------------------------------------------
# Plots -- only what each case needs to evidence its criteria
# ---------------------------------------------------------------------------
def plots_for(spec):
    """Return the figure names this case should produce."""
    wanted = []
    if spec["wheels"] == "SATURATED":
        # The whole point of the case: DESAT runs first, momentum comes down.
        wanted += ["acs_mode", "wheel_momentum"]
    if spec["cmd"] == 1 or spec["attitude"] == "TUMBLED":
        wanted.append("body_rate")
    if spec["cmd"] in (4, 5, 6):
        wanted.append("pointing_error")
    if spec["cmd"] == 9:
        wanted.append("centroid")
    if "acs_mode" not in wanted:
        wanted.append("acs_mode")
    return wanted


#: Figure geometry.  The aspect ratio is fixed so every plot tiles uniformly
#: in the document's grid; the DPI is high enough that click-to-enlarge still
#: looks sharp on a full-width screen.
FIG_SIZE = (6.4, 3.1)
FIG_DPI = 160


def _finish(fig, ax, path, title):
    ax.set_title(title, fontsize=10)
    ax.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(path, dpi=FIG_DPI)
    plt.close(fig)


def plot_acs_mode(result, path):
    """ACS state against time -- shows DESAT preceding the commanded mode."""
    diagnostics = result.telemetry.diagnostics
    time_s = np.asarray(diagnostics.mode_time_seconds)
    states = np.asarray(diagnostics.actual_acs_history)

    fig, ax = plt.subplots(figsize=FIG_SIZE)
    ax.step(time_s, states, where="post", lw=1.4, color="#1b4f8f")
    present = sorted(set(int(s) for s in states))
    ax.set_yticks(present)
    ax.set_yticklabels([f"{s} {suite.ACS_NAMES.get(s, '?')}" for s in present],
                       fontsize=8)
    ax.set_xlabel("time [s]")
    ax.set_ylim(min(present) - .5, max(present) + .5)
    _finish(fig, ax, path, "Generated-controller ACS state")


def plot_wheel_momentum(result, path):
    """Per-wheel stored momentum with the desaturation bands drawn in."""
    telemetry = result.telemetry
    time_s = telemetry.rw_time_seconds
    momentum = act_cfg.CONTROLLER_I_RW * np.abs(telemetry.wheel_speeds)

    fig, ax = plt.subplots(figsize=FIG_SIZE)
    for index in range(momentum.shape[1]):
        ax.plot(time_s, momentum[:, index], lw=1.2, label=f"RW{index + 1}")
    entry = act_cfg.DESAT_ENTRY_FRACTION * act_cfg.CONTROLLER_H_MAX
    exit_band = act_cfg.DESAT_EXIT_FRACTION * act_cfg.CONTROLLER_H_MAX
    ax.axhline(entry, ls="--", lw=1, color="#b3261e",
               label=f"desat entry {entry:g}")
    ax.axhline(exit_band, ls="--", lw=1, color="#0f7b3d",
               label=f"desat exit {exit_band:g}")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("|h| per wheel [N m s]")
    ax.legend(fontsize=7.5, ncol=2)
    _finish(fig, ax, path, "Reaction-wheel stored momentum")


def plot_body_rate(result, path):
    """Body-rate magnitude -- the detumble evidence."""
    telemetry = result.telemetry
    fig, ax = plt.subplots(figsize=FIG_SIZE)
    ax.plot(telemetry.time_seconds, telemetry.body_rate_norm,
            lw=1.3, color="#1b4f8f")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("|omega_BN_B| [rad/s]")
    initial = telemetry.body_rate_norm[0]
    if initial > 0:
        ax.axhline(0.2 * initial, ls="--", lw=1, color="#0f7b3d",
                   label="20% of initial (pass threshold)")
        ax.legend(fontsize=8)
    _finish(fig, ax, path, "Body-rate magnitude")


def plot_pointing_error(result, path):
    """Boresight angle to the commanded reference -- must fall to ~zero."""
    telemetry = result.telemetry
    target = result.case.pointing_target
    direction = {"nadir": telemetry.earth_direction_B,
                 "sun": telemetry.sun_direction_B,
                 "target": telemetry.target_direction_B}[target]
    angle = suite.boresight_angle_deg(direction)

    fig, ax = plt.subplots(figsize=FIG_SIZE)
    ax.plot(telemetry.time_seconds, angle, lw=1.3, color="#1b4f8f")
    limit = result.case.max_pointing_error_deg
    ax.axhline(limit, ls="--", lw=1, color="#0f7b3d",
               label=f"pass threshold {limit:g} deg")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("angle: +Z boresight to reference [deg]")
    ax.legend(fontsize=8)
    _finish(fig, ax, path,
            f"Pointing error to {POINTING_LABEL[target]}")


def plot_centroid(result, path):
    """Tracked-target offset in the image plane."""
    time_s, x_cm, y_cm = result.feed.centroid_offsets()
    radius = np.hypot(x_cm, y_cm)

    fig, ax = plt.subplots(figsize=FIG_SIZE)
    ax.plot(time_s, x_cm, lw=1, alpha=.8, label="x_cm (column)")
    ax.plot(time_s, y_cm, lw=1, alpha=.8, label="y_cm (row)")
    ax.plot(time_s, radius, lw=1.5, color="#1b4f8f", label="radius")
    ax.axhline(suite.OPTICAL_HOLD_TOLERANCE_PX, ls="--", lw=1, color="#0f7b3d",
               label=f"hold tolerance {suite.OPTICAL_HOLD_TOLERANCE_PX:g} px")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("offset from frame centre [px]")
    ax.legend(fontsize=7.5, ncol=2)
    _finish(fig, ax, path, "Optical centroid offset")


PLOT_FUNCS = {
    "acs_mode": plot_acs_mode,
    "wheel_momentum": plot_wheel_momentum,
    "body_rate": plot_body_rate,
    "pointing_error": plot_pointing_error,
    "centroid": plot_centroid,
}


def make_plots(spec, result):
    """Render this case's figures.  Returns the filenames written."""
    out_dir = PLOT_ROOT / spec["id"]
    out_dir.mkdir(parents=True, exist_ok=True)
    written = []
    for name in plots_for(spec):
        try:
            PLOT_FUNCS[name](result, out_dir / f"{name}.png")
            written.append(f"{name}.png")
        except Exception as error:                      # noqa: BLE001
            print(f"    ! plot '{name}' failed: "
                  f"{type(error).__name__}: {error}")
    return written


# ---------------------------------------------------------------------------
# Writing the document
# ---------------------------------------------------------------------------
def summarise(result):
    """One-paragraph 'actual result' text built from the measured values."""
    if result.error:
        return f"Run failed: {result.error}"

    lines = []
    for label, ok, detail in result.checks:
        lines.append(f"{'PASS' if ok else 'FAIL'}  {label}: {detail}")

    metrics = result.metrics
    interesting = [
        ("ACS occupancy", metrics.get("acs_occupancy")),
        ("body rate initial -> final [rad/s]",
         _pair(metrics, "rate_initial_rad_s", "rate_final_rad_s")),
        ("wheel |h| initial -> final [N m s]",
         _pair(metrics, "wheel_h_initial_nms", "wheel_h_final_nms")),
        ("boresight error initial -> final [deg]",
         _pair(metrics, "boresight_err_initial_deg",
               "boresight_err_final_deg")),
        ("best aligned axis", metrics.get("best_aligned_axis")),
        ("centroid radius initial -> final [px]",
         _pair(metrics, "centroid_r_initial_px", "centroid_r_final_px")),
        ("frames with target visible", metrics.get("frames_target_visible")),
    ]
    lines.append("")
    for label, value in interesting:
        if value is not None:
            lines.append(f"{label}: {value}")
    return "\n".join(lines)


def _pair(metrics, first, second):
    """Format two related metrics as 'a -> b', or None if absent."""
    if first not in metrics or second not in metrics:
        return None
    return f"{metrics[first]:.4g} -> {metrics[second]:.4g}"


def load_results(html_path=HTML_FILE):
    """Read the currently embedded RESULTS object."""
    text = html_path.read_text(encoding="utf-8")
    found = re.search(r'window\.RESULTS = (\{.*?\});\n</script>', text, re.S)
    if not found:
        raise RuntimeError("results-data block not found in the HTML")
    return json.loads(found.group(1))


def write_results(data, html_path=HTML_FILE):
    """Replace the embedded RESULTS object, leaving the rest untouched."""
    text = html_path.read_text(encoding="utf-8")
    payload = json.dumps(data, indent=1, sort_keys=True)
    text, count = re.subn(r'window\.RESULTS = \{.*?\};\n</script>',
                          lambda _: f"window.RESULTS = {payload};\n</script>",
                          text, count=1, flags=re.S)
    if count != 1:
        raise RuntimeError("failed to substitute the results-data block")
    html_path.write_text(text, encoding="utf-8")


def record(spec, result, plot_files, html_path=HTML_FILE):
    """Merge one case's outcome into the document."""
    data = load_results(html_path)
    case_id = spec["id"]
    if result.error:
        status = "BLOCKED"
    else:
        status = "PASS" if result.passed else "FAIL"

    data[f"{case_id}.status"] = status
    data[f"{case_id}.actual"] = summarise(result)
    data[f"{case_id}.rundate"] = date.today().isoformat()
    data[f"{case_id}.dll"] = dll_stamp()
    data[f"{case_id}.plots"] = ", ".join(plot_files)
    data[f"sum.{case_id}.date"] = date.today().isoformat()
    write_results(data, html_path)
    return status


def dll_stamp():
    """Identify the binary under test, so a result is traceable to a build."""
    library = Path(sim_paths.ADCS_LIBRARY)
    if not library.is_file():
        return "DLL not found"
    from datetime import datetime
    built = datetime.fromtimestamp(library.stat().st_mtime)
    return f"{library.name} {built:%Y-%m-%d %H:%M:%S} ({library.stat().st_size} B)"


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------
def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Run the test cases declared in ADCS_TEST_CASES.html "
                    "and fill the document in.")
    parser.add_argument("--case", action="append", metavar="TC_ID",
                        help="run only this case (repeatable)")
    parser.add_argument("--from", dest="start", metavar="TC_ID",
                        help="run from this case onward")
    parser.add_argument("--list", action="store_true",
                        help="list the cases and exit")
    args = parser.parse_args(argv)

    specs = parse_cases()
    if args.case:
        wanted = {c.upper() for c in args.case}
        specs = [s for s in specs if s["id"] in wanted]
    elif args.start:
        ids = [s["id"] for s in specs]
        if args.start.upper() not in ids:
            print(f"No such case: {args.start}")
            return 2
        specs = specs[ids.index(args.start.upper()):]

    if args.list:
        for spec in specs:
            print(f"{spec['id']}  cmd {spec['cmd']}  {spec['dur']:>6.0f} s  "
                  f"{spec['attitude']:<13} {spec['wheels']:<12} "
                  f"{spec['title']}")
        return 0

    if not specs:
        print("No cases selected.")
        return 2

    print(f"Document : {HTML_FILE}")
    print(f"Binary   : {dll_stamp()}")
    print(f"Cases    : {len(specs)}\n")

    failures = 0
    for index, spec in enumerate(specs, 1):
        print(f"[{index}/{len(specs)}] {spec['id']}  {spec['title']}")
        print(f"    start state : {spec['attitude']} / {spec['wheels']}, "
              f"{spec['dur']:g} s, mission command {spec['cmd']}")
        print(f"    plots       : {', '.join(plots_for(spec))}")
        print("    running ...", flush=True)

        result = suite.run_case(to_mission_case(spec))
        plot_files = [] if result.error else make_plots(spec, result)
        status = record(spec, result, plot_files)

        for label, ok, detail in result.checks:
            print(f"      {'ok  ' if ok else 'FAIL'}  {label}: {detail}")
        if result.error:
            print(f"      ERROR {result.error}")
        print(f"    -> {status}; document updated. Refresh the browser.\n",
              flush=True)
        if status != "PASS":
            failures += 1

    print("=" * 64)
    print(f"{len(specs) - failures}/{len(specs)} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())

