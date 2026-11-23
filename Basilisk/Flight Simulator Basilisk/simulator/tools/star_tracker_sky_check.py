"""Measure how Vizard's rendered star sky lines up with Basilisk's J2000 frame.

Pairs every star tracker frame from the last run with the true attitude it was
rendered at, solves each frame with the reference ``startracker.m`` (lost in
space, no body mounting -- it returns the *camera* attitude ``[CN]``), and
compares that with the camera attitude the simulator says it rendered:

    C_CN_true = STAR_TRACKER_CAMERA_DCM_CB @ C_BN_true

The DLL is not involved, so the flight software's ``DCM_CB`` cannot colour the
result.  For every solved frame it prints the boresight RA/Dec offset and two
candidate error rotations:

* inertial side  ``S = C_CN_true^T C_CN_meas``  -- constant if the *sky* is
  rotated (a skybox misalignment),
* camera side    ``R = C_CN_meas C_CN_true^T``  -- constant if the *camera*
  is mounted or imaged differently from what the simulator assumes.

Whichever stays fixed while the attitude changes is where the error lives.

Usage (after a live run with ``STAR_TRACKER_TRUTH_LOG`` enabled)::

    python tools/star_tracker_sky_check.py [--max-frames 40]
"""

import argparse
import csv
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk

SIMULATOR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIMULATOR))

from adcs_sim.config import camera_feed as feed_cfg  # noqa: E402
from adcs_sim.config import visualization as viz_cfg  # noqa: E402
from adcs_sim.flightsoftware import camera_feed  # noqa: E402
from adcs_sim.scenario.camera import star_tracker_dcm_cb  # noqa: E402

STARTRACKER_DIR = (SIMULATOR.parents[2] / "FlightSW_testing" / "startracker_m")
MATLAB = Path(r"C:\Program Files\MATLAB\R2026a\bin\matlab.exe")
FRAME_TIME = re.compile(r"_(\d+)\.png$")


def load_truth(path):
    """Return ``(times, quats, wall-clock start)`` from the truth CSV."""
    with open(path, encoding="ascii") as handle:
        started = float(handle.readline().split("=", 1)[1])
    data = np.loadtxt(path, delimiter=",", skiprows=2, ndmin=2)
    return data[:, 0], data[:, 1:5], started


def truth_at(times, quats, t):
    """Nearest logged step at or before ``t`` (the attitude the DLL saw)."""
    index = int(np.searchsorted(times, t + 1e-9)) - 1
    return quats[max(index, 0)], times[max(index, 0)]


def run_frames(started, max_frames):
    """Frames written since the run started (wall clock, seconds)."""
    width, height = viz_cfg.STAR_TRACKER_CAMERA_RESOLUTION
    pattern = (f"InstrumentCamera{feed_cfg.STAR_TRACKER_FRAME_CAMERA_ID}"
               f"_{width}x{height}_*.png")
    frames = [p for p in camera_feed.frame_directory().glob(pattern)
              if p.stat().st_mtime >= started - 1.0]
    frames.sort(key=lambda p: int(FRAME_TIME.search(p.name).group(1)))
    if len(frames) > max_frames:   # spread the sample over the whole run
        picks = np.linspace(0, len(frames) - 1, max_frames).round().astype(int)
        frames = [frames[i] for i in sorted(set(picks))]
    return frames


def solve_with_matlab(frames):
    """Run startracker.m on each frame; return {path: (q_CN, valid, nMatched)}."""
    work = Path(tempfile.mkdtemp(prefix="st_sky_"))
    listing = work / "frames.txt"
    listing.write_text("\n".join(str(p) for p in frames), encoding="utf-8")
    result = work / "solved.csv"
    script = work / "solve_frames.m"
    script.write_text(f"""
addpath('{STARTRACKER_DIR}');
files = strtrim(splitlines(fileread('{listing}')));
fid = fopen('{result}', 'w');
for k = 1:numel(files)
    if isempty(files{{k}}), continue; end
    img = imread(files{{k}});
    if ndims(img) == 3
        img = uint8(0.299*double(img(:,:,1)) + 0.587*double(img(:,:,2)) ...
                    + 0.114*double(img(:,:,3)));
    end
    [q, ~, valid, info] = startracker(img);
    fprintf(fid, '%s,%.12f,%.12f,%.12f,%.12f,%d,%d\\n', files{{k}}, q, ...
            valid, info.nMatched);
end
fclose(fid);
""", encoding="utf-8")
    subprocess.run([str(MATLAB), "-batch", f"run('{script}')"], check=True)
    solved = {}
    with open(result, newline="", encoding="utf-8") as handle:
        for row in csv.reader(handle):
            solved[Path(row[0])] = (np.array(row[1:5], float), row[5] == "1",
                                    int(row[6]))
    return solved


def radec(dcm_CN):
    z = dcm_CN[2]
    return (np.degrees(np.arctan2(z[1], z[0])) % 360,
            np.degrees(np.arcsin(np.clip(z[2], -1, 1))))


def angle_axis(R):
    angle = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    w, v = np.linalg.eig(R)
    axis = np.real(v[:, np.argmin(abs(w - 1))])
    return angle, axis / np.linalg.norm(axis)


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--truth", type=Path,
                        default=feed_cfg.STAR_TRACKER_TRUTH_LOG)
    parser.add_argument("--max-frames", type=int, default=40)
    args = parser.parse_args()

    times, quats, started = load_truth(args.truth)
    frames = run_frames(started, args.max_frames)
    if not frames:
        sys.exit("No star tracker frames from this run found in "
                 f"{camera_feed.frame_directory()}")
    print(f"truth log {args.truth}: {len(times)} steps, "
          f"t = {times[0]:.2f} .. {times[-1]:.2f} s")
    print(f"solving {len(frames)} frames with startracker.m ...")
    solved = solve_with_matlab(frames)
    dcm_CB = star_tracker_dcm_cb()

    print(f"\n{'t [s]':>7} {'ok':>3} {'match':>5} {'RA true':>8} {'RA meas':>8}"
          f" {'dRA':>7} {'dDec':>6}   inertial-side S (angle/axis N)"
          f"        camera-side R (angle/axis C)")
    inertial, camera = [], []
    for frame in frames:
        t = int(FRAME_TIME.search(frame.name).group(1)) * 1e-9
        if t < times[0] or t > times[-1]:
            continue
        q_meas, valid, matched = solved.get(frame, (None, False, 0))
        if not valid:
            print(f"{t:7.1f}  no  {matched:5d}   (not solved)")
            continue
        q_true, _ = truth_at(times, quats, t)
        C_true = dcm_CB @ rbk.EP2C(q_true)
        C_meas = rbk.EP2C(q_meas)
        ra_t, dec_t = radec(C_true)
        ra_m, dec_m = radec(C_meas)
        S = C_true.T @ C_meas
        R = C_meas @ C_true.T
        inertial.append(S)
        camera.append(R)
        s_ang, s_ax = angle_axis(S)
        r_ang, r_ax = angle_axis(R)
        print(f"{t:7.1f} yes {matched:5d} {ra_t:8.2f} {ra_m:8.2f}"
              f" {wrap180(ra_m - ra_t):+7.2f} {dec_m - dec_t:+6.2f}   "
              f"{s_ang:6.2f} {np.round(s_ax, 3)}   {r_ang:6.2f} {np.round(r_ax, 3)}")

    if len(inertial) < 2:
        sys.exit("\nFewer than two solved frames -- not enough to separate "
                 "a sky rotation from a camera rotation.")

    def spread(mats):
        mean = mats[0]
        return max(angle_axis(m @ mean.T)[0] for m in mats)

    print(f"\nsolved frames: {len(inertial)}")
    print(f"inertial-side S varies by up to {spread(inertial):.3f} deg "
          "between frames")
    print(f"camera-side   R varies by up to {spread(camera):.3f} deg "
          "between frames")
    att_span = max(angle_axis(rbk.EP2C(truth_at(times, quats, 0)[0])
                              @ rbk.EP2C(q).T)[0] for q in quats[::50])
    print(f"(true attitude itself changed by up to {att_span:.1f} deg "
          "over the run)")


if __name__ == "__main__":
    main()
