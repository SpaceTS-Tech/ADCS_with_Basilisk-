"""Check the simulated star tracker camera against the flight star tracker.

Renders frames at random attitudes with ``StarTrackerCamera``, hands each one
to the generated DLL's ``STR 1`` inport, steps the model once, and compares
``q_BI_est`` with the attitude the frame was rendered at.  No simulation, no
Vizard -- a quick check after regenerating the model or changing the camera.

It also reports the constant part of the error on both sides,

* inertial side ``S = C_true^T C_est`` -- constant if the *sky* frames differ
  (e.g. the flight software still compensating Vizard's skybox),
* body side     ``R = C_est C_true^T`` -- constant if the *mounting* differs,

so a mismatch says where it lives.

Usage::

    python tools/star_tracker_render_check.py [--frames 40] [--sky-rotation 0]
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk

SIMULATOR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIMULATOR))

from adcs_sim.config import paths  # noqa: E402
from adcs_sim.config import star_tracker_camera as cam_cfg  # noqa: E402
from adcs_sim.flightsoftware.library import GeneratedADCSLibrary  # noqa: E402
from adcs_sim.models.star_tracker_camera import StarTrackerCamera  # noqa: E402
from adcs_sim.scenario.camera import star_tracker_dcm_cb  # noqa: E402


def angle_arcsec(dcm):
    cos = np.clip((np.trace(dcm) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cos)) * 3600.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--frames", type=int, default=40)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--sky-rotation", type=float, default=None,
                        help="override SKY_ROTATION_Z_DEG [deg]")
    args = parser.parse_args()

    if args.sky_rotation is not None:
        cam_cfg.SKY_ROTATION_Z_DEG = args.sky_rotation
    camera = StarTrackerCamera(star_tracker_dcm_cb())
    library = GeneratedADCSLibrary(paths.ADCS_LIBRARY)
    rng = np.random.default_rng(args.seed)
    # Far from Earth and with no Sun, so every frame is pure star field.
    position = np.array([7.0e6, 0.0, 0.0]) * 1e3

    errors, inertial, body, render_ms, no_fix = [], [], [], [], 0
    for _ in range(args.frames):
        sigma = rng.normal(size=3)
        sigma *= rng.uniform(0.05, 0.95) / np.linalg.norm(sigma)
        started = time.perf_counter()
        camera.render(sigma, np.zeros(3), position)
        render_ms.append(1e3 * (time.perf_counter() - started))

        library.initialize()                      # lost in space every frame
        library.set_star_tracker_feed(camera.feed_buffer())
        library.step()
        q = np.array(library.outputs.q_BI_est, dtype=float)
        if not np.all(np.isfinite(q)) or not q.any():
            no_fix += 1
            continue
        c_true = rbk.MRP2C(sigma)
        c_est = rbk.EP2C(q / np.linalg.norm(q))
        errors.append(angle_arcsec(c_est @ c_true.T))
        inertial.append(c_true.T @ c_est)
        body.append(c_est @ c_true.T)

    print(f"sky rotation {camera.sky_rotation_deg:+g} deg, "
          f"{len(camera.vmag)} stars to V {cam_cfg.MAGNITUDE_LIMIT}")
    print(f"render: {np.mean(render_ms):.2f} ms per frame "
          f"(max {np.max(render_ms):.2f})")
    print(f"solved {len(errors)}/{args.frames} frames, no fix on {no_fix}")
    if not errors:
        return 1
    errors = np.array(errors)
    print(f"attitude error: median {np.median(errors):.2f} arcsec, "
          f"95% {np.percentile(errors, 95):.2f}, max {errors.max():.2f}")
    for name, stack in (("inertial side", inertial), ("body side", body)):
        stack = np.array(stack)
        spread = float(stack.std(axis=0).max())
        mean = stack.mean(axis=0)
        print(f"{name:13s}: constant offset {angle_arcsec(mean):10.1f} arcsec "
              f"(spread {spread:.2e})")
    return 0 if np.median(errors) < 60.0 else 1


if __name__ == "__main__":
    sys.exit(main())
