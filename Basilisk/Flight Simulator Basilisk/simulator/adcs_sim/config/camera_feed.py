"""Routing of the rendered optical image into the generated flight software.

The Simulink model's 10th inport, ``'<Root>/Camera feed'``, is a 512x512
``uint8`` grayscale frame.  This module configures where those frames come
from and how stale a frame may be before it is discarded.

How the image gets here
-----------------------
Vizard's **instrument camera** renders frames and writes them to disk as PNGs.
Basilisk does *not* receive them back over the socket -- that needs Vizard's
optical-navigation mode, which cannot run at the same time as ``liveStream``
(``vizSupport`` rejects ``noDisplay`` with ``liveStream``).  So the scenario
reads the rendered files.

That means the path is **file-based and one frame behind**: Vizard writes,
the scenario picks the newest file up on its next poll.  It is adequate for
exercising the flight software's image input, and it is not a real-time
sensor interface.  Treat measured latency through this path as an artefact of
the plumbing, not of the flight software.
"""

from pathlib import Path

from adcs_sim.config import visualization as viz_cfg

#: Feed rendered frames into the controller's camera inport.
#:
#: With this False the inport is zero-filled once at reset and never written,
#: which is what the model sees if no camera is configured.
ENABLE_CAMERA_FEED = True

#: Directory Vizard writes instrument-camera frames to.  Vizard chooses this
#: itself -- it is the user's Documents folder, not something the scenario
#: sets -- so it is stated here rather than derived.
FRAME_DIRECTORY = None   # None => default resolved in camera_feed.frame_directory()

#: Which instrument camera to read.  Must match a configured camera ID;
#: Optical Cam is 1, LiDAR is 2 and STR1 is 3.
FRAME_CAMERA_ID = viz_cfg.INSTRUMENT_CAMERA_ID

#: How often to look for a new frame [s of simulation time].
#:
#: Scanning the frame directory is a filesystem walk, far too slow to do on
#: every 0.05 s control step, and pointless besides: Vizard only produces a
#: frame every ``INSTRUMENT_CAMERA_RENDER_RATE_S``.  Between polls the last
#: decoded frame is held.
POLL_INTERVAL_S = viz_cfg.INSTRUMENT_CAMERA_RENDER_RATE_S

#: Discard a frame older than this [s of simulation time], and feed a blank
#: image instead.  ``None`` holds the last frame indefinitely.
#:
#: A held-over frame is indistinguishable to the flight software from a fresh
#: one -- the inport carries no timestamp or validity flag -- so a generous
#: value silently feeds stale imagery.  Default is a few render intervals.
MAX_FRAME_AGE_S = 5.0 * viz_cfg.INSTRUMENT_CAMERA_RENDER_RATE_S

#: Blank the inport when no usable frame is available, rather than leaving the
#: previous one in place.  Prefer True: an explicitly empty frame is at least
#: honest about the absence of data.  Applies to both feeds.
CLEAR_WHEN_STALE = True


# --- Star tracker feed -------------------------------------------------------
#: Feed star tracker frames into the controller's ``STR 1`` inport.
#:
#: Where they come from is ``config/star_tracker_camera.IMAGE_SOURCE``: by
#: default the simulator renders them itself (no Vizard).  The settings below
#: apply only to ``IMAGE_SOURCE = "vizard"``, which reads Vizard's files.
ENABLE_STAR_TRACKER_FEED = True

#: Which instrument camera to read for ``STR 1`` (Vizard source only).
STAR_TRACKER_FRAME_CAMERA_ID = viz_cfg.STAR_TRACKER_CAMERA_ID

#: Poll and staleness limits for the star tracker feed, on the same reasoning
#: as the payload camera above.
STAR_TRACKER_POLL_INTERVAL_S = viz_cfg.STAR_TRACKER_CAMERA_RENDER_RATE_S
STAR_TRACKER_MAX_FRAME_AGE_S = 5.0 * viz_cfg.STAR_TRACKER_CAMERA_RENDER_RATE_S

#: CSV of the true attitude at every controller step -- ``t_s, q0, q1, q2,
#: q3`` with ``q = q_BN`` scalar first, exactly the ``q_BI`` handed to the
#: DLL.  Rendered frames are named by sim time, so this lets each star tracker
#: frame be paired with the attitude it was rendered at (see
#: ``tools/star_tracker_sky_check.py``).  ``None`` disables it.  Overwritten
#: at every reset.
STAR_TRACKER_TRUTH_LOG = (Path(__file__).resolve().parents[2]
                          / "output" / "star_tracker_truth.csv")
