"""Simulated star tracker camera -- the image source for the ``STR 1`` inport.

Basilisk has no image renderer, so ``STR 1`` used to be filled from frames
Vizard rendered and wrote to disk.  :mod:`adcs_sim.models.star_tracker_camera`
renders the star field inside the simulation instead, from the **true**
attitude, every frame, with no Vizard and no files.  This module configures
it.

Optics (resolution, field of view, mounting ``DCM_CB``) are shared with the
Vizard camera in :mod:`adcs_sim.config.visualization`, so both sources image
the same sky the same way.  The detector model and its values follow the
dataset generator the flight star tracker was developed against
(``Star Trcaker/Dataset_generation/generate_star_dataset.py``).
"""

import os
from pathlib import Path

from adcs_sim.config import visualization as viz_cfg

#: Where ``STR 1`` frames come from:
#:
#: ``"basilisk"`` -- rendered in the simulation by ``StarTrackerCamera``.
#: ``"vizard"``   -- Vizard's instrument camera, read back from disk (the old
#:                   path; needs Vizard running, lags by a frame).
#: The environment variable ``ADCS_STR_SOURCE`` overrides it for one run.
IMAGE_SOURCE = os.environ.get("ADCS_STR_SOURCE", "basilisk")

# --- optics (shared with the Vizard camera) ---------------------------------
RESOLUTION = viz_cfg.STAR_TRACKER_CAMERA_RESOLUTION          #: (width, height)
FIELD_OF_VIEW_DEG = viz_cfg.STAR_TRACKER_CAMERA_FIELD_OF_VIEW_DEG
#: Frame interval [s].  A frame is rendered on the first step at or after
#: each interval; between frames the last image is held.
FRAME_INTERVAL_S = viz_cfg.STAR_TRACKER_CAMERA_RENDER_RATE_S
CAMERA_ID = viz_cfg.STAR_TRACKER_CAMERA_ID

# --- catalogue ---------------------------------------------------------------
#: Built by ``tools/build_star_catalog.py`` from Hipparcos (V <= 7, J2000).
CATALOG_PATH = Path(__file__).resolve().parents[1] / "data" / "hipparcos_j2000.npz"
#: Faintest star rendered [V mag].
MAGNITUDE_LIMIT = 6.99

#: Rotate the rendered star field about the J2000 +Z axis (the celestial
#: pole) by this angle [deg].  **0 is the true sky.**
#:
#: The current flight software build (``ADCS_Software_v1``) post-multiplies
#: its star solution by a fixed 90 deg rotation (the quaternion ``j`` in the
#: '<Root>/MATLAB Function' block).  That compensates Vizard's skybox, which is
#: drawn rotated 90 deg about the pole relative to J2000.  Against the true
#: sky the same correction makes ``q_BI_est`` 90 deg wrong, so until ``j`` is
#: removed from the model the sky is rotated here to match: with 90 the flight
#: star tracker solves to ~1 arcsec; with 0 it is off by exactly 90 deg.
#: Set to 0 once the model no longer applies ``j``.  Every run prints a
#: warning while this is non-zero.
SKY_ROTATION_Z_DEG = 90.0

# --- detector (electrons unless stated) --------------------------------------
EXPOSURE_S = 0.1
FLUX_E_MAG0 = 2.0e6          #: electrons from a magnitude-0 star per exposure
PSF_SIGMA_PX = 1.1           #: Gaussian point-spread function, 1 sigma
PSF_RADIUS_SIGMA = 4.0       #: PSF drawn out to this many sigma
DARK_E_PER_S = 100.0
SKY_E_PER_S = 30.0
READ_NOISE_E = 30.0
GAIN_E_PER_ADU = 30.0
BIAS_ADU = 12.0

#: Smear stars along their motion during the exposure.  At 1 deg/s a star
#: moves ~5 px in 0.1 s, which matters for a star tracker during slews.
MOTION_BLUR = True
#: Upper bound on samples per smeared star (keeps fast tumbles cheap).
MOTION_BLUR_MAX_SAMPLES = 24

#: Background noise is drawn once into this many frames at reset and one is
#: picked per frame; only the star patches get fresh noise.  This is what
#: keeps a 1024 x 1024 frame cheap enough for every 0.05 s step.
NOISE_BANK_SIZE = 6
#: Seed for the noise -- fixed so runs are repeatable.
RANDOM_SEED = 0

# --- bright bodies ------------------------------------------------------------
#: Earth in the field of view: stars behind it are hidden and its sunlit side
#: is drawn at this level [ADU] (a 0.1 s exposure saturates on the day side).
EARTH_DAYSIDE_ADU = 255
EARTH_RADIUS_M = 6378137.0
#: Extra height counted as Earth (airglow, atmosphere) [m].
EARTH_ATMOSPHERE_M = 100e3

#: Sun within this angle of the boresight blinds the sensor: the whole frame
#: saturates.  Typical star tracker baffles exclude 25-45 deg.
SUN_EXCLUSION_DEG = 30.0

# --- diagnostics ----------------------------------------------------------------
#: Write every n-th rendered frame as PNG into ``FRAME_DUMP_DIR`` (0 = never).
FRAME_DUMP_EVERY = 0
FRAME_DUMP_DIR = Path(__file__).resolve().parents[2] / "output" / "star_tracker_frames"
