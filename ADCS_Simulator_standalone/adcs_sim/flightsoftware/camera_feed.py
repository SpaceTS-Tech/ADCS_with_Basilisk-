"""Read Vizard's rendered frames and hand them to the flight software.

Locates the newest instrument-camera PNG, decodes it to a 512x512 ``uint8``
grayscale image, and flattens it **column-major** ready for the generated
model's ``Camera feed`` inport.

Two details that are easy to get wrong
--------------------------------------
**Column-major.**  Simulink flattens a 2-D signal column-major; PIL and numpy
are row-major.  :func:`prepare_frame` applies ``order="F"``, so a frame that
looks right on screen also looks right to the model.  Skip that and the model
silently receives a transposed image.

**Vizard names files by simulation time**, not wall-clock
(``InstrumentCamera<ID>_<W>x<H>_<simtime_ns>.png``).  Runs covering the same
sim times therefore overwrite each other's frames, so "newest file" is decided
by modification time rather than by the name.
"""

import os
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from adcs_sim.config import camera_feed as feed_cfg
from adcs_sim.flightsoftware import structs

#: Path Vizard appends under the user's Documents folder.
VIZARD_OUTPUT_SUBPATH = Path("VizardData") / "Screenshots"

#: Filename pattern Vizard uses, per camera ID.
FRAME_PATTERN = "InstrumentCamera{camera_id}_*.png"


def documents_directory():
    """Return the user's real Documents folder.

    ``Path.home() / "Documents"`` is wrong whenever Documents has been
    redirected -- to OneDrive, a network share, another drive.  On Windows the
    authoritative location is the ``Personal`` shell-folder registry value, so
    consult that first and fall back to the naive path elsewhere.
    """
    if sys.platform == "win32":
        try:
            import winreg

            key = (r"Software\Microsoft\Windows\CurrentVersion\Explorer"
                   r"\Shell Folders")
            with winreg.OpenKey(winreg.HKEY_CURRENT_USER, key) as handle:
                personal, _ = winreg.QueryValueEx(handle, "Personal")
            if personal:
                return Path(os.path.expandvars(personal))
        except OSError:
            pass
    return Path.home() / "Documents"


def frame_directory():
    """Return the directory rendered frames are read from."""
    if feed_cfg.FRAME_DIRECTORY is not None:
        return Path(feed_cfg.FRAME_DIRECTORY)
    return documents_directory() / VIZARD_OUTPUT_SUBPATH


def newest_frame_path(directory=None, camera_id=None):
    """Return the most recently written frame, or ``None`` if there is none."""
    directory = Path(directory) if directory else frame_directory()
    if not directory.is_dir():
        return None

    camera_id = (feed_cfg.FRAME_CAMERA_ID if camera_id is None else camera_id)
    pattern = FRAME_PATTERN.format(camera_id=camera_id)
    try:
        frames = list(directory.glob(pattern))
    except OSError:
        return None
    if not frames:
        return None
    # By mtime, not by name: names encode sim time and repeat across runs.
    return max(frames, key=lambda path: path.stat().st_mtime)


#: ``(width, height)`` of the ``Camera feed`` inport.
CAMERA_FEED_SHAPE = (structs.CAMERA_FEED_WIDTH, structs.CAMERA_FEED_HEIGHT)

#: ``(width, height)`` of the ``STR 1`` star tracker inport.
STAR_TRACKER_FEED_SHAPE = (structs.STAR_TRACKER_FEED_WIDTH,
                           structs.STAR_TRACKER_FEED_HEIGHT)


def prepare_frame(image, shape=CAMERA_FEED_SHAPE):
    """Convert a PIL image to the flat column-major buffer the model wants.

    Parameters
    ----------
    image : PIL.Image.Image
    shape : (int, int)
        ``(width, height)`` the target inport expects.

    Returns
    -------
    numpy.ndarray
        ``uint8``, ``width * height`` elements, C-contiguous.

    Raises
    ------
    ValueError
        If the image is not the resolution the inport expects.  Resizing is
        deliberately not done silently -- a mismatch means the camera config
        and the model have diverged, and that should be visible.
    """
    grayscale = image.convert("L")
    width, height = grayscale.size
    if (width, height) != tuple(shape):
        raise ValueError(
            f"Rendered frame is {width}x{height} but the model's inport is "
            f"{shape[0]}x{shape[1]}. Set the camera's resolution in "
            "config/visualization.py to match, or regenerate the model for "
            "this resolution.")

    pixels = np.asarray(grayscale, dtype=np.uint8)      # (rows, cols)
    # Simulink flattens column-major; numpy is row-major.
    return np.ascontiguousarray(pixels.flatten(order="F"))


@dataclass
class FrameStatus:
    """What the reader did on a given step, for diagnostics."""

    updated: bool = False
    stale: bool = False
    path: object = None
    age_s: float = 0.0


class CameraFeedReader:
    """Supply the newest rendered frame, polling at a bounded rate.

    Decoding a PNG on every 0.05 s control step would dominate the run, and
    would be wasted work: Vizard only emits a frame every render interval.
    The reader therefore polls at most every
    :data:`~adcs_sim.config.camera_feed.POLL_INTERVAL_S` of simulation time
    and holds the last decoded frame in between.
    """

    def __init__(self, directory=None, camera_id=None, poll_interval_s=None,
                 max_frame_age_s=feed_cfg.MAX_FRAME_AGE_S,
                 shape=CAMERA_FEED_SHAPE):
        # Defaults read the payload camera for the 'Camera feed' inport.
        # max_frame_age_s=None is meaningful (hold frames indefinitely), so
        # its default is the payload limit itself rather than None.
        self._directory = Path(directory) if directory else frame_directory()
        self._camera_id = (feed_cfg.FRAME_CAMERA_ID if camera_id is None
                           else camera_id)
        self._poll_interval_s = (feed_cfg.POLL_INTERVAL_S
                                 if poll_interval_s is None else poll_interval_s)
        self._max_frame_age_s = max_frame_age_s
        self._shape = tuple(shape)
        self.reset()

    def reset(self):
        """Forget any cached frame.  Called from the controller's ``Reset``."""
        self._frame = None
        self._frame_path = None
        self._frame_mtime = None
        self._frame_time_s = None
        self._next_poll_s = 0.0
        self.frames_loaded = 0
        self.decode_failures = 0

    def update(self, time_s):
        """Poll if due and return the current :class:`FrameStatus`.

        Parameters
        ----------
        time_s : float
            Simulation time [s].
        """
        status = FrameStatus(path=self._frame_path)

        if time_s >= self._next_poll_s:
            self._next_poll_s = time_s + max(self._poll_interval_s, 1e-9)
            status.updated = self._load_newest(time_s)
            status.path = self._frame_path

        if self._frame_time_s is not None:
            status.age_s = time_s - self._frame_time_s
            limit = self._max_frame_age_s
            status.stale = limit is not None and status.age_s > limit
        else:
            status.stale = True
        return status

    def frame(self):
        """The flat column-major buffer, or ``None`` if nothing is loaded."""
        return self._frame

    def _load_newest(self, time_s):
        """Decode the newest frame if it is one we have not already read."""
        path = newest_frame_path(self._directory, self._camera_id)
        if path is None:
            return False

        try:
            mtime = path.stat().st_mtime
        except OSError:
            return False
        if path == self._frame_path and mtime == self._frame_mtime:
            return False       # same file, unchanged

        try:
            from PIL import Image
            with Image.open(path) as image:
                buffer = prepare_frame(image, self._shape)
        except (OSError, ValueError):
            # A half-written file is normal: Vizard may still be flushing it.
            # Keep the previous frame and try again on the next poll.
            self.decode_failures += 1
            return False

        self._frame = buffer
        self._frame_path = path
        self._frame_mtime = mtime
        self._frame_time_s = time_s
        self.frames_loaded += 1
        return True

    def describe(self):
        """Return printable lines summarising what the reader saw."""
        return [
            f"  camera feed: {self.frames_loaded} frame(s) loaded from "
            f"{self._directory}",
            f"    pattern {FRAME_PATTERN.format(camera_id=self._camera_id)}, "
            f"decode failures {self.decode_failures}",
        ]
