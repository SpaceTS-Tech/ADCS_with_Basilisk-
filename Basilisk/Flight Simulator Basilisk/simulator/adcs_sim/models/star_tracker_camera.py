"""Star tracker camera: renders the ``STR 1`` image inside the simulation.

Basilisk models sensors as numbers, not pictures; until now the star tracker
image came from Vizard, rendered to disk and read back a frame late.  This
module draws the star field itself, from the **true** spacecraft attitude, on
the simulation's own clock -- no Vizard, no files, repeatable run to run.

Camera model
------------
Frames follow the flight star tracker's conventions (``startracker.h``):
the camera looks along +Z_C, +X_C runs along image columns (u, right), +Y_C
along rows (v, down), and pixel (row i, column j) is centred on
``(u, v) = (j + 0.5, i + 0.5)``.  The mounting is ``DCM_CB`` from
:func:`adcs_sim.scenario.camera.star_tracker_dcm_cb`, the matrix the flight
software must also use.

Each frame:

1. **Stars** -- Hipparcos stars (V <= 7, J2000) within the field are
   projected through a pinhole of the configured field of view and drawn as a
   pixel-integrated Gaussian PSF, flux ``F0 * 10^(-0.4 V)``.  With motion blur
   on, each star is smeared along the path it traces during the exposure.
2. **Detector** -- dark current, sky background, shot noise and read noise,
   then gain, bias, rounding and 8-bit clipping.  Background noise comes from
   a bank drawn at reset; star patches get fresh shot noise every frame.
3. **Earth** -- stars behind the Earth (plus atmosphere) are hidden, and its
   sunlit side is drawn saturated.
4. **Sun** -- within the exclusion angle of the boresight the frame saturates.

The detector values match the dataset generator the flight star tracker was
developed against; see :mod:`adcs_sim.config.star_tracker_camera`.

Messages
--------
``scStateInMsg``  (SCStatesMsg)          true attitude, rate and position
``sunStateInMsg`` (SpicePlanetStateMsg)   Sun position, Earth-centred
``imageOutMsg``   (CameraImageMsg)        frame time, validity, camera ID, size

The pixels themselves are on :attr:`frame` (rows x columns) and, flattened
column-major for the Simulink inport, from :meth:`feed_buffer`.
``CameraImageMsg`` carries an image *pointer* that cannot be set from Python,
so the pixels travel alongside the message rather than inside it.
"""

import math
import time

import numpy as np
from Basilisk.architecture import messaging, sysModel
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import macros

from adcs_sim.config import star_tracker_camera as cam_cfg

SQRT2 = math.sqrt(2.0)

try:                                    # vectorised erf, if SciPy is present
    from scipy.special import erf as _erf
except ImportError:                     # pragma: no cover - fallback
    _erf = np.vectorize(math.erf)


def load_catalog(path=cam_cfg.CATALOG_PATH, magnitude_limit=None):
    """``(unit vectors [n, 3], V magnitudes [n])`` from the catalogue file."""
    data = np.load(path)
    vectors = np.asarray(data["vector"], dtype=float)
    vmag = np.asarray(data["vmag"], dtype=float)
    if magnitude_limit is not None:
        keep = vmag <= magnitude_limit
        vectors, vmag = vectors[keep], vmag[keep]
    return np.ascontiguousarray(vectors), vmag


class StarTrackerCamera(sysModel.SysModel):
    """Render star tracker frames from the simulated spacecraft state.

    Parameters
    ----------
    dcm_CB : array_like, 3x3
        Camera mounting, body to camera.
    config : module, optional
        Settings namespace; defaults to
        :mod:`adcs_sim.config.star_tracker_camera`.
    """

    def __init__(self, dcm_CB, config=cam_cfg):
        super().__init__()
        self.ModelTag = "StarTrackerCamera"
        self.cfg = config
        self.dcm_CB = np.asarray(dcm_CB, dtype=float)

        self.scStateInMsg = messaging.SCStatesMsgReader()
        self.sunStateInMsg = messaging.SpicePlanetStateMsgReader()
        self.imageOutMsg = messaging.CameraImageMsg()

        self.width, self.height = (int(n) for n in config.RESOLUTION)
        half_fov = math.radians(config.FIELD_OF_VIEW_DEG) / 2.0
        self.focal_px = (self.width / 2.0) / math.tan(half_fov)
        half_diagonal = math.atan(math.hypot(self.width, self.height) / 2.0
                                  / self.focal_px)
        blur_margin = math.radians(2.0)
        self._half_diagonal = half_diagonal
        self._search_cos = math.cos(half_diagonal + blur_margin)

        self.vectors, self.vmag = load_catalog(config.CATALOG_PATH,
                                               config.MAGNITUDE_LIMIT)
        self.sky_rotation_deg = float(getattr(config, "SKY_ROTATION_Z_DEG",
                                              0.0) or 0.0)
        if self.sky_rotation_deg:
            # Compatibility with a flight software build that compensates
            # Vizard's rotated skybox; see config.SKY_ROTATION_Z_DEG.
            angle = math.radians(self.sky_rotation_deg)
            c, s = math.cos(angle), math.sin(angle)
            rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
            self.vectors = np.ascontiguousarray(self.vectors @ rotation.T)
        self.flux_e = config.FLUX_E_MAG0 * np.power(10.0, -0.4 * self.vmag)

        self.frame = np.zeros((self.height, self.width), dtype=np.uint8)
        self._rays_C = None                 # pixel grid, built when Earth shows
        self._feed = None
        self._interval_ns = max(1, int(macros.sec2nano(config.FRAME_INTERVAL_S)))
        self.Reset(0)

    # ------------------------------------------------------------------ setup

    def Reset(self, current_sim_nanos):
        del current_sim_nanos
        cfg = self.cfg
        self._rng = np.random.default_rng(cfg.RANDOM_SEED)
        background_e = (cfg.DARK_E_PER_S + cfg.SKY_E_PER_S) * cfg.EXPOSURE_S
        bank = []
        for _ in range(max(1, int(cfg.NOISE_BANK_SIZE))):
            electrons = self._rng.poisson(background_e,
                                          (self.height, self.width))
            electrons = electrons + self._rng.normal(
                0.0, cfg.READ_NOISE_E, (self.height, self.width))
            bank.append((electrons / cfg.GAIN_E_PER_ADU
                         + cfg.BIAS_ADU).astype(np.float32))
        self._bank = bank
        self._bank_u8 = [np.clip(np.rint(b), 0, 255).astype(np.uint8)
                         for b in bank]
        self._next_frame_ns = 0
        self.frame[:] = 0
        self._feed = None
        self.frame_count = 0
        self.frame_time_ns = None
        self.stars_drawn = 0
        self.earth_in_view = False
        self.sun_blinded = False
        self.render_seconds = 0.0

    # ------------------------------------------------------------------ step

    def UpdateState(self, current_sim_nanos):
        if current_sim_nanos < self._next_frame_ns:
            return
        self._next_frame_ns = (current_sim_nanos // self._interval_ns + 1) \
            * self._interval_ns

        started = time.perf_counter()
        state = self.scStateInMsg()
        sun = self.sunStateInMsg() if self.sunStateInMsg.isLinked() else None
        self.render(np.asarray(state.sigma_BN, dtype=float),
                    np.asarray(state.omega_BN_B, dtype=float),
                    np.asarray(state.r_BN_N, dtype=float),
                    np.asarray(sun.PositionVector, dtype=float)
                    if sun is not None else None)
        self.render_seconds += time.perf_counter() - started

        self.frame_count += 1
        self.frame_time_ns = current_sim_nanos
        payload = messaging.CameraImageMsgPayload()
        payload.timeTag = current_sim_nanos
        payload.valid = 1
        payload.cameraID = int(self.cfg.CAMERA_ID)
        payload.imageBufferLength = self.width * self.height
        payload.imageType = 1                      # single-channel 8-bit
        self.imageOutMsg.write(payload, current_sim_nanos, self.moduleID)

        every = int(self.cfg.FRAME_DUMP_EVERY or 0)
        if every and self.frame_count % every == 0:
            self._dump(current_sim_nanos)

    def feed_buffer(self):
        """The current frame flattened column-major (Simulink inport order)."""
        if self._feed is None:
            self._feed = np.ascontiguousarray(self.frame.T).ravel()
        return self._feed

    # ------------------------------------------------------------------ render

    def render(self, sigma_BN, omega_BN_B, r_BN_N, r_sun_N=None):
        """Draw one frame into :attr:`frame` for the given true state."""
        cfg = self.cfg
        self._feed = None
        dcm_CN = self.dcm_CB @ rbk.MRP2C(sigma_BN)
        boresight_N = dcm_CN[2]

        index = int(self._rng.integers(len(self._bank)))
        np.copyto(self.frame, self._bank_u8[index])
        self.stars_drawn = 0
        self.earth_in_view = False
        self.sun_blinded = False

        if r_sun_N is not None:
            to_sun = r_sun_N - r_BN_N
            to_sun /= np.linalg.norm(to_sun)
            if boresight_N @ to_sun > math.cos(
                    math.radians(cfg.SUN_EXCLUSION_DEG)):
                self.sun_blinded = True
                self.frame.fill(255)
                return self.frame

        earth = self._earth_geometry(r_BN_N, boresight_N)
        self._draw_stars(dcm_CN, omega_BN_B, index, earth)
        if earth is not None:
            self._draw_earth(dcm_CN, r_BN_N, r_sun_N, earth)
        return self.frame

    def _earth_geometry(self, r_BN_N, boresight_N):
        """``(direction, cos of angular radius)`` if the Earth may show."""
        distance = np.linalg.norm(r_BN_N)
        radius = self.cfg.EARTH_RADIUS_M + self.cfg.EARTH_ATMOSPHERE_M
        if distance <= radius:
            return None
        to_earth = -r_BN_N / distance
        angular_radius = math.asin(radius / distance)
        off_axis = math.acos(float(np.clip(boresight_N @ to_earth, -1, 1)))
        if off_axis > self._half_diagonal + angular_radius:
            return None
        return to_earth, math.cos(angular_radius)

    def _draw_stars(self, dcm_CN, omega_BN_B, bank_index, earth):
        """Draw every star in view, vectorised over stars and blur samples."""
        cfg = self.cfg
        near = self.vectors @ dcm_CN[2] >= self._search_cos
        if earth is not None:
            near &= self.vectors @ earth[0] < earth[1]        # behind Earth
        if not np.any(near):
            return
        r_C = self.vectors[near] @ dcm_CN.T
        flux = self.flux_e[near]

        if cfg.MOTION_BLUR:
            omega_C = self.dcm_CB @ omega_BN_B
            # dr_C/dt = -omega x r_C, so over the exposure the star moves
            # from r + (T/2) omega x r to r - (T/2) omega x r.
            swing = 0.5 * cfg.EXPOSURE_S * np.cross(omega_C, r_C)
            start, end = r_C + swing, r_C - swing
        else:
            start = end = r_C
        front = (start[:, 2] > 1e-6) & (end[:, 2] > 1e-6)
        start, end, flux = start[front], end[front], flux[front]
        u0, v0 = self._project(start)
        u1, v1 = self._project(end)

        sigma = cfg.PSF_SIGMA_PX
        radius = int(math.ceil(cfg.PSF_RADIUS_SIGMA * sigma))
        visible = ((np.maximum(u0, u1) + radius >= 0)
                   & (np.minimum(u0, u1) - radius < self.width)
                   & (np.maximum(v0, v1) + radius >= 0)
                   & (np.minimum(v0, v1) - radius < self.height))
        if not visible.any():
            return
        u0, v0, u1, v1, flux = (a[visible] for a in (u0, v0, u1, v1, flux))
        self.stars_drawn = int(visible.sum())

        # One sample per PSF sigma of smear (at least one), all stars
        # together; closer spacing changes nothing a Gaussian PSF can show.
        length = np.hypot(u1 - u0, v1 - v0)
        samples = np.clip(np.ceil(length / sigma), 1,
                          cfg.MOTION_BLUR_MAX_SAMPLES).astype(int)
        star = np.repeat(np.arange(len(samples)), samples)
        first = np.cumsum(samples) - samples
        t = (np.arange(len(star)) - first[star] + 0.5) / samples[star]
        us = u0[star] + t * (u1[star] - u0[star])
        vs = v0[star] + t * (v1[star] - v0[star])
        weight = flux[star] / samples[star]

        # Pixel-integrated Gaussian, separable: exact and cheap.
        offsets = np.arange(-radius, radius + 1)
        xs = np.floor(us).astype(int)[:, None] + offsets
        ys = np.floor(vs).astype(int)[:, None] + offsets
        scale = 1.0 / (sigma * SQRT2)
        fx = 0.5 * (_erf((xs + 1.0 - us[:, None]) * scale)
                    - _erf((xs - us[:, None]) * scale))
        fy = 0.5 * (_erf((ys + 1.0 - vs[:, None]) * scale)
                    - _erf((ys - vs[:, None]) * scale))
        values = weight[:, None, None] * fy[:, :, None] * fx[:, None, :]
        inside = ((ys >= 0) & (ys < self.height))[:, :, None] & \
                 ((xs >= 0) & (xs < self.width))[:, None, :]
        flat = (ys[:, :, None] * self.width + xs[:, None, :])[inside]
        values = values[inside]
        if flat.size < 200_000:
            # Typical frame: sum overlapping patches over the touched pixels.
            pixels, slot = np.unique(flat, return_inverse=True)
            electrons = np.bincount(slot, weights=values)
        else:
            # Fast tumble, long smears: a full-frame sum is cheaper than a sort.
            full = np.bincount(flat, weights=values,
                               minlength=self.width * self.height)
            pixels = np.flatnonzero(full)
            electrons = full[pixels]

        # Fresh shot noise on the star light; the bank supplies the rest.
        shot = self._rng.poisson(electrons)
        adu = self._bank[bank_index].ravel()[pixels] + \
            shot / cfg.GAIN_E_PER_ADU
        self.frame.ravel()[pixels] = np.clip(np.rint(adu), 0, 255)

    def _project(self, r_C):
        scale = self.focal_px / r_C[:, 2]
        return (r_C[:, 0] * scale + self.width / 2.0,
                r_C[:, 1] * scale + self.height / 2.0)

    def _draw_earth(self, dcm_CN, r_BN_N, r_sun_N, earth):
        """Blank stars behind the Earth's disk and saturate its day side."""
        to_earth_N, cos_radius = earth
        e = (dcm_CN @ to_earth_N).astype(np.float32)
        cos_radius = np.float32(cos_radius)
        x, y, norm = self._pixel_grid()

        def in_disk(rows, cols):
            # cos(angle to Earth centre) >= cos(radius), without unit rays.
            return (x[rows, cols] * e[0] + y[rows, cols] * e[1] + e[2])                 >= cos_radius * norm[rows, cols]

        # Find the disk on a coarse grid, then work only inside its box.
        step = 8
        coarse = in_disk(slice(None, None, step), slice(None, None, step))
        if not coarse.any():
            return
        self.earth_in_view = True
        if r_sun_N is None:
            return                                 # no Sun: night side only
        rows_hit, cols_hit = np.nonzero(coarse)
        r0 = max(0, (rows_hit.min() - 1) * step)
        r1 = min(self.height, (rows_hit.max() + 2) * step)
        c0 = max(0, (cols_hit.min() - 1) * step)
        c1 = min(self.width, (cols_hit.max() + 2) * step)
        box = (slice(r0, r1), slice(c0, c1))
        # Whole frame inside the disk (looking straight at the Earth): the
        # per-pixel test is unnecessary.
        disk = True if coarse.all() else in_disk(*box)

        # Day/night on a coarse grid: the terminator needs no finer detail.
        coarse_box = (slice(r0, r1, 4), slice(c0, c1, 4))
        xc, yc, nc = x[coarse_box], y[coarse_box], norm[coarse_box]
        rays_C = np.stack((xc / nc, yc / nc, 1.0 / nc), axis=-1).reshape(-1, 3)
        d_N = rays_C.astype(float) @ dcm_CN
        along = -(d_N @ r_BN_N)
        miss2 = r_BN_N @ r_BN_N - along ** 2
        surface = self.cfg.EARTH_RADIUS_M ** 2
        depth = np.sqrt(np.clip(surface - miss2, 0.0, None))
        # Surface point where the ray hits; closest approach through the
        # atmosphere ring.
        reach = np.where(miss2 < surface, along - depth, along)
        points = r_BN_N + reach[:, None] * d_N
        lit = (points @ (r_sun_N / np.linalg.norm(r_sun_N)) > 0.0)
        lit = lit.reshape(xc.shape)
        if not lit.any():
            return
        if disk is True and lit.all():
            self.frame.fill(self.cfg.EARTH_DAYSIDE_ADU)
            return
        lit = np.repeat(np.repeat(lit, 4, axis=0), 4, axis=1)
        lit = lit[:r1 - r0, :c1 - c0]
        self.frame[box][disk & lit] = self.cfg.EARTH_DAYSIDE_ADU

    def _pixel_grid(self):
        """Pixel-centre tangents x, y and |(x, y, 1)| (cached, float32)."""
        if self._rays_C is None:
            u = (np.arange(self.width, dtype=np.float32) + 0.5
                 - self.width / 2.0) / np.float32(self.focal_px)
            v = (np.arange(self.height, dtype=np.float32) + 0.5
                 - self.height / 2.0) / np.float32(self.focal_px)
            x = np.broadcast_to(u[None, :], (self.height, self.width))
            y = np.broadcast_to(v[:, None], (self.height, self.width))
            norm = np.sqrt(x * x + y * y + 1.0).astype(np.float32)
            self._rays_C = (x, y, norm)
        return self._rays_C

    def _dump(self, current_sim_nanos):
        from PIL import Image
        folder = self.cfg.FRAME_DUMP_DIR
        folder.mkdir(parents=True, exist_ok=True)
        Image.fromarray(self.frame, mode="L").save(
            folder / f"str1_{current_sim_nanos:015d}.png")

    def describe(self):
        mean_ms = 1e3 * self.render_seconds / max(1, self.frame_count)
        lines = [f"  star tracker camera: {self.frame_count} frames rendered "
                 f"in the simulation ({mean_ms:.2f} ms each), "
                 f"{len(self.vmag)} catalogue stars to V "
                 f"{self.cfg.MAGNITUDE_LIMIT}"]
        lines += self.sky_warning()
        return lines

    def sky_warning(self):
        """Lines to print while the sky is deliberately not the true sky."""
        if not self.sky_rotation_deg:
            return []
        return [f"  WARNING: star field rotated {self.sky_rotation_deg:+g} deg "
                f"about J2000 Z to match the flight software's Vizard skybox "
                f"correction (config/star_tracker_camera.SKY_ROTATION_Z_DEG)"]


def add_star_tracker_camera(sim, task_name, dcm_CB):
    """Create and register the camera.  Wire its inputs in the builder."""
    camera = StarTrackerCamera(dcm_CB)
    sim.AddModelToTask(task_name, camera)
    return camera
