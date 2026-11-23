"""Onboard camera views in Vizard -- "what the satellite sees".

Two independent mechanisms, both configured in
:mod:`adcs_sim.config.visualization`:

**Standard camera** (:func:`add_standard_camera`)
    A live view panel rendered inside the Vizard window, following a boresight
    fixed in the spacecraft body frame.  Costs nothing on the Basilisk side and
    needs no message wiring.  This is the one that answers "show me what the
    spacecraft is looking at".

**Optical Cam** -- instrument camera 1 (:func:`add_instrument_camera`)
    A configured sensor that renders frames at a fixed resolution and rate and
    can return the image data to Basilisk for image processing or optical
    navigation.  Off by default -- a view panel does not need it.

**STR1** -- instrument camera 3 (:func:`add_star_tracker_camera`)
    A second visual instrument camera with a 20 deg field of view, rendering
    the star field for a star tracker.  Its mounting is the explicit
    ``STAR_TRACKER_CAMERA_DCM_CB``, which must equal the flight software's
    ``DCM_CB``.  It has its own sky and HUD settings: stars always on,
    overlays always off.

**LiDAR** -- instrument camera 2 (:func:`add_lidar_camera`)
    The same instrument camera in Vizard's depth render mode: it outputs the
    **depth buffer** between two clipping planes instead of a visual image, so
    each pixel carries geometric range.  This is a range image, i.e. a flash
    lidar / depth camera -- there is no beam scanning, pulse timing, return
    intensity, point cloud or range noise.

Payloads without engine support
-------------------------------
A **thermal / infrared imager has no built-in support** and is deliberately not
implemented here.  Basilisk's ``sensorThermal`` and ``motorThermal`` model the
*temperature of a hardware component* (absorptivity, emissivity, specific heat,
``temperatureOutMsg``) -- they are not imagers -- and Vizard's assembly contains
no thermal or infrared rendering path at all.  Producing IR imagery would mean
writing a radiometric renderer, not configuring an existing one.

Both live in the Vizard settings block, so they apply to a live stream *and* to
a recorded file, and therefore to playback of that file.

.. note::
   The panels appear **inside** the Vizard window (dockable/resizable), not as
   separate operating-system windows.  Vizard has no command-line option to
   detach them.

.. warning::
   ``createCameraConfigMsg`` makes ``vizInterface`` subscribe to the *pointer*
   of the message it creates -- Basilisk's own source comments note the message
   must be retained.  :class:`Cameras` holds that reference; drop it and the
   subscription dangles.
"""

from dataclasses import dataclass, field

import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import macros, vizSupport

from adcs_sim.config import visualization as viz_cfg


#: Vizard ``renderMode`` values for an instrument camera.
RENDER_VISUAL = 0
RENDER_DEPTH = 1


@dataclass
class Cameras:
    """Camera handles plus anything that must outlive the call."""

    standard: object = None
    instrument_payload: object = None
    star_tracker: object = None
    lidar_payload: object = None
    #: Retained: ``vizInterface`` holds raw pointers to these messages.
    retained: list = field(default_factory=list)


def sigma_cb_for_boresight(boresight_B=None):
    """Return the MRP ``sigma_CB`` that aims a camera along a body direction.

    Vizard's instrument camera looks along its own **+Z**.  Verified: with
    ``sigma_CB = 0`` and the spacecraft in LVLH attitude (body +Z at nadir),
    the rendered frame is filled by Earth.  So the camera's boresight at zero
    MRP is body +Z, matching ``createStandardCamera``'s pointing vector.

    Builds ``DCM_CB`` whose third row -- the camera +Z axis in body
    components -- is the requested boresight, with the remaining two axes
    completing a right-handed orthonormal set.  Roll about the boresight is
    unconstrained by "point at X" and is fixed here with a deterministic
    reference axis so results are repeatable.

    Parameters
    ----------
    boresight_B : sequence of 3 floats, optional
        Defaults to :data:`~adcs_sim.config.visualization.PAYLOAD_BORESIGHT_B`.

    Returns
    -------
    list of 3 floats
        ``sigma_CB``.  Exactly zeros when the boresight is body +Z.
    """
    if boresight_B is None:
        boresight_B = viz_cfg.PAYLOAD_BORESIGHT_B

    z_hat = np.asarray(boresight_B, dtype=float)
    norm = np.linalg.norm(z_hat)
    if norm == 0.0:
        raise ValueError("PAYLOAD_BORESIGHT_B must not be the zero vector")
    z_hat = z_hat / norm

    # Identity case, so the common configuration stays exactly zero.
    if np.allclose(z_hat, (0.0, 0.0, 1.0)):
        return [0.0, 0.0, 0.0]

    # Any reference not parallel to the boresight fixes the roll.
    reference = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(reference, z_hat)) > 0.9:
        reference = np.array([1.0, 0.0, 0.0])

    x_hat = np.cross(reference, z_hat)
    x_hat /= np.linalg.norm(x_hat)
    y_hat = np.cross(z_hat, x_hat)

    dcm_CB = np.vstack((x_hat, y_hat, z_hat))
    return list(rbk.C2MRP(dcm_CB))


def _field_of_view_rad(degrees):
    """Convert an edge-to-edge FOV in degrees to Basilisk's radians.

    ``None`` maps to ``-1``, which tells Vizard to use its own default.
    """
    if degrees is None:
        return -1.0
    return float(degrees) * macros.D2R


def add_standard_camera(viz, spacecraft_name=None):
    """Add the live view panel.  Returns the camera, or ``None`` if disabled."""
    if not viz_cfg.ENABLE_STANDARD_CAMERA:
        return None

    return vizSupport.createStandardCamera(
        viz,
        spacecraftName=spacecraft_name,
        setMode=viz_cfg.CAMERA_SET_MODE,
        setView=viz_cfg.CAMERA_SET_VIEW,
        bodyTarget=viz_cfg.CAMERA_BODY_TARGET,
        fieldOfView=_field_of_view_rad(viz_cfg.CAMERA_FIELD_OF_VIEW_DEG),
        pointingVector_B=list(viz_cfg.CAMERA_POINTING_VECTOR_B),
        position_B=list(viz_cfg.CAMERA_POSITION_B),
        displayName=viz_cfg.CAMERA_DISPLAY_NAME,
        showHUDElementsInImage=_hud_setting(viz_cfg.PAYLOAD_SHOW_HUD_IN_IMAGE),
    )


def _hud_setting(show_hud_in_image):
    """Map a tri-state HUD switch onto Vizard's 0 / -1 / 1 encoding."""
    if show_hud_in_image is None:
        return None
    return 1 if show_hud_in_image else -1


def _add_instrument_camera(viz, spacecraft_name, *, camera_id, resolution,
                           render_rate_s, field_of_view_deg, position_B,
                           sigma_CB, show_hud_in_image, skybox="",
                           render_mode=RENDER_VISUAL, clipping_planes=None):
    """Create one instrument camera.  Returns ``(payload, msg)``.

    ``camera_id`` must be unique across instrument cameras.  Depth mode
    (``render_mode=RENDER_DEPTH``) is the only mode that accepts
    ``clipping_planes``; Basilisk rejects them otherwise.
    """
    return vizSupport.createCameraConfigMsg(
        viz,
        cameraID=camera_id,
        parentName=spacecraft_name,
        fieldOfView=_field_of_view_rad(field_of_view_deg),
        resolution=list(resolution),
        renderRate=render_rate_s,
        cameraPos_B=list(position_B),
        sigma_CB=list(sigma_CB),
        skyBox=skybox,
        renderMode=render_mode,
        depthMapClippingPlanes=(list(clipping_planes)
                                if clipping_planes is not None else None),
        # HUD lines are burned into the pixels, which matters if the image is
        # going to be processed.
        showHUDElementsInImage=_hud_setting(show_hud_in_image),
    )


def add_instrument_camera(viz, spacecraft_name=None):
    """Add the visual image-rendering camera, or ``None`` if disabled."""
    if not viz_cfg.ENABLE_INSTRUMENT_CAMERA:
        return None

    return _add_instrument_camera(
        viz, spacecraft_name,
        camera_id=viz_cfg.INSTRUMENT_CAMERA_ID,
        resolution=viz_cfg.INSTRUMENT_CAMERA_RESOLUTION,
        render_rate_s=viz_cfg.INSTRUMENT_CAMERA_RENDER_RATE_S,
        field_of_view_deg=viz_cfg.INSTRUMENT_CAMERA_FIELD_OF_VIEW_DEG,
        position_B=viz_cfg.INSTRUMENT_CAMERA_POSITION_B,
        sigma_CB=(viz_cfg.INSTRUMENT_CAMERA_SIGMA_CB
                  if viz_cfg.INSTRUMENT_CAMERA_SIGMA_CB is not None
                  else sigma_cb_for_boresight()),
        # Same HUD setting as the panel, so both show the same scene.
        show_hud_in_image=viz_cfg.PAYLOAD_SHOW_HUD_IN_IMAGE,
        skybox=viz_cfg.INSTRUMENT_CAMERA_SKYBOX,
        render_mode=RENDER_VISUAL,
    )


def star_tracker_dcm_cb():
    """Return the star tracker mounting DCM ``[C <- B]`` actually rendered.

    ``STAR_TRACKER_CAMERA_SIGMA_CB`` wins if set, then
    ``STAR_TRACKER_CAMERA_DCM_CB``; with neither, the mounting is derived from
    ``STAR_TRACKER_CAMERA_BORESIGHT_B``.

    Raises
    ------
    ValueError
        If the DCM is not a proper rotation or its third row is not the
        boresight -- the same checks the flight software's star tracker block
        applies.
    """
    sigma_CB = getattr(viz_cfg, "STAR_TRACKER_CAMERA_SIGMA_CB", None)
    if sigma_CB is not None:
        return rbk.MRP2C(np.asarray(sigma_CB, dtype=float))
    boresight = np.asarray(viz_cfg.STAR_TRACKER_CAMERA_BORESIGHT_B, dtype=float)
    dcm_setting = getattr(viz_cfg, "STAR_TRACKER_CAMERA_DCM_CB", None)
    if dcm_setting is None:
        return rbk.MRP2C(np.asarray(sigma_cb_for_boresight(boresight)))

    dcm_CB = np.asarray(dcm_setting, dtype=float)
    if (dcm_CB.shape != (3, 3)
            or np.linalg.norm(dcm_CB @ dcm_CB.T - np.eye(3)) > 1e-9
            or np.linalg.det(dcm_CB) < 0):
        raise ValueError(
            "STAR_TRACKER_CAMERA_DCM_CB must be a proper 3x3 rotation "
            "(orthonormal, det = +1)")
    boresight = boresight / np.linalg.norm(boresight)
    if np.linalg.norm(dcm_CB[2] - boresight) > 1e-9:
        raise ValueError(
            "STAR_TRACKER_CAMERA_DCM_CB third row must be "
            f"STAR_TRACKER_CAMERA_BORESIGHT_B {tuple(boresight)}, "
            f"got {tuple(dcm_CB[2])}")
    return dcm_CB


def add_star_tracker_camera(viz, spacecraft_name=None):
    """Add the star tracker camera, or ``None`` if disabled."""
    if not viz_cfg.ENABLE_STAR_TRACKER_CAMERA:
        return None

    return _add_instrument_camera(
        viz, spacecraft_name,
        camera_id=viz_cfg.STAR_TRACKER_CAMERA_ID,
        resolution=viz_cfg.STAR_TRACKER_CAMERA_RESOLUTION,
        render_rate_s=viz_cfg.STAR_TRACKER_CAMERA_RENDER_RATE_S,
        field_of_view_deg=viz_cfg.STAR_TRACKER_CAMERA_FIELD_OF_VIEW_DEG,
        position_B=viz_cfg.STAR_TRACKER_CAMERA_POSITION_B,
        sigma_CB=list(rbk.C2MRP(star_tracker_dcm_cb())),
        show_hud_in_image=viz_cfg.STAR_TRACKER_CAMERA_SHOW_HUD_IN_IMAGE,
        skybox=viz_cfg.STAR_TRACKER_CAMERA_SKYBOX,
        render_mode=RENDER_VISUAL,
    )


def add_lidar_camera(viz, spacecraft_name=None):
    """Add the depth-rendering (lidar) camera, or ``None`` if disabled.

    Uses Vizard's depth render mode, which outputs per-pixel geometric range
    between the two clipping planes -- a range image, not a scanned point
    cloud.  See the config block for what this does and does not model.
    """
    if not viz_cfg.ENABLE_LIDAR_DEPTH_CAMERA:
        return None

    near, far = viz_cfg.LIDAR_RANGE_M
    if not 0 < near < far:
        raise ValueError(
            f"LIDAR_RANGE_M must be (near, far) with 0 < near < far, "
            f"got {viz_cfg.LIDAR_RANGE_M}")

    return _add_instrument_camera(
        viz, spacecraft_name,
        camera_id=viz_cfg.LIDAR_CAMERA_ID,
        resolution=viz_cfg.LIDAR_RESOLUTION,
        render_rate_s=viz_cfg.LIDAR_RENDER_RATE_S,
        field_of_view_deg=viz_cfg.LIDAR_FIELD_OF_VIEW_DEG,
        position_B=viz_cfg.LIDAR_POSITION_B,
        sigma_CB=(viz_cfg.LIDAR_SIGMA_CB
                  if viz_cfg.LIDAR_SIGMA_CB is not None
                  else sigma_cb_for_boresight()),
        show_hud_in_image=viz_cfg.PAYLOAD_SHOW_HUD_IN_IMAGE,
        render_mode=RENDER_DEPTH,
        clipping_planes=viz_cfg.LIDAR_RANGE_M,
    )


def add_cameras(viz, spacecraft_name=None):
    """Add every enabled camera.  Returns a :class:`Cameras` to keep alive."""
    cameras = Cameras()
    cameras.standard = add_standard_camera(viz, spacecraft_name)

    _check_unique_camera_ids()

    instrument = add_instrument_camera(viz, spacecraft_name)
    if instrument is not None:
        cameras.instrument_payload, message = instrument
        cameras.retained.append(message)

    star_tracker = add_star_tracker_camera(viz, spacecraft_name)
    if star_tracker is not None:
        cameras.star_tracker, message = star_tracker
        cameras.retained.append(message)

    lidar = add_lidar_camera(viz, spacecraft_name)
    if lidar is not None:
        cameras.lidar_payload, message = lidar
        cameras.retained.append(message)
    return cameras


def _check_unique_camera_ids():
    """Reject duplicate instrument-camera IDs before Vizard sees them."""
    used = []
    if viz_cfg.ENABLE_INSTRUMENT_CAMERA:
        used.append((viz_cfg.INSTRUMENT_CAMERA_NAME,
                     viz_cfg.INSTRUMENT_CAMERA_ID))
    if viz_cfg.ENABLE_STAR_TRACKER_CAMERA:
        used.append((viz_cfg.STAR_TRACKER_CAMERA_NAME,
                     viz_cfg.STAR_TRACKER_CAMERA_ID))
    if viz_cfg.ENABLE_LIDAR_DEPTH_CAMERA:
        used.append((viz_cfg.LIDAR_CAMERA_NAME, viz_cfg.LIDAR_CAMERA_ID))

    identifiers = [value for _, value in used]
    if len(identifiers) != len(set(identifiers)):
        names = ", ".join(f"{name} id={value}" for name, value in used)
        raise ValueError(
            f"Instrument cameras must have distinct cameraIDs, got {names}")


def describe():
    """Return printable lines describing the cameras in effect."""
    if not (viz_cfg.ENABLE_STANDARD_CAMERA
            or viz_cfg.ENABLE_INSTRUMENT_CAMERA
            or viz_cfg.ENABLE_STAR_TRACKER_CAMERA
            or viz_cfg.ENABLE_LIDAR_DEPTH_CAMERA):
        return ["Onboard camera: none"]

    lines = []
    if viz_cfg.ENABLE_STANDARD_CAMERA:
        boresight = viz_cfg.CAMERA_POINTING_VECTOR_B
        fov = viz_cfg.CAMERA_FIELD_OF_VIEW_DEG
        mode = ("pointing vector" if viz_cfg.CAMERA_SET_MODE == 1
                else f"body target '{viz_cfg.CAMERA_BODY_TARGET or 'first'}'")
        lines.append(
            f"Onboard camera: '{viz_cfg.CAMERA_DISPLAY_NAME}' panel in Vizard")
        lines.append(
            f"  boresight_B = ({boresight[0]:+g}, {boresight[1]:+g}, "
            f"{boresight[2]:+g}) | FOV = "
            f"{'Vizard default' if fov is None else format(fov, 'g') + ' deg'}"
            f" | mode = {mode}")
    if viz_cfg.ENABLE_INSTRUMENT_CAMERA:
        width, height = viz_cfg.INSTRUMENT_CAMERA_RESOLUTION
        lines.append(
            f"  {viz_cfg.INSTRUMENT_CAMERA_NAME} "
            f"(instrument camera id={viz_cfg.INSTRUMENT_CAMERA_ID}): "
            f"{width}x{height} px every "
            f"{viz_cfg.INSTRUMENT_CAMERA_RENDER_RATE_S:g} s")
    if viz_cfg.ENABLE_STAR_TRACKER_CAMERA:
        width, height = viz_cfg.STAR_TRACKER_CAMERA_RESOLUTION
        boresight = viz_cfg.STAR_TRACKER_CAMERA_BORESIGHT_B
        position = viz_cfg.STAR_TRACKER_CAMERA_POSITION_B
        lines.append(
            f"  {viz_cfg.STAR_TRACKER_CAMERA_NAME} "
            f"(instrument camera id={viz_cfg.STAR_TRACKER_CAMERA_ID}): "
            f"{width}x{height} px every "
            f"{viz_cfg.STAR_TRACKER_CAMERA_RENDER_RATE_S:g} s, "
            f"FOV {viz_cfg.STAR_TRACKER_CAMERA_FIELD_OF_VIEW_DEG:g} deg, "
            f"boresight_B = ({boresight[0]:+g}, {boresight[1]:+g}, "
            f"{boresight[2]:+g}) at ({position[0]:+g}, {position[1]:+g}, "
            f"{position[2]:+g}) m, sky '{viz_cfg.STAR_TRACKER_CAMERA_SKYBOX}'")
        dcm_CB = star_tracker_dcm_cb()
        rows = "; ".join(" ".join(f"{value:+g}" for value in row) + ""
                         for row in np.round(dcm_CB, 9) + 0.0)
        lines.append(f"    DCM_CB = [{rows}]  (must match the flight software)")
    if viz_cfg.ENABLE_LIDAR_DEPTH_CAMERA:
        width, height = viz_cfg.LIDAR_RESOLUTION
        near, far = viz_cfg.LIDAR_RANGE_M
        lines.append(
            f"  {viz_cfg.LIDAR_CAMERA_NAME} "
            f"(instrument camera id={viz_cfg.LIDAR_CAMERA_ID}, depth): "
            f"{width}x{height} px every "
            f"{viz_cfg.LIDAR_RENDER_RATE_S:g} s, "
            f"range gate {near:g}-{far:g} m, "
            f"FOV {viz_cfg.LIDAR_FIELD_OF_VIEW_DEG:g} deg")
    return lines
