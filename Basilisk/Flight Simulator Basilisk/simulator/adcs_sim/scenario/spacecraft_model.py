"""Select and place the spacecraft's 3D model in Vizard.

Replaces Vizard's default satellite shape with a CAD model, positioned so the
model's origin sits on the spacecraft body frame origin.

Rotation convention -- read this before changing angles
-------------------------------------------------------
:mod:`adcs_sim.config.visualization` states the orientation the way a person
reads it off Vizard's custom-model dialog: **degrees, about the body x, y and z
axes, in that order**.

Basilisk's ``vizSupport.createCustomModel`` wants something different: a 3-2-1
Euler set in **radians**, ordered ``[about z, about y, about x]``.  That was
confirmed empirically -- Basilisk builds the argument itself with
``RigidBodyKinematics.EP2Euler321``, and for a pure 90 deg rotation about x that
returns ``[0, 0, pi/2]``, i.e. the x angle lands in the *last* slot.

So a naive ``rotation=(90, 0, 0)`` would request **90 radians about z**: wrong
axis, wrong unit, and no error message.  :func:`rotation_321_rad` is the single
place that reordering and conversion happens.
"""

from pathlib import Path

from Basilisk.utilities import macros, vizSupport

from adcs_sim.config import visualization as viz_cfg


def rotation_321_rad(rotation_deg_xyz):
    """Convert ``(x, y, z)`` degrees to Basilisk's ``[z, y, x]`` radians.

    Parameters
    ----------
    rotation_deg_xyz : sequence of 3 floats
        Rotations about the body x, y and z axes [deg].

    Returns
    -------
    list of 3 floats
        3-2-1 Euler angles [rad] in the order ``createCustomModel`` expects.
    """
    if len(rotation_deg_xyz) != 3:
        raise ValueError("Rotation must be three angles (x, y, z) in degrees")
    about_x, about_y, about_z = (float(angle) * macros.D2R
                                 for angle in rotation_deg_xyz)
    return [about_z, about_y, about_x]


def model_path():
    """Return the configured model file, checked for existence.

    Raises
    ------
    FileNotFoundError
        A missing model is not reported by Vizard -- it silently draws its
        default satellite -- so fail here instead.
    """
    path = Path(viz_cfg.SPACECRAFT_MODEL_PATH)
    if not path.is_file():
        raise FileNotFoundError(
            f"Spacecraft 3D model not found: {path}\n"
            "Fix SPACECRAFT_MODEL_PATH in adcs_sim/config/visualization.py, or "
            "set USE_CUSTOM_SPACECRAFT_MODEL = False to use Vizard's default.")
    return path


def apply_custom_model(viz, spacecraft_names=None):
    """Attach the configured CAD model to the spacecraft in ``viz``.

    Call after :func:`~adcs_sim.scenario.visualization.enable_visualization`
    and before ``InitializeSimulation``: the model is part of the Vizard
    settings block, which is transmitted once at the start of the stream and is
    also embedded in a recorded file, so playback shows the model too.

    Parameters
    ----------
    viz : vizInterface.VizInterface
        The interface returned by ``enableUnityVisualization``.
    spacecraft_names : list of str, optional
        Bodies to replace.  Defaults to Vizard's first spacecraft, which is the
        only one this scenario defines.

    Returns
    -------
    pathlib.Path or None
        The model applied, or ``None`` when the feature is switched off.
    """
    if not viz_cfg.USE_CUSTOM_SPACECRAFT_MODEL:
        return None

    path = model_path()
    vizSupport.createCustomModel(
        viz,
        modelPath=str(path),
        simBodiesToModify=spacecraft_names,
        offset=list(viz_cfg.SPACECRAFT_MODEL_OFFSET_M),
        rotation=rotation_321_rad(viz_cfg.SPACECRAFT_MODEL_ROTATION_DEG),
        scale=list(viz_cfg.SPACECRAFT_MODEL_SCALE),
        customTexturePath=viz_cfg.SPACECRAFT_MODEL_TEXTURE_PATH,
        normalMapPath=viz_cfg.SPACECRAFT_MODEL_NORMAL_MAP_PATH,
        shader=viz_cfg.SPACECRAFT_MODEL_SHADER,
        color=(list(viz_cfg.SPACECRAFT_MODEL_COLOR)
               if viz_cfg.SPACECRAFT_MODEL_COLOR is not None else None),
    )
    return path


def apply_companion_model(viz, spacecraft_name):
    """Apply the companion's model, if one is configured.

    With ``COMPANION_MODEL_PATH = None`` this does nothing, which is what
    leaves the companion drawn with **Vizard's own built-in satellite mesh** --
    the easiest way to tell it apart from the main spacecraft's CAD model.
    ``"CUBE"``, ``"CYLINDER"`` and ``"SPHERE"`` are the other built-in options.

    Returns
    -------
    str or None
        The model applied, or ``None`` when Vizard's default is left in place.
    """
    from adcs_sim.config import companion as comp_cfg

    model = comp_cfg.COMPANION_MODEL_PATH
    if model is None:
        return None

    vizSupport.createCustomModel(
        viz,
        modelPath=str(model),
        simBodiesToModify=[spacecraft_name],
        offset=list(comp_cfg.COMPANION_MODEL_OFFSET_M),
        rotation=rotation_321_rad(comp_cfg.COMPANION_MODEL_ROTATION_DEG),
        scale=list(comp_cfg.COMPANION_MODEL_SCALE),
        color=(list(comp_cfg.COMPANION_MODEL_COLOR)
               if comp_cfg.COMPANION_MODEL_COLOR is not None else None),
    )
    return str(model)


def describe():
    """Return printable lines describing the model actually applied."""
    if not viz_cfg.USE_CUSTOM_SPACECRAFT_MODEL:
        return ["Spacecraft model: Vizard default"]

    offset = viz_cfg.SPACECRAFT_MODEL_OFFSET_M
    rotation_deg = viz_cfg.SPACECRAFT_MODEL_ROTATION_DEG
    rotation_rad = rotation_321_rad(rotation_deg)
    return [
        f"Spacecraft model: {Path(viz_cfg.SPACECRAFT_MODEL_PATH).name}",
        f"  offset  [m]  x={offset[0]:+g} y={offset[1]:+g} z={offset[2]:+g}",
        f"  rotation[deg] x={rotation_deg[0]:+g} y={rotation_deg[1]:+g} "
        f"z={rotation_deg[2]:+g}",
        f"  -> Basilisk 3-2-1 [rad] (z, y, x) = "
        f"[{rotation_rad[0]:+.6f}, {rotation_rad[1]:+.6f}, "
        f"{rotation_rad[2]:+.6f}]",
        f"  scale        {tuple(viz_cfg.SPACECRAFT_MODEL_SCALE)}",
    ]
