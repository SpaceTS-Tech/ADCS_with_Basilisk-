"""Vizard interface: file recording, live socket and real-time pacing."""

from Basilisk.simulation import simSynch
from Basilisk.utilities import vizSupport

from adcs_sim.config import visualization as viz_cfg
from adcs_sim.scenario import camera, spacecraft_model


def require_vizard_support():
    """Raise unless this Basilisk build includes Vizard support."""
    if not vizSupport.vizFound:
        raise RuntimeError(
            "Basilisk was built without Vizard support; the recording file "
            "cannot be created.")


def add_clock_sync(sim, task_name):
    """Register real-time pacing for a live run."""
    clock_sync = simSynch.ClockSynch()
    clock_sync.accelFactor = viz_cfg.LIVE_ACCELERATION_FACTOR
    sim.AddModelToTask(task_name, clock_sync)
    return clock_sync


def enable_visualization(bundle, live_stream, viz_file=None):
    """Attach the Vizard interface.

    Parameters
    ----------
    bundle : builder.SimulationBundle
        The assembled scenario.
    live_stream : bool
        True for an indefinite two-way socket stream, False to record a file.
    viz_file : pathlib.Path, optional
        Destination of the recording in file mode.  Ignored when streaming.
    """
    # scList[0] becomes Vizard's firstSpacecraftName -- the default target for
    # the camera and the custom model -- so the main spacecraft must come
    # first.  rwEffectorList must match scList in length, with None for a
    # spacecraft that has no wheels.
    viz = vizSupport.enableUnityVisualization(
        bundle.sim, bundle.task_name, bundle.visualized_spacecraft,
        saveFile=None if live_stream else str(viz_file),
        rwEffectorList=bundle.rw_effector_list,
        liveStream=live_stream,
    )
    if live_stream:
        viz.settings.keyboardLiveInput = viz_cfg.VIZ_KEYBOARD_LIVE_INPUT
        viz.reqComProtocol = viz_cfg.VIZ_SOCKET_PROTOCOL
        viz.reqComAddress = viz_cfg.VIZ_SOCKET_ADDRESS
        viz.reqPortNumber = viz_cfg.VIZ_SOCKET_PORT

    # rwEffectorList above wires the RW *data* into Vizard; it does not by
    # itself show the panel -- that is a separate GUI setting that otherwise
    # sits on Vizard's own (disabled) default.
    vizSupport.setActuatorGuiSetting(
        viz, spacecraftName=bundle.spacecraft.ModelTag,
        viewRWPanel=viz_cfg.SHOW_RW_PANEL,
        viewRWHUD=viz_cfg.SHOW_RW_HUD,
        showRWLabels=viz_cfg.SHOW_RW_LABELS,
    )

    # The custom model and the cameras live in the Vizard settings block, which
    # is sent once at the start of a live stream and embedded in a recorded
    # file -- so these apply to both modes, and to playback of the recording.
    # Named explicitly so the CAD model lands on the main spacecraft only: any
    # companion keeps Vizard's built-in shape and stays distinguishable.
    spacecraft_model.apply_custom_model(
        viz, spacecraft_names=[bundle.spacecraft.ModelTag])
    for line in spacecraft_model.describe():
        print(line)

    if bundle.companion is not None:
        spacecraft_model.apply_companion_model(
            viz, bundle.companion.spacecraft.ModelTag)

    cameras = camera.add_cameras(viz, spacecraft_name=bundle.spacecraft.ModelTag)
    # vizInterface holds a raw pointer to the instrument camera message.
    bundle.retained.append(cameras)
    for line in camera.describe():
        print(line)
    return viz
