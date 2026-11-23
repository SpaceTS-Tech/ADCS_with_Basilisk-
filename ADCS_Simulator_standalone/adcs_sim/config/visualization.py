"""Vizard visualization and live-stream settings.

Inventory item **C4** (the mission-command keyboard interface and the Vizard
launcher).
"""

from adcs_sim.config import paths

# Connect explicitly to Vizard's IPv4 loopback server on Windows.  This
# installed Basilisk build acts as the client; ``0.0.0.0`` is not a valid
# remote destination and ``localhost`` can resolve to the IPv6 address ``::1``.
VIZ_SOCKET_PROTOCOL = "tcp"
VIZ_SOCKET_ADDRESS = "127.0.0.1"
VIZ_SOCKET_PORT = "5556"

#: Keys Vizard forwards to the scenario during a live run.
#: ``0``-``9`` and ``a``/``b`` select mission commands 0-11; ``p`` toggles
#: pause; ``q`` quits.
VIZ_KEYBOARD_LIVE_INPUT = "0123456789abpq"

#: Real-time pacing factor applied by ``ClockSynch`` in live mode.  1.0 is
#: real time; larger values run faster than real time.
LIVE_ACCELERATION_FACTOR = 50.0

#: How long the live loop sleeps between polls while paused [s].
LIVE_PAUSE_POLL_SECONDS = 0.02

# --- live ACS-mode strip chart ---------------------------------------------
#: Open a matplotlib window during ``--mode live`` plotting the ACS state the
#: controller is really in (``B.actual_acs``) with wheel momentum underneath.
#: Together they show whether a desaturation is actually shedding momentum or
#: merely reporting that it is.  Disable per-run with ``--no-live-plot``.
#: Master switch for the window; each panel below has its own switch.
SHOW_LIVE_ACS_PLOT = True

#: ACS-mode panel (``B.actual_acs``).
SHOW_LIVE_MODE_PLOT = False

#: Reaction-wheel momentum panel, with the desat entry/exit thresholds.
SHOW_LIVE_MOMENTUM_PLOT = False

#: Add a panel to that window with solar-array generation, so the power
#: cost of a maneuver is visible next to the mode that caused it.  Ignored when
#: ``config/power.POWER_MODEL_ENABLED`` is False.
SHOW_LIVE_POWER_PLOT = False

#: Add a panel with the error quaternion of the flight software's 7th output,
#: ``q_BI_est`` (the star tracker's attitude estimate), relative to the true
#: ``q_BI`` fed to the model: ``C(q_err) = C(q_BI_est) C(q_BI)^T``.  A perfect
#: estimate reads ``[1, 0, 0, 0]``; the title gives the error angle.  Gaps mean
#: no fix.
SHOW_LIVE_Q_EST_PLOT = True

#: Wall-clock interval between canvas redraws [s].  The simulation loop runs at
#: 20 Hz; redrawing anywhere near that rate would dominate the run.
LIVE_PLOT_REFRESH_SECONDS = 0.5

#: Simulation-time span kept on screen [s].  ``None`` keeps the whole run,
#: which grows without bound on an indefinite stream.
LIVE_PLOT_WINDOW_SECONDS = 2400.0

#: Reaction-wheel visualisation in Vizard.
#: ``vizSupport.enableUnityVisualization``'s ``rwEffectorList`` wires the wheel
#: *data* through regardless of these flags -- without a separate
#: ``setActuatorGuiSetting`` call they stay on Vizard's own defaults, which are
#: off.  ``None`` leaves a default alone; ``True``/``False`` force it.
#:
#: ``SHOW_RW_PANEL`` is the docked momentum/torque readout panel.
#: ``SHOW_RW_HUD`` draws wheel indicators onto the spacecraft in the 3D view --
#: off, so it does not clutter the model or the camera panel.
#: ``SHOW_RW_LABELS`` annotates those HUD indicators, so it follows the HUD:
#: leaving it on with the HUD off would put stray wheel text back in the 3D
#: view, which is exactly what turning the HUD off is meant to avoid.
SHOW_RW_PANEL = True
SHOW_RW_HUD = False
SHOW_RW_LABELS = False


# ===========================================================================
# Automatic Vizard launch (live mode only)
# ===========================================================================
# Basilisk's vizInterface is the CLIENT: it connects out to the address above
# and waits.  Vizard is the SERVER and must be listening first.  See
# :mod:`adcs_sim.scenario.vizard_app` for what Vizard's command line can and
# cannot do -- in short, ``-directComm`` selects direct communication but there
# is no flag to set the socket address or to open the socket, so the final
# connect click in the Vizard UI cannot be automated away.

#: Start Vizard automatically when ``--mode live`` is selected.
#: Disable per-run with ``--no-vizard``.
AUTO_LAUNCH_VIZARD = True

#: Explicit path to ``Vizard.exe``, or ``None`` to auto-discover.
VIZARD_EXECUTABLE = None

#: Environment variable consulted before the search globs below.
VIZARD_EXECUTABLE_ENV_VAR = "VIZARD_EXECUTABLE"

#: Locations searched, in order, when no explicit path is given.  Entries may
#: contain ``*``/``?`` wildcards and ``~``/``%VAR%`` expansions.
VIZARD_SEARCH_GLOBS = (
    r"D:\Vizard_Windows64\Vizard\Vizard.exe",
    r"C:\Vizard_Windows64\Vizard\Vizard.exe",
    r"~\Vizard*\Vizard\Vizard.exe",
    r"C:\Vizard*\Vizard\Vizard.exe",
    r"D:\Vizard*\Vizard\Vizard.exe",
    r"C:\Program Files\Vizard*\Vizard.exe",
    r"/Applications/Vizard.app/Contents/MacOS/Vizard",
)

#: Arguments passed to Vizard on launch.  ``{socket}`` is substituted with
#: ``{protocol}://{address}:{port}`` built from the settings above.
#:
#: ``-directComm`` **requires** the socket address as the next argument: given
#: the flag alone, Vizard throws ``IndexOutOfRangeException`` in
#: ``StartUpScreenManager.Start()`` and never opens the socket.  With the
#: address supplied it binds within a couple of seconds and starts waiting for
#: Basilisk, with no UI interaction at all.
#:
#: Other flags Vizard accepts: ``-noDisplay``, ``-opNav``, ``-opNavMode``,
#: ``-loadFile``, ``-saveMsgFile``, ``-saveMetrics``.
VIZARD_LAUNCH_ARGS = ("-directComm", "{socket}")

#: Block until Vizard's socket is open *before* running
#: ``InitializeSimulation``.
#:
#: Leave this False.  Both sides wait for each other, but only one waits
#: forever: Basilisk blocks indefinitely at the end of ``InitializeSimulation``
#: ("Waiting for Vizard at ..."), whereas Vizard gives up with "Timed out
#: waiting for messages to load".  Waiting here *serialises* Vizard start-up
#: (5-20 s) ahead of Basilisk's initialisation, so Vizard's timer is already
#: running while Basilisk resets its models -- which was observed to lose the
#: race on a loaded machine.  With this False the two start up concurrently and
#: Basilisk is normally already waiting when Vizard's socket appears.
VIZARD_WAIT_FOR_SOCKET = False

#: How long to wait for Vizard's socket when ``VIZARD_WAIT_FOR_SOCKET`` is on,
#: and how long to poll for diagnostics afterwards otherwise [s].
VIZARD_STARTUP_TIMEOUT_SECONDS = 60.0

#: Interval between socket probes while waiting [s].
VIZARD_POLL_SECONDS = 0.5

#: Close an auto-launched Vizard when the run ends.  A Vizard that was already
#: running when the scenario started is never terminated.
VIZARD_TERMINATE_ON_EXIT = True

#: Grace period for Vizard to exit before it is killed [s].
VIZARD_SHUTDOWN_TIMEOUT_SECONDS = 10.0

# --- deadlock recovery ------------------------------------------------------
# Both sides wait for each other but only Basilisk waits forever, so every
# start-up race has exactly one bad outcome: Vizard opens the socket, starts
# its "waiting for messages to load" timer, gives up (red "Timed out waiting
# for connection"), and Basilisk then blocks at "Waiting for Vizard" against a
# port nobody is serving.  Nothing detects that, which is why the run has to be
# killed by hand.  The two settings below make it self-healing.

#: Reuse a Vizard that is already listening on the port instead of starting a
#: new one.
#:
#: **False by default, deliberately.**  A Vizard that is merely *bound* to the
#: port is not necessarily able to serve the stream -- the most common case is
#: precisely the opposite: a Vizard orphaned by a previous killed run, sitting
#: on its timed-out start screen.  Adopting that guarantees an immediate hang,
#: and the hang then repeats on every subsequent run, which is what makes this
#: fault look intermittent and self-inflicted.  With this False the stale
#: listener is terminated and a fresh Vizard is launched, so every run starts
#: from a known state.
VIZARD_ADOPT_EXISTING = False

#: Relaunch Vizard, while ``InitializeSimulation`` is still blocked, if it dies
#: or times out before the stream connects.  Basilisk's vizInterface is a
#: ZeroMQ client and reconnects on its own, so a freshly launched Vizard on the
#: same port is picked up without restarting the scenario.
VIZARD_WATCHDOG_ENABLED = True

#: Relaunch if the stream still has not connected this long after the last
#: launch [s].  Must exceed a cold Vizard start (5-20 s observed).
VIZARD_RELAUNCH_AFTER_SECONDS = 30.0

#: Give up after this many relaunches and leave Basilisk waiting, so a manual
#: connect is still possible.
VIZARD_RELAUNCH_ATTEMPTS = 3


# ===========================================================================
# Spacecraft 3D model
# ===========================================================================
# Replaces Vizard's default satellite shape with a CAD model.  The setting is
# part of the Vizard settings block, so it applies to both a live stream and a
# recorded file (and therefore to playback of that file).

#: Draw the spacecraft with the custom model below instead of Vizard's default.
USE_CUSTOM_SPACECRAFT_MODEL = True

#: Model file.  Vizard imports ``.obj`` and glTF (``.glb``/``.gltf``); this
#: build bundles glTFast + Draco, so ``.glb`` loads natively.
SPACECRAFT_MODEL_PATH = paths.SPACECRAFT_MODEL_FILE

#: Model origin offset in spacecraft body coordinates [m], as (x, y, z).
#: Use this to bring the CAD origin onto the body frame origin.
SPACECRAFT_MODEL_OFFSET_M = (0.75, -0.6, -0.75)

#: Model orientation as intrinsic rotations about the body (x, y, z) axes
#: **in degrees** -- the same order and units as Vizard's custom-model dialog.
#:
#: .. warning::
#:    Basilisk's ``createCustomModel`` does **not** take this form.  It wants a
#:    3-2-1 Euler set in **radians**, ordered ``[about z, about y, about x]``
#:    (confirmed against ``RigidBodyKinematics.EP2Euler321``, which is what
#:    Basilisk itself feeds that argument).  Passing ``(90, 0, 0)`` straight
#:    through would request 90 *radians* about *z*.
#:    :func:`adcs_sim.scenario.spacecraft_model.rotation_321_rad` performs the
#:    reordering and the degree-to-radian conversion in one place.
SPACECRAFT_MODEL_ROTATION_DEG = (0.0, 0.0, 90.0)

#: Per-axis scaling of the model in body coordinates.
SPACECRAFT_MODEL_SCALE = (1.0, 1.0, 1.0)

#: Optional texture and normal map; empty strings use whatever the model
#: carries (a ``.glb`` embeds its own materials).
SPACECRAFT_MODEL_TEXTURE_PATH = ""
SPACECRAFT_MODEL_NORMAL_MAP_PATH = ""

#: Shader: -1 Vizard default, 0 Unity Specular Standard, 1 Unity Standard.
SPACECRAFT_MODEL_SHADER = -1

#: Albedo tint as RGBA 0-255, or ``None`` to leave the model's own colours.
SPACECRAFT_MODEL_COLOR = None


# ===========================================================================
# Onboard camera
# ===========================================================================
# Vizard renders a live camera panel inside its own window showing the view
# from the spacecraft.  Two independent mechanisms exist:
#
#   Standard camera    -- a view panel only.  Cheap, needs nothing on the
#                         Basilisk side.  This is what shows "what the
#                         satellite sees".
#   Instrument camera  -- a configured sensor that also renders frames at a
#                         fixed resolution and rate, and can hand the image
#                         data back to Basilisk for image processing / opNav.
#
# Both are part of the Vizard settings block, so they apply to a live stream
# and to a recorded file.

# ---------------------------------------------------------------------------
# Shared payload optics -- SINGLE SOURCE OF TRUTH
# ---------------------------------------------------------------------------
# The view panel and the image-rendering camera are separate Vizard mechanisms
# with separate settings, so nothing stops them drifting apart and showing
# different scenes.  Both derive their boresight, mounting point, field of view
# and HUD setting from the three constants below, so "the panel" and "the
# optical image" cannot disagree.
#
# Change these, not the per-camera values underneath.

#: Payload boresight in **body** coordinates.
#:
#: Body +Z is the payload axis: the generated controller aligns +Z with nadir,
#: the Sun or the ground target depending on mission mode, so a +Z camera shows
#: whatever the spacecraft is currently pointing at.  Basilisk's own default is
#: +X, which would look sideways here.
#:
#: Verified against Vizard: an instrument camera with ``sigma_CB = 0`` renders
#: Earth filling the frame when body +Z is at nadir, so its boresight at zero
#: MRP is body +Z -- the same axis the panel uses.
#: :func:`adcs_sim.scenario.camera.sigma_cb_for_boresight` derives the matching
#: MRP for any other boresight.
PAYLOAD_BORESIGHT_B = (0.0, 0.0, 1.0)

#: Payload mounting point in body coordinates [m].  An all-zero vector places
#: the camera just outside the spacecraft along the boresight, which avoids
#: looking from inside the CAD model.
PAYLOAD_POSITION_B = (0.0, 0.0, 0.0)

#: Edge-to-edge field of view [deg], or ``None`` for the Vizard default.
PAYLOAD_FIELD_OF_VIEW_DEG = 6.0

#: Draw Vizard HUD overlays (orbit tracks, labels) into the rendered image:
#: ``None`` Vizard default, ``False`` off, ``True`` on.  Applied to both
#: cameras so the panel and the saved image match.  Set ``False`` if the image
#: is going to be consumed by image processing -- HUD lines are burned into the
#: pixels and will be picked up as features.
PAYLOAD_SHOW_HUD_IN_IMAGE = None

#: Show a live camera panel from the spacecraft in Vizard.
ENABLE_STANDARD_CAMERA = True

#: Panel title in the Vizard window.
CAMERA_DISPLAY_NAME = "Payload +Z"

# -- derived from the shared block above; do not edit individually ----------
CAMERA_POINTING_VECTOR_B = PAYLOAD_BORESIGHT_B
CAMERA_POSITION_B = PAYLOAD_POSITION_B
CAMERA_FIELD_OF_VIEW_DEG = PAYLOAD_FIELD_OF_VIEW_DEG

#: 1 = follow ``CAMERA_POINTING_VECTOR_B`` (default), 0 = body targeting, in
#: which case ``CAMERA_BODY_TARGET`` / ``CAMERA_SET_VIEW`` apply instead.
CAMERA_SET_MODE = 1

#: Body to track in body-targeting mode; "" selects the first celestial body.
CAMERA_BODY_TARGET = ""

#: Body-targeting view: 0 nadir, 1 orbit normal, 2 along track.
CAMERA_SET_VIEW = 0

CAMERA_SHOW_HUD_IN_IMAGE = PAYLOAD_SHOW_HUD_IN_IMAGE

#: .. note::
#:    **The standard camera has no resolution setting and cannot be given one.**
#:    ``vizInterface.StdCameraSettings`` exposes only ``spacecraftName``,
#:    ``displayName``, ``setMode``, ``setView``, ``bodyTarget``,
#:    ``fieldOfView``, ``pointingVector_B``, ``position_B`` and
#:    ``showHUDElementsInImage`` -- there is no pixel-count field, because it
#:    is a view panel that Vizard sizes to the window.
#:
#:    A fixed-resolution *optical image* comes from the instrument camera
#:    below, in visual render mode.  That is the one to use when the pixels
#:    have to be a defined size (feeding image data to flight software,
#:    image processing, opNav).


# --- Optical Cam: instrument camera 1 (used by tracking mode) --------------
#: The optical payload proper: renders a real image at a defined pixel size,
#: which is what an image consumer (flight software, image processing, opNav)
#: needs.  Unlike the standard camera above, this one has a resolution.
#:
#: Runs happily alongside the lidar; they must have distinct ``cameraID``
#: values, which :func:`adcs_sim.scenario.camera.add_cameras` enforces.
#:
#: .. note::
#:    Vizard names the written frames after the **simulation time**
#:    (``InstrumentCamera<ID>_<W>x<H>_<simtime_ns>.png``), not wall-clock time.
#:    Two runs covering the same sim times therefore overwrite each other's
#:    frames, so counting new files is not a reliable way to tell whether a
#:    camera rendered.  Move or clear the output folder between runs when
#:    comparing.
ENABLE_INSTRUMENT_CAMERA = True

#: Name used in this project's logs and summaries.  Vizard itself has no name
#: field for instrument cameras: its panel and frame files are labelled by ID
#: (``InstrumentCamera1_...png``).
INSTRUMENT_CAMERA_NAME = "Optical Cam"
INSTRUMENT_CAMERA_ID = 1
INSTRUMENT_CAMERA_RESOLUTION = (512, 512)      #: (width, height) [px]
INSTRUMENT_CAMERA_RENDER_RATE_S = 1.0          #: Frame interval [s]

# -- derived from the shared payload block; do not edit individually --------
#: Matches the panel's field of view so the two show the same scene.
INSTRUMENT_CAMERA_FIELD_OF_VIEW_DEG = 40.0
INSTRUMENT_CAMERA_POSITION_B = (0.0, 0.0, 0.6)

#: ``None`` derives the MRP from ``PAYLOAD_BORESIGHT_B`` so this camera looks
#: along the same axis as the panel.  Override with an explicit 3-element MRP
#: only to mount this camera off the payload boresight on purpose.
#:
#: .. warning::
#:    This is an **MRP**, not an axis.  A unit-magnitude MRP is a 180 degree
#:    rotation (the angle is ``4*atan(|sigma|)``), so the plausible-looking
#:    ``(0, 0, 1)`` is a half turn about the boresight: ``DCM_CB =
#:    diag(-1, -1, 1)``, which inverts both image axes.  That leaves the view
#:    looking correct -- the boresight is unchanged and the scene is merely
#:    upside down -- while silently flipping the sign of the centroid error the
#:    optical control law feeds back, turning mission command 9 into positive
#:    feedback.  It was set that way until 2026-08-03.
INSTRUMENT_CAMERA_SIGMA_CB = None

#: Star-field preference.  ``""`` uses the Vizard default star field;
#: ``"black"`` renders an empty sky.
#:
#: .. warning::
#:    ``"black"`` is a **simulation** workaround for a **flight software**
#:    weakness, and it is not flight representative -- a real optical payload
#:    sees stars.
#:
#:    ``<S18>/MATLAB Function2`` thresholds at ``pixel > 5`` and centroids the
#:    entire frame, so it cannot tell the target from the sky.  Measured over
#:    129 rendered frames: the default star field contributes ~1218 blobs and
#:    ~2600 lit pixels against the target's ~1000, dragging the reported
#:    centroid a mean of 186 px away from the satellite.  The loop then holds
#:    the *star field* centred and parks the target ~200 px off-axis.
#:    ``"black"`` drops that to 1-7 blobs and ~850 lit pixels, essentially the
#:    target alone.
#:
#:    Restore ``""`` once the model rejects small blobs.  Validated on the same
#:    129 frames: dropping connected components below 100 px recovers the true
#:    target centroid exactly (0.0 px error, largest star being 64 px against
#:    the smallest target at 815 px); a 7x7 morphological opening gets within
#:    2.1 px mean / 3.7 px worst case.
INSTRUMENT_CAMERA_SKYBOX = ""


# --- STR1: instrument camera 3 (star tracker) ------------------------------
#: A second optical instrument camera, looking out along body +Y at the star
#: field, as the image source for a star tracker.  It renders images only;
#: nothing here turns them into an attitude measurement.  Not to be confused
#: with ``config/sensors.STAR_TRACKER_ENABLED``, which is Basilisk's
#: ``starTracker`` sensor model: that reports attitude directly, with no image.
#:
#: Frames are written as ``InstrumentCamera3_<W>x<H>_<simtime_ns>.png`` beside
#: the payload camera's.  They reach the flight software's ``STR 1`` inport
#: only with ``config/star_tracker_camera.IMAGE_SOURCE = "vizard"``; by
#: default the simulator renders ``STR 1`` itself with the same optics (the
#: resolution, field of view and ``DCM_CB`` below), and this camera is just a
#: view in Vizard.  The ``Camera feed`` inport keeps reading only the payload
#: camera.
ENABLE_STAR_TRACKER_CAMERA = True

#: Name used in this project's logs and summaries; Vizard labels it by ID.
STAR_TRACKER_CAMERA_NAME = "STR1"
#: Must be unique across all instrument cameras.
STAR_TRACKER_CAMERA_ID = 3
STAR_TRACKER_CAMERA_RESOLUTION = (1024, 1024)    #: (width, height) [px]
STAR_TRACKER_CAMERA_RENDER_RATE_S = 0.05       #: Frame interval [s] (20 Hz)

#: Edge-to-edge along camera y [deg].  Independent of the payload optics.
STAR_TRACKER_CAMERA_FIELD_OF_VIEW_DEG = 20.0

#: Boresight in body coordinates: body +Y, perpendicular to the payload (+Z).
STAR_TRACKER_CAMERA_BORESIGHT_B = (0.0, 1.0, 0.0)

#: Mounting point in body coordinates [m], just outside the +X face.
#:
#: With the offset and rotation above, the custom model reaches 0.93 m from
#: the body origin along X (worked out from the extents in the model file), so
#: 1.0 m clears it.  Re-derive this if the model, its offset or its rotation
#: changes.  Vizard never draws a camera's own spacecraft in that camera's
#: image, so the model cannot block the view either way.
STAR_TRACKER_CAMERA_POSITION_B = (0.0, 1.0, 0.0)

#: **Star tracker mounting DCM -- use this same matrix as ``DCM_CB`` in the
#: flight software.**  ``v_C = DCM_CB v_B``, so row i is camera axis i in body
#: axes: row 1 = image +x (increasing column), row 2 = image +y (increasing
#: row), row 3 = boresight.  Must be a proper rotation (det = +1) whose third
#: row is ``STAR_TRACKER_CAMERA_BORESIGHT_B``; the scenario refuses to start
#: otherwise.  MATLAB form:  DCM_CB = [-1 0 0; 0 0 1; 0 1 0];
#: ``None`` derives the mounting from the boresight instead.
STAR_TRACKER_CAMERA_DCM_CB = (
    (-1.0, 0.0, 0.0),    # image +x  = body -X
    ( 0.0, 0.0, 1.0),    # image +y  = body +Z
    ( 0.0, 1.0, 0.0),    # boresight = body +Y
)

#: Overrides ``STAR_TRACKER_CAMERA_DCM_CB`` when not ``None``.
STAR_TRACKER_CAMERA_SIGMA_CB = None

#: A star tracker needs the star field.  ``"NASA_SVS"`` is named rather than
#: left as ``""`` so a change of Vizard default cannot swap it out; ``"ESO"``
#: is missing from this Vizard build.
STAR_TRACKER_CAMERA_SKYBOX = ""

#: Keep HUD overlays (orbit lines, labels) out of the image: burned into the
#: pixels, they would be picked up as false stars.
STAR_TRACKER_CAMERA_SHOW_HUD_IN_IMAGE = False


# --- LiDAR: instrument camera 2 (depth map) --------------------------------
# Vizard's instrument camera has a second render mode that outputs the **depth
# buffer** instead of a visual image, bounded by near/far clipping planes.  That
# is a range image: functionally a flash lidar / depth camera.
#
# What it is NOT: a scanning lidar.  There is no beam pattern, no pulse timing,
# no return intensity, no point-cloud output and no range noise.  It renders
# per-pixel geometric distance to whatever surface the pixel sees.  Treat it as
# a depth sensor, and add the sampling and noise model yourself if a real lidar
# product is needed.

#: Add a depth-rendering camera and its Vizard panel.
ENABLE_LIDAR_DEPTH_CAMERA = True

#: Name used in this project's logs and summaries; Vizard labels it by ID.
LIDAR_CAMERA_NAME = "LiDAR"
#: Must be unique across all instrument cameras.
LIDAR_CAMERA_ID = 2

LIDAR_RESOLUTION = (512, 512)          #: (width, height) [px]
LIDAR_RENDER_RATE_S = 0.5              #: Frame interval [s]
#: Kept independent of the optical FOV: a lidar normally has its own.
LIDAR_FIELD_OF_VIEW_DEG = 30.0         #: Edge-to-edge along camera y

# -- derived from the shared payload block; co-boresighted with the optics ---
LIDAR_POSITION_B = PAYLOAD_POSITION_B

#: ``None`` derives the MRP from ``PAYLOAD_BORESIGHT_B``, co-boresighting the
#: lidar with the optical camera.  Override to mount it elsewhere.
LIDAR_SIGMA_CB = None

#: Near and far clipping planes [m] -- the depth range the image spans.
#: Everything nearer than the first or further than the second is clipped.
#:
#: Sized for proximity operations: the companion satellite sits ~60 m ahead
#: along-track, so a 1-500 m window puts it well inside the range gate.  For
#: Earth-surface ranging this would need to be on the order of 3.6e6 m.
LIDAR_RANGE_M = (1.0, 500.0)
