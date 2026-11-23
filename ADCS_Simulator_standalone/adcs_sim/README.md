# `adcs_sim` — package map

Basilisk scenario for the Simulink-generated `ADCS_Software_v1` flight code,
split so that adding a model, a plot or a parameter touches one obvious file.

Entry point: [`../scenarioADCSSoftwareRW.py`](../scenarioADCSSoftwareRW.py)
Fidelity tracker: [`../ADCS_SIM_MODEL_INVENTORY.md`](../ADCS_SIM_MODEL_INVENTORY.md) — how good each model is
Data index: [`../ADCS_SIM_DATA_INDEX.md`](../ADCS_SIM_DATA_INDEX.md) — every constant and the file it lives in

---

## Layout

```
adcs_sim/
├── config/              every tunable number; imports no Basilisk sim modules
│   ├── paths.py            repo/DLL/output locations
│   ├── simulation.py       run length, controller rate, task priorities
│   ├── spacecraft.py       mass properties, integrator, orbit, ICs (E1)
│   ├── actuators.py        RW / MTQ / THR ratings + fidelity switches (A1-A3)
│   ├── sensors.py          nav error model + future sensor switches (S1-S6)
│   ├── environment.py      gravity, SPICE epoch, mag field, disturbances (E2-E9)
│   ├── power.py            solar-array geometry, eclipse, monitor (P1-P2)
│   ├── target.py           ground-target location                (S7)
│   ├── star_tracker_camera.py  STR 1 image source, optics, detector (S8)
│   └── visualization.py    Vizard socket & live-stream settings
│
├── geometry/            pure vector maths, no simulation state
│   ├── ground_target.py    target position, planet-fixed and ECI  (S7)
│   └── pointing.py         body-frame directions, solar elevation
│
├── flightsoftware/      the ctypes bridge to the generated DLL    (C1)
│   ├── structs.py          ExtU / ExtY / B mirrors + sensor/actuator interface (C2)
│   ├── library.py          typed DLL handle: load, tune, step
│   ├── diagnostics.py      mode histories & saturation aggregates
│   └── controller.py       the SysModel that marshals messages
│
├── models/              one module per simulated subsystem
│   ├── hub.py              rigid body + RKF45 integrator          (E1)
│   ├── environment.py      gravity, SPICE, magnetic field         (E2-E4)
│   ├── reaction_wheels.py  HR16 cluster, initial state, limits    (A1)
│   ├── magnetorquers.py    torque rods + voltage→dipole           (A2)
│   ├── thrusters.py        ideal-torque placeholder               (A3)
│   ├── navigation.py       SimpleNav + error model                (S1)
│   ├── solar_panels.py     array + eclipse + power monitor        (P1-P2)
│   ├── star_tracker_camera.py  renders STR 1 star images in-sim   (S8)
│   ├── companion.py        passive proximity satellite (visual)
│   └── initial_state.py    orbit and LVLH attitude initialisation
│
├── scenario/            assembly and execution
│   ├── builder.py          instantiate models in order, wire messages
│   ├── recorders.py        message recorders for file mode
│   ├── visualization.py    Vizard interface and clock sync
│   ├── spacecraft_model.py custom CAD model + placement           (C4)
│   ├── camera.py           onboard camera views                   (C4)
│   ├── vizard_app.py       find/launch Vizard, socket handshake   (C4)
│   ├── live.py             keyboard-driven live loop              (C4)
│   ├── runner.py           top-level run() for both modes
│   └── cli.py              argument parsing
│
├── data/
│   └── hipparcos_j2000.npz star catalogue for S8 (tools/build_star_catalog.py)
│
└── analysis/            post-run processing
    ├── backend.py          matplotlib backend selection
    ├── derived.py          recorders → analysis-ready arrays
    ├── plots.py            one function per figure, in a registry
    └── summary.py          printed telemetry summary
```

The bracketed codes (`A1`, `S7`, `E4`, `C2` …) are the inventory item IDs, so a
model's status, configuration and construction code are always one hop apart.

---

## Where to add what

| I want to… | Touch this |
|---|---|
| Change a rating, limit, gain or epoch | the matching `config/` module — **nothing else** |
| Turn on RW friction, nav noise, a disturbance | the switch in `config/`, then the `if` that reads it in `models/` |
| Add a new sensor or actuator | new `models/<thing>.py` with an `add_*` function + a `config/` block, then one call in `scenario/builder.py` and one subscription in `connect_messages` |
| Add a plot | one `plot_*` function in `analysis/plots.py` + one line in `PLOT_BUILDERS` |
| Add a derived quantity | a field in `analysis/derived.DerivedTelemetry` + its computation in `derive()` |
| Change what the DLL is fed | `flightsoftware/controller.py` `_write_inputs`, and the interface table in `flightsoftware/structs.py` (sensor values are passed through unconverted) |
| Add a DLL entry point | `flightsoftware/library.py` |
| Regenerate the Simulink code | `flightsoftware/structs.py` **must** be updated in the same commit |

---

## Two invariants worth knowing before you edit

**1. Model registration order is load-bearing.**
Within a Basilisk task, higher priority runs first and equal-priority models run
in the order they were added. `scenario/builder.build_simulation` reproduces the
original ordering:

```
SPICE (3) → reaction wheels (2) → hub (1)
  → magnetorquers → thrusters → magnetic field
  → navigation → controller
  → recorders / clock sync → Vizard interface
```

Reordering those calls changes which measurement each model sees within a step.

**2. In live mode, Basilisk must be the side that waits.**
Basilisk's `vizInterface` is the *client*: at the end of `InitializeSimulation` it
connects out to the socket and blocks forever (`Waiting for Vizard at ...`). Vizard is the
*server* and gives up (`Timed out waiting for messages to load`). So Vizard is launched and
then `InitializeSimulation` runs **immediately**, overlapping the two start-ups — do not
insert a blocking wait for Vizard's socket in between, or Vizard's timer runs down while
Basilisk resets its models and the handshake loses the race on a loaded machine.

**3. Basilisk holds raw C++ pointers, so Python lifetime matters.**
Every `add_*` function returns *all* objects that must outlive the call — the
integrator, the gravity-body factory, the standalone `MTBArrayConfigMsg`. They
are parked in `Hub`, `GravityEnvironment`, `Magnetorquers` and
`SimulationBundle.retained`. Dropping one of those references dangles a pointer
inside the C++ layer, which does not fail loudly.

---

## Running

```bash
python ../scenarioADCSSoftwareRW.py                      # 5000 s, writes output/
python ../scenarioADCSSoftwareRW.py --seconds 500 --show-plots
python ../scenarioADCSSoftwareRW.py --rw-momentum 0 0 0  # start unsaturated
python ../scenarioADCSSoftwareRW.py --mode live          # opens Vizard automatically
```

Artifacts (11 figures + `test2.bin`) land in `../output/` by default;
override with `--output-dir`.

### Live mode

`--mode live` launches Vizard itself, as
`Vizard.exe -directComm tcp://127.0.0.1:5556`, and streams to it with no clicking.
`-directComm` **requires** the address as its next argument — given the flag alone Vizard
throws `IndexOutOfRangeException` in `StartUpScreenManager.Start()` and never opens a socket.

The executable is resolved in this order, the first three strictly (a named path that does not
exist is an error, not a fall-through): `--vizard-exe`, `$VIZARD_EXECUTABLE`,
`config.visualization.VIZARD_EXECUTABLE`, then `VIZARD_SEARCH_GLOBS`.

```bash
python ../scenarioADCSSoftwareRW.py --mode live --no-vizard          # start Vizard yourself
python ../scenarioADCSSoftwareRW.py --mode live --vizard-exe D:\...\Vizard.exe
```

A Vizard already listening on the port is adopted rather than duplicated, and is never
terminated on exit; an auto-launched one is closed when the run ends
(`VIZARD_TERMINATE_ON_EXIT`). A force-kill of the Python process cannot clean it up.

### Spacecraft 3D model

The spacecraft is drawn with `3d models/m4v.glb` (Vizard loads `.glb` with its GLB runtime
importer). Configured in `config/visualization.py`, applied by
[`scenario/spacecraft_model.py`](scenario/spacecraft_model.py):

```python
SPACECRAFT_MODEL_OFFSET_M    = (0.75, -0.6, -0.75)   # metres, body x/y/z
SPACECRAFT_MODEL_ROTATION_DEG = (90.0, 0.0, 0.0)     # degrees about body x/y/z
```

**The rotation is expressed here as degrees about x, y, z — not the way Basilisk wants it.**
`vizSupport.createCustomModel` takes a 3-2-1 Euler set in **radians** ordered
`[about z, about y, about x]`, so `(90, 0, 0)` passed straight through would mean *90 radians
about z*. `rotation_321_rad()` does the reorder and unit conversion, and every run prints both
forms so the two can be checked against each other:

```
rotation[deg] x=+90 y=+0 z=+0
  -> Basilisk 3-2-1 [rad] (z, y, x) = [+0.000000, +0.000000, +1.570796]
```

The setting is part of Vizard's settings block, so it applies to live streams **and** to
recorded files, hence to playback. A missing model file raises rather than silently falling
back to Vizard's default satellite; set `USE_CUSTOM_SPACECRAFT_MODEL = False` for that on
purpose.

### Onboard camera — "what the satellite sees"

A live camera panel is rendered **inside** the Vizard window, following a body-fixed boresight.
Configured in `config/visualization.py`, applied by [`scenario/camera.py`](scenario/camera.py):

```python
ENABLE_STANDARD_CAMERA   = True
CAMERA_POINTING_VECTOR_B = (0.0, 0.0, 1.0)   # body +Z = the payload axis
CAMERA_FIELD_OF_VIEW_DEG = 40.0
```

Boresight is **body +Z** because that is the axis the controller points at nadir, the Sun or
the ground target depending on mission mode — so the panel shows whatever the spacecraft is
currently tracking. Basilisk's own default is +X, which would look sideways here.

The panel is dockable/resizable within Vizard; it is **not** a separate OS window, and Vizard
offers no flag to detach it. Like the CAD model it lives in the settings block, so it works in
live streams and in recorded-file playback.

### Companion satellite

A second, passive spacecraft flies in close proximity so the formation can be visualised.
Configured in `config/companion.py`, built by [`models/companion.py`](models/companion.py):

```python
ENABLE_COMPANION_SATELLITE     = True
COMPANION_ALONG_TRACK_OFFSET_M = 60.0    # + ahead in the orbit, − trailing
COMPANION_MODEL_PATH           = None    # None ⇒ Vizard's built-in satellite
```

It has mass properties and orbital dynamics but **no controller, actuators or sensors**, and
is not connected to the ADCS flight code.

**It does not perturb the main spacecraft.** Basilisk integrates each `Spacecraft`
independently in the central body's field — there is no inter-spacecraft gravity. Verified by
A/B run: attitude, body rate, position, wheel speeds and motor torques are all *bit-identical*
with the companion on and off. The only cost is integration time.

**Placement is an offset in true anomaly, not a position displacement.** Same semi-major axis
⇒ same period ⇒ no secular drift. Because the orbit is slightly eccentric (`e = 0.01`) the
separation breathes with orbital radius between **59.46 m and 60.66 m** — bounded and periodic,
not drift. Radial and cross-track offsets are deliberately not offered: displacing the position
would change the orbit, and a bounded relative orbit in those directions needs a proper
Clohessy-Wiltshire initialisation.

Leaving `COMPANION_MODEL_PATH = None` applies no custom model, so Vizard draws it with its own
built-in satellite mesh — distinguishable at a glance from the M4V `.glb`. `"CUBE"`,
`"CYLINDER"` and `"SPHERE"` are the other built-in options; a path loads a CAD model.

Two ordering constraints, both handled in `builder.SimulationBundle`: the main spacecraft must
be **first** in `scList` (Vizard takes `scList[0]` as its default target for the camera and
custom model), and `rwEffectorList` must match `scList` in length with `None` for the
wheel-less companion.

`ENABLE_INSTRUMENT_CAMERA` (default off) is the other mechanism: a configured sensor that
renders frames at a set resolution/rate and can hand image data back to Basilisk for opNav or
image processing. A view panel does not need it. If you enable it, verify the boresight
convention implied by `INSTRUMENT_CAMERA_SIGMA_CB` against an expected image — zero MRP means
the camera frame equals the body frame, which is not necessarily the boresight you want.

### Lidar (depth camera)

```python
ENABLE_LIDAR_DEPTH_CAMERA = True
LIDAR_RANGE_M             = (1.0, 500.0)   # near/far clipping planes
LIDAR_FIELD_OF_VIEW_DEG   = 30.0
```

Uses Vizard's depth render mode (`renderMode = 1`): each pixel carries geometric range between
the two clipping planes, so it is a **range image / flash lidar**, verified rendering the
companion satellite at ~60 m.

**What it is not:** a scanning lidar. No beam pattern, pulse timing, return intensity, point
cloud, or range noise. Add those yourself if a real lidar product is needed — this gives you
the geometry, not the radiometry.

The range gate is sized for proximity work (the companion sits ~60 m ahead). For Earth-surface
ranging it would need to be of order `3.6e6 m`.

> **Rendered frames are written to disk** by Vizard, at
> `Documents/VizardData/Screenshots/InstrumentCamera<ID>_<W>x<H>_<time>.png`, one per render
> interval. At `LIDAR_RENDER_RATE_S = 0.5` that is 7200 files/hour, and on this machine that
> directory is inside OneDrive, so they sync. Raise the interval or disable the camera for long
> runs.

### Thermal / IR imager — not available

Deliberately not implemented: **no built-in support exists.** Basilisk's `sensorThermal` and
`motorThermal` model the *temperature of a hardware component* (absorptivity, emissivity,
specific heat, `temperatureOutMsg`), not an imager, and Vizard's assembly contains no thermal
or infrared rendering path. Producing IR imagery would mean writing a radiometric renderer
rather than configuring an existing one.
