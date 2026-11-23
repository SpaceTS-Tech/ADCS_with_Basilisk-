# ADCS Simulation — Data Definition Index

**What this is:** every number, switch, path and data structure in the scenario, and the exact
file it is defined in.

**Scope:** `ADCS_software_test/simulator/` — entry point `scenarioADCSSoftwareRW.py`, package
`adcs_sim/`. Paths below are relative to that folder.
**Generated as of:** 2026-07-31 · Basilisk 2.11.0 · Python 3.12.10

**Companion documents**
- [`adcs_sim/README.md`](adcs_sim/README.md) — package map, "where to add what", the two
  load-bearing invariants.
- [`ADCS_SIM_MODEL_INVENTORY.md`](ADCS_SIM_MODEL_INVENTORY.md) — *fidelity* status of each
  model (what is real, what is idealised, what is missing). This document says **where data
  lives**; the inventory says **how good it is**.

> Line numbers are a convenience and will drift as files are edited. Constant *names* and file
> paths are stable — search by name if a line number looks wrong.

---

## 1. Fast lookup — "I want to change…"

| I want to change… | File | Constant |
|---|---|---|
| How long a run lasts | [`config/simulation.py`](adcs_sim/config/simulation.py) | `SIMULATION_SECONDS` (or `--seconds`) |
| Controller execution rate | [`config/simulation.py`](adcs_sim/config/simulation.py) | `CONTROLLER_STEP_SECONDS` |
| Which mission mode starts | [`config/simulation.py`](adcs_sim/config/simulation.py) | `MISSION_CMD` |
| Spacecraft mass / inertia | [`config/spacecraft.py`](adcs_sim/config/spacecraft.py) | `HUB_MASS_KG`, `HUB_INERTIA_SUPPLIED` |
| The orbit | [`config/spacecraft.py`](adcs_sim/config/spacecraft.py) | `ORBIT_*` |
| Initial tumble rate | [`config/spacecraft.py`](adcs_sim/config/spacecraft.py) | `INITIAL_TUMBLE_RATE_MAGNITUDE_RAD_S` |
| **Initial attitude / quaternion** | [`config/spacecraft.py`](adcs_sim/config/spacecraft.py) | `ALIGN_INITIAL_ATTITUDE_TO_LVLH`, `INITIAL_QUATERNION_BI` |
| Mission mode at startup | [`config/simulation.py`](adcs_sim/config/simulation.py) | `MISSION_CMD` |
| **Initial RW momentum** | [`config/actuators.py`](adcs_sim/config/actuators.py) | `DEFAULT_INITIAL_RW_MOMENTUM_NMS` (or `--rw-momentum`) |
| RW capacity / torque limit | [`config/actuators.py`](adcs_sim/config/actuators.py) | `RW_PHYSICAL_H_MAX`, `RW_MAX_MOTOR_TORQUE` |
| **RW friction coefficients** | [`config/actuators.py`](adcs_sim/config/actuators.py) | `RW_COULOMB_FRICTION`, `RW_VISCOUS_FRICTION` |
| Magnetorquer ratings | [`config/actuators.py`](adcs_sim/config/actuators.py) | `MTQ_MAX_VOLTAGE`, `MTQ_MAX_DIPOLE` |
| Thruster torque limit | [`config/actuators.py`](adcs_sim/config/actuators.py) | `THR_MAX_BODY_TORQUE` |
| Turn on nav sensor noise | [`config/sensors.py`](adcs_sim/config/sensors.py) | `NAV_ERROR_ENABLED`, `NAV_P_MATRIX` |
| Simulation epoch | [`config/environment.py`](adcs_sim/config/environment.py) | `SPICE_EPOCH` |
| Add disturbance torques | [`config/environment.py`](adcs_sim/config/environment.py) | `*_ENABLED` flags |
| Ground target location | [`config/target.py`](adcs_sim/config/target.py) | `TARGET_LATITUDE_DEG`, `TARGET_LONGITUDE_DEG` |
| **Solar-panel area / efficiency** | [`config/power.py`](adcs_sim/config/power.py) | `SOLAR_PANELS` (one entry per cell face) |
| Eclipse on/off, bus load | [`config/power.py`](adcs_sim/config/power.py) | `POWER_ECLIPSE_ENABLED`, `BUS_LOAD_W` |
| **3D model offset/rotation** | [`config/visualization.py`](adcs_sim/config/visualization.py) | `SPACECRAFT_MODEL_OFFSET_M`, `SPACECRAFT_MODEL_ROTATION_DEG` |
| Companion satellite | [`config/companion.py`](adcs_sim/config/companion.py) | `ENABLE_COMPANION_SATELLITE`, `COMPANION_ALONG_TRACK_OFFSET_M` |
| Onboard camera boresight/FOV | [`config/visualization.py`](adcs_sim/config/visualization.py) | `CAMERA_POINTING_VECTOR_B`, `CAMERA_FIELD_OF_VIEW_DEG` |
| Vizard socket / launch | [`config/visualization.py`](adcs_sim/config/visualization.py) | `VIZ_SOCKET_*`, `VIZARD_*` |
| Sensor/actuator interface to the DLL | [`flightsoftware/structs.py`](adcs_sim/flightsoftware/structs.py) | module docstring (interface table) |
| Where outputs are written | [`config/paths.py`](adcs_sim/config/paths.py) | `OUTPUT_DIR` (or `--output-dir`) |
| Add a plot | [`analysis/plots.py`](adcs_sim/analysis/plots.py) | `PLOT_BUILDERS` |

---

## 2. `config/paths.py` — filesystem locations

All derived from the file's own location, so the checkout can move without edits.

| Constant | Line | Value / derivation | Notes |
|---|---:|---|---|
| `GENERATED_CODE_DIR_NAME` | 11 | `"ADCS_Software_v1_grt_rtw"` | also the marker used to find `REPO_ROOT` |
| `PACKAGE_ROOT` | 26 | `<...>/ADCS_software_test/simulator/adcs_sim` | derived from `__file__` |
| `SCENARIO_DIR` | 28 | `<...>/ADCS_software_test/simulator` | entry-point directory |
| `REPO_ROOT` | 30 | `<...>/Basilisk` | nearest ancestor containing `GENERATED_CODE_DIR_NAME` |
| `ADCS_DIR` | 33 | `REPO_ROOT/ADCS_Software_v1_grt_rtw` | generated Simulink code |
| `ADCS_LIBRARY` | 34 | `ADCS_DIR/ADCS_Software_v1.dll` | the flight-code DLL |
| `OUTPUT_DIR` | 38 | `SCENARIO_DIR/output` | plots + Vizard recording |
| `VIZ_FILE_NAME` | 39 | `"test2.bin"` | recording filename |
| `MODEL_INVENTORY` | 42 | `SCENARIO_DIR/ADCS_SIM_MODEL_INVENTORY.md` | fidelity tracker |

`REPO_ROOT` is found by walking up from `SCENARIO_DIR` rather than by a fixed
number of `.parent` hops, so the scenario keeps working if the folders around
it are rearranged.

Functions: `resolve_output_dir()`, `viz_file()` — create the directory on demand.

---

## 3. `config/simulation.py` — timing and execution order

| Constant | Line | Value | Unit | Meaning |
|---|---:|---|---|---|
| `MISSION_CMD` | 5 | `4` | — | Mission mode at reset (0–11; 4 = nadir) |
| `SIMULATION_SECONDS` | 8 | `5000.0` | s | File-mode run length |
| `CONTROLLER_STEP_SECONDS` | 12 | `0.05` | s | DLL base sample time = task rate |
| `PLOT_SAMPLE_SECONDS` | 16 | `1.0` | s | Slow-state recording interval |
| `SETTLING_TIME_SECONDS` | 19 | `200.0` | s | Analysis window start |
| `SPICE_PRIORITY` | 33 | `3` | — | Task priority (higher runs first) |
| `REACTION_WHEEL_PRIORITY` | 34 | `2` | — | " |
| `HUB_PRIORITY` | 35 | `1` | — | " |
| `PROCESS_NAME` | 38 | `"simProcess"` | — | Basilisk process name |
| `TASK_NAME` | 39 | `"simTask"` | — | Basilisk task name |

> **Load-bearing:** the three priorities plus the registration order in
> [`scenario/builder.py`](adcs_sim/scenario/builder.py) determine which measurement each model
> sees within a step. Do not reorder casually.

---

## 4. Sensor/actuator interface to the DLL

Defined in the module docstring of
[`flightsoftware/structs.py`](adcs_sim/flightsoftware/structs.py) (`config/conventions.py`
was removed on 2026-09-15). The bridge passes sensor values through as measured and
applies commands as body-frame actuator commands; **no frame conversion and no mode
dependence**. `q_BI` = quaternion of the body relative to inertial (Basilisk `q_BN`,
scalar first); `omega` = `omega_BN_B` in body axes; `RW_tau` / `THR_tau` = torques on the
spacecraft in body axes (wheel motor torque = −`RW_tau`, fixed in `controller._read_outputs`).

| Constant | File | Value | Meaning |
|---|---|---|---|
| `OPTICAL_PIXEL_THRESHOLD` | `flightsoftware/controller.py` | `5.0` | Centroid threshold for the `ADCS_OPTICAL_DEBUG` trace only |

> **These are interface definitions, not tuned compensations.** The Simulink model must match
> them. A generated model built against the old interface (up to 2.99: conjugated quaternion,
> negated ECI rate, ECI torque outputs) does **not** point correctly with this simulator —
> verified 2026-09-15, a nadir run diverges with the wheels saturated. Inventory C2 /
> backlog #11 still tracks the unit test (known attitude in → known body torque out).

---

## 5. `config/spacecraft.py` — mass properties, integrator, orbit, initial state

### Rigid hub

| Constant | Line | Value | Unit |
|---|---:|---|---|
| `MODEL_TAG` | 11 | `"bsk-Sat"` | — |
| `HUB_MASS_KG` | 13 | `750.0` | kg |
| `HUB_COM_OFFSET_B` | 16 | `[[0],[0],[0]]` | m |
| `HUB_INERTIA_SUPPLIED` | 19 | `[202.6910, 0.6765, 0.2248, 0.6765, 115.7325, −16.5856, 0.2248, −16.5856, 161.4387]` | kg·m² |
| `HUB_INERTIA_SCALE` | 26 | `0.5` | — |
| `HUB_INERTIA_B` | 29 | **derived** = `SUPPLIED × SCALE` | kg·m² |

> The vehicle model supplies `L`; the scenario flies `L/2`. Edit `HUB_INERTIA_SUPPLIED` or
> `HUB_INERTIA_SCALE`, never `HUB_INERTIA_B`.

### Integrator (RKF45, adaptive)

| Constant | Line | Value |
|---|---:|---|
| `INTEGRATOR_REL_TOL` | 38 | `1.0e-8` |
| `INTEGRATOR_ABS_TOL` | 39 | `1.0e-10` |

### Orbit (classical elements, Earth-centred)

| Constant | Line | Value | Unit |
|---|---:|---|---|
| `ORBIT_SEMI_MAJOR_AXIS_M` | 44 | `10 000 000.0` | m (≈3600 km altitude) |
| `ORBIT_ECCENTRICITY` | 45 | `0.01` | — |
| `ORBIT_INCLINATION_DEG` | 46 | `33.3` | deg |
| `ORBIT_RAAN_DEG` | 47 | `48.2` | deg |
| `ORBIT_ARG_PERIAPSIS_DEG` | 48 | `347.8` | deg |
| `ORBIT_TRUE_ANOMALY_DEG` | 49 | `85.3` | deg |

### Initial attitude and rate

| Constant | Value | Meaning |
|---|---|---|
| `INITIAL_ATTITUDE_MODE` | `"RAM"` | `"LVLH"` (+Z nadir), `"RAM"` (+Z along velocity) or `"QUATERNION"` (use the value below) |
| `INITIAL_QUATERNION_BI` | `(0.069644714, −0.289358264, 0.823480650, −0.483012451)` | Explicit attitude: quaternion of the **body relative to inertial**, scalar first — Basilisk `q_BN`, exactly what the DLL receives. Only used when the mode above is `"QUATERNION"` |
| `INITIAL_QUATERNION_NORM_TOLERANCE` | `1.0e-6` | Rejects a non-unit quaternion rather than silently renormalising |
| `INITIAL_TUMBLE_RATE_MAGNITUDE_RAD_S` | `0.0` | **No tumble** — detumble path unexercised (backlog #12) |
| `INITIAL_TUMBLE_RATE_B` | **derived** = magnitude/√3 per axis | rad/s |

> Two representations of the same attitude exist: `sigma_BN` (MRP — the only form
> `hub.sigma_BNInit` accepts) and `q_BI` (what the DLL is fed = Basilisk `q_BN` unchanged, no
> conjugation). Conversion happens only in `initial_state.mrp_bn_from_q_bi()` /
> `q_bi_from_mrp_bn()`, which are exact inverses. The configured quaternion is 0.57° from the
> LVLH attitude of the current orbit (checked 2026-09-15); use `"LVLH"` for an exact nadir start.

---

## 6. `config/actuators.py` — actuator ratings and fidelity switches

### A1 · Reaction wheels (3 × Honeywell HR16, body-aligned)

| Constant | Line | Value | Unit | Meaning |
|---|---:|---|---|---|
| `RW_MODEL_TAG` | 14 | `"RW_cluster"` | — | Basilisk model tag |
| `RW_DEVICE_TYPE` | 15 | `"Honeywell_HR16"` | — | `rwFactory` profile name |
| `RW_SPIN_AXES` | 18 | `(1,0,0),(0,1,0),(0,0,1)` | — | Body-fixed spin axes |
| `RW_PHYSICAL_H_MAX` | 22 | `50.0` | N·m·s | Momentum capacity per wheel |
| `CONTROLLER_I_RW` | 27 | `0.07957747154594767` | kg·m² | Spin inertia `Js`; also sent to the DLL |
| `CONTROLLER_H_MAX` | 30 | `50.0` | N·m·s | DLL `h_max` parameter |
| `DESAT_ENTRY_FRACTION` | 36 | `0.95` | — | DESAT entry, × `CONTROLLER_H_MAX` |
| `DESAT_EXIT_FRACTION` | 37 | `0.25` | — | DESAT exit, × `CONTROLLER_H_MAX` |
| `RW_MAX_MOTOR_TORQUE` | 41 | `0.2` | N·m | Peak motor torque `u_max` |
| `DEFAULT_INITIAL_RW_MOMENTUM_NMS` | 45 | **derived** = `full(3, RW_PHYSICAL_H_MAX)` | N·m·s | Starts saturated ⇒ run begins in DESAT |
| `RW_USE_MAX_TORQUE` | 49 | `True` | — | Enforce `u_max` in the effector |
| `RW_IMBALANCE_MODEL` | 169 | `None` | — | `None` = `BalancedWheels` (no jitter) |

### A1 · Friction model

Measured law in this Basilisk build:
`|Ω| ≥ omegaLimitCycle → τ = −sign(Ω)·fCoulomb − cViscous·Ω`; below that it ramps linearly to 0.

| Constant | Line | Value | Unit | Applied? |
|---|---:|---|---|---|
| `RW_USE_FRICTION` | 89 | `True` | — | gate for `fCoulomb` **only** |
| `RW_COULOMB_FRICTION` | 97 | `5.0e-4` | N·m | ✅ yes — HR16 profile value |
| `RW_VISCOUS_FRICTION` | 150 | `7.0e-6` | N·m·s/rad | ✅ yes — estimate |
| `RW_LIMIT_CYCLE_SPEED` | 120 | `None` (⇒ `1.0e-4`) | rad/s | ✅ yes — anti-chatter ramp |
| `RW_STATIC_FRICTION` | 115 | `1.0e-3` | N·m | ⛔ **INERT** in Basilisk 2.11.0 |
| `RW_STRIBECK_BETA` | 164 | `0.5` | rad/s | ⛔ **INERT** in Basilisk 2.11.0 |
| `RW_USE_MIN_TORQUE` | 131 | `True` | — | enable command deadband |
| `RW_MIN_MOTOR_TORQUE` | 135 | `1.0e-5` | N·m | ✅ yes — deadband threshold |

Resulting drag at the 6000 RPM rating: **4.90e-3 N·m** (2.45 % of `u_max`), 3.1 W, coast-down
`Js/cViscous` = 3.2 h.

### A2 · Magnetic torque rods (3 × orthogonal)

| Constant | Line | Value | Unit |
|---|---:|---|---|
| `MTQ_MODEL_TAG` | 175 | `"MTQ_effector"` | — |
| `MTQ_COUNT` | 177 | `3` | — |
| `MTQ_ALIGNMENT_MATRIX_B` | 180 | identity 3×3, row-major | — |
| `MTQ_MAX_VOLTAGE` | 191 | `5.0` | V |
| `MTQ_MAX_DIPOLE` | 192 | `0.2` | A·m² |
| `MTQ_DIPOLE_PER_VOLT` | 196 | **derived** = `MAX_DIPOLE / MAX_VOLTAGE` = `0.04` | A·m²/V |

### A3 · Thrusters (desaturation)

| Constant | Line | Value | Unit |
|---|---:|---|---|
| `THR_MODEL_TAG` | 202 | `"Thruster_torque_effector"` | — |
| `THR_MAX_BODY_TORQUE` | 214 | `0.05` | N·m per body axis |

---

## 7. `config/sensors.py` — navigation and future sensors

| Constant | Line | Value | Meaning |
|---|---:|---|---|
| `NAV_MODEL_TAG` | 19 | `"SimpleNavigation"` | — |
| `NAV_ERROR_ENABLED` | 22 | `False` | **Truth state — no noise, bias or latency** |
| `NAV_P_MATRIX` | 27 | `zeros(18,18)` | Error σ; order `[r(3), v(3), σ_BN(3), ω(3), sunPnt(3), ΔV(3)]` |
| `NAV_WALK_BOUNDS` | 30 | `zeros(18)` | Random-walk bounds |
| `STAR_TRACKER_ENABLED` | 48 | `False` | Not instantiated (S2) |
| `IMU_ENABLED` | 49 | `False` | Not instantiated (S3) |
| `MAGNETOMETER_ENABLED` | 50 | `False` | Not instantiated (S4) |
| `COARSE_SUN_SENSOR_ENABLED` | 51 | `False` | Not instantiated (S5) |
| `RW_ENCODER_ENABLED` | 52 | `False` | Not instantiated (S6) |

---

## 8. `config/environment.py` — gravity, ephemeris, field, disturbances

| Constant | Line | Value | Meaning |
|---|---:|---|---|
| `GRAVITY_BODIES` | 14 | `("earth", "sun")` | Order fixes SPICE message indices |
| `CENTRAL_BODY` | 17 | `"earth"` | Integration centre |
| `USE_SPHERICAL_HARMONICS` | 21 | `False` | Point-mass only — **no J2** |
| `SPICE_EPOCH` | 28 | `"2026 JULY 13 06:30:00.0"` | UTC (= 12:00 IST) |
| `SPICE_ZERO_BASE` | 32 | `"Earth"` | All vectors Earth-centred J2000 |
| `MAGNETIC_FIELD_MODEL_TAG` | 38 | `"CenteredDipoleMagneticField"` | — |
| `MAGNETIC_FIELD_PLANET` | 41 | `"earth"` | Dipole coefficient set |
| `USE_WMM_MAGNETIC_FIELD` | 45 | `False` | Tilted centred dipole in use |
| `DRAG_ENABLED` | 65 | `False` | E5 — negligible at this altitude |
| `SOLAR_RADIATION_PRESSURE_ENABLED` | 66 | `False` | E6 — **not** negligible |
| `GRAVITY_GRADIENT_ENABLED` | 67 | `False` | E7 — **not** negligible |
| `RESIDUAL_DIPOLE_ENABLED` | 68 | `False` | E8 |
| `ECLIPSE_ENABLED` | 69 | `False` | E9 |

---

## 9. `config/target.py` — ground target

| Constant | Line | Value | Unit |
|---|---:|---|---|
| `TARGET_LATITUDE_DEG` | 7 | `28.522431782788708` | deg N |
| `TARGET_LONGITUDE_DEG` | 8 | `77.02428245359896` | deg E |
| `TARGET_RADIUS_M` | 17 | `6 378 137.0` | m — **spherical Earth, no WGS-84 flattening** |
| `USE_REFERENCE_ELLIPSOID` | 21 | `False` | Set True when the ellipsoid model lands |

---

## 10. `config/visualization.py` — Vizard socket, launcher, 3D model

### Socket and live loop

| Constant | Line | Value |
|---|---:|---|
| `VIZ_SOCKET_PROTOCOL` | 12 | `"tcp"` |
| `VIZ_SOCKET_ADDRESS` | 13 | `"127.0.0.1"` |
| `VIZ_SOCKET_PORT` | 14 | `"5556"` |
| `VIZ_KEYBOARD_LIVE_INPUT` | 19 | `"0123456789abpq"` |
| `LIVE_ACCELERATION_FACTOR` | 23 | `20.0` (× real time) |
| `LIVE_PAUSE_POLL_SECONDS` | 26 | `0.02` s |

> Vizard warns that `a` and `q` collide with its own hotkeys.

### Automatic Vizard launch

| Constant | Line | Value |
|---|---:|---|
| `AUTO_LAUNCH_VIZARD` | 41 | `True` |
| `VIZARD_EXECUTABLE` | 44 | `None` (auto-discover) |
| `VIZARD_EXECUTABLE_ENV_VAR` | 47 | `"VIZARD_EXECUTABLE"` |
| `VIZARD_SEARCH_GLOBS` | 51 | 7 paths, `D:\Vizard_Windows64\...` first |
| `VIZARD_LAUNCH_ARGS` | 72 | `("-directComm", "{socket}")` |
| `VIZARD_WAIT_FOR_SOCKET` | 85 | `False` — **leave False** (start-up race) |
| `VIZARD_STARTUP_TIMEOUT_SECONDS` | 89 | `60.0` s |
| `VIZARD_POLL_SECONDS` | 92 | `0.5` s |
| `VIZARD_TERMINATE_ON_EXIT` | 96 | `True` |
| `VIZARD_SHUTDOWN_TIMEOUT_SECONDS` | 99 | `10.0` s |

### Spacecraft 3D model

| Constant | Line | Value | Unit |
|---|---:|---|---|
| `USE_CUSTOM_SPACECRAFT_MODEL` | 110 | `True` | — |
| `SPACECRAFT_MODEL_PATH` | 114 | `REPO_ROOT/"3d models"/"m4v.glb"` | — |
| `SPACECRAFT_MODEL_OFFSET_M` | 118 | `(0.75, −0.6, −0.75)` | m, body (x, y, z) |
| `SPACECRAFT_MODEL_ROTATION_DEG` | 131 | `(0.0, 0.0, 0.0)` | deg about body (x, y, z) |
| `SPACECRAFT_MODEL_SCALE` | 134 | `(1.0, 1.0, 1.0)` | — |
| `SPACECRAFT_MODEL_TEXTURE_PATH` | 138 | `""` | — |
| `SPACECRAFT_MODEL_NORMAL_MAP_PATH` | 139 | `""` | — |
| `SPACECRAFT_MODEL_SHADER` | 142 | `-1` (Vizard default) | — |
| `SPACECRAFT_MODEL_COLOR` | 145 | `None` | RGBA 0–255 |

> Rotation is stored as **degrees about x, y, z**. Basilisk wants 3-2-1 Euler **radians**
> ordered `[z, y, x]`; `spacecraft_model.rotation_321_rad()` converts. Never hand these values
> to `createCustomModel` directly.

### Onboard camera

Applied by [`scenario/camera.py`](adcs_sim/scenario/camera.py).

| Constant | Value | Unit | Meaning |
|---|---|---|---|
| `ENABLE_STANDARD_CAMERA` | `True` | — | Live view panel inside the Vizard window |
| `CAMERA_DISPLAY_NAME` | `"Payload +Z"` | — | Panel title |
| `CAMERA_POINTING_VECTOR_B` | `(0, 0, 1)` | — | Boresight in body axes (+Z = payload axis) |
| `CAMERA_POSITION_B` | `(0, 0, 0)` | m | Zero ⇒ placed just outside the hull along the boresight |
| `CAMERA_FIELD_OF_VIEW_DEG` | `40.0` | deg | Edge-to-edge; `None` ⇒ Vizard default |
| `CAMERA_SET_MODE` | `1` | — | 1 = pointing vector, 0 = body targeting |
| `CAMERA_BODY_TARGET` | `""` | — | Body-targeting mode only |
| `CAMERA_SET_VIEW` | `0` | — | 0 nadir, 1 orbit normal, 2 along track |
| `CAMERA_SHOW_HUD_IN_IMAGE` | `None` | — | `None`/`False`/`True` |
| `ENABLE_INSTRUMENT_CAMERA` | `False` | — | Image-rendering sensor (opNav), off by default |
| `INSTRUMENT_CAMERA_ID` | `1` | — | Camera ID |
| `INSTRUMENT_CAMERA_RESOLUTION` | `(512, 512)` | px | Width × height |
| `INSTRUMENT_CAMERA_RENDER_RATE_S` | `1.0` | s | Frame interval |
| `INSTRUMENT_CAMERA_FIELD_OF_VIEW_DEG` | `40.0` | deg | Along camera y |
| `INSTRUMENT_CAMERA_POSITION_B` | `(0, 0, 0)` | m | Camera position |
| `INSTRUMENT_CAMERA_SIGMA_CB` | `(0, 0, 0)` | — | MRP camera←body; zero ⇒ frames coincide |
| `INSTRUMENT_CAMERA_SKYBOX` | `""` | — | Star-field preference |
| `ENABLE_LIDAR_DEPTH_CAMERA` | `True` | — | Depth-render camera (`renderMode=1`) + Vizard panel |
| `LIDAR_CAMERA_ID` | `2` | — | Must differ from `INSTRUMENT_CAMERA_ID` |
| `LIDAR_RESOLUTION` | `(512, 512)` | px | Width × height |
| `LIDAR_RENDER_RATE_S` | `0.5` | s | Frame interval — also the rate PNGs hit disk |
| `LIDAR_FIELD_OF_VIEW_DEG` | `30.0` | deg | Edge-to-edge along camera y |
| `LIDAR_POSITION_B` | `(0, 0, 0)` | m | Sensor position in body frame |
| `LIDAR_SIGMA_CB` | `(0, 0, 0)` | — | MRP sensor←body |
| `LIDAR_RANGE_M` | `(1.0, 500.0)` | m | Near/far clipping planes = the range gate |

> **Thermal / IR imager is not implemented — no built-in module exists.** `sensorThermal` and
> `motorThermal` are component-temperature models, not imagers, and Vizard has no infrared
> rendering path. See [`scenario/camera.py`](adcs_sim/scenario/camera.py) for the evidence.

> FOV is stored in **degrees**; `camera._field_of_view_rad()` converts to the radians Basilisk
> wants (`None` → `-1`, meaning "Vizard default"). The instrument camera's config **message
> must be retained** — `vizInterface` subscribes to its pointer — which `Cameras.retained`
> handles.

---

## 10b. `config/companion.py` — proximity satellite

Passive second spacecraft, present for visualisation. Built by
[`models/companion.py`](adcs_sim/models/companion.py). Has orbital dynamics but no controller,
actuators or sensors, and **does not perturb the main spacecraft** (verified bit-identical A/B).

| Constant | Value | Unit | Meaning |
|---|---|---|---|
| `ENABLE_COMPANION_SATELLITE` | `True` | — | Add it to the sim and the Vizard scene |
| `COMPANION_MODEL_TAG` | `"companion-Sat"` | — | Must differ from `spacecraft.MODEL_TAG`; Vizard rejects duplicates |
| `COMPANION_MASS_KG` | `50.0` | kg | Nominal |
| `COMPANION_INERTIA_B` | `diag(5, 5, 5)` | kg·m² | Row-major 3×3 |
| `COMPANION_COM_OFFSET_B` | `[[0],[0],[0]]` | m | — |
| `COMPANION_ALONG_TRACK_OFFSET_M` | `60.0` | m | + ahead, − trailing. Applied as a true-anomaly offset |
| `COMPANION_ALIGN_INITIAL_ATTITUDE_TO_LVLH` | `True` | — | Start LVLH-aligned, then tumble freely |
| `COMPANION_INITIAL_RATE_B` | `(0, 0, 0.02)` | rad/s | Slow spin; zeros for a static companion |
| `COMPANION_MODEL_PATH` | `None` | — | `None` ⇒ **Vizard built-in satellite**; also `"CUBE"`/`"CYLINDER"`/`"SPHERE"`, or a CAD path |
| `COMPANION_MODEL_OFFSET_M` | `(0, 0, 0)` | m | Only when a model path is set |
| `COMPANION_MODEL_ROTATION_DEG` | `(0, 0, 0)` | deg | " |
| `COMPANION_MODEL_SCALE` | `(1, 1, 1)` | — | " |
| `COMPANION_MODEL_COLOR` | `None` | RGBA 0–255 | Tint to tell the two apart |

> Separation is stable by construction — identical semi-major axis means identical period.
> With `e = 0.01` it breathes between **59.46 m and 60.66 m** with orbital radius; that is
> bounded periodic variation, not drift. Radial/cross-track offsets are not offered because
> they would need a Clohessy-Wiltshire initialisation to stay bounded.

---

## 10c. `config/power.py` — solar array, eclipse, power monitor

Telemetry only: nothing here applies a torque or reaches the flight software.

| Constant | Value | Unit | Meaning |
|---|---|---|---|
| `POWER_MODEL_ENABLED` | `True` | — | Master switch; `False` builds no panels, eclipse or monitor |
| `SOLAR_PANELS` | 2 × (2.0 m², 30 %) | — | One `SolarPanel(name, normal_B, area_m2, efficiency)` per cell face; **placeholder geometry** |
| `SOLAR_CONSTANT_1AU_W_M2` | `1361.0` | W/m² | Irradiance Basilisk assumes at 1 AU; mirrored for plots, the module holds its own copy |
| `POWER_ECLIPSE_ENABLED` | `True` | — | Earth shadow; off means full sun for the whole orbit |
| `ECLIPSE_OCCULTING_BODIES` | `("earth",)` | — | Must be listed in `environment.GRAVITY_BODIES` |
| `INITIAL_STORED_ENERGY_WS` | `0.0` | W·s | 0 makes the energy trace read as "generated since t = 0" |
| `BUS_LOAD_W` | `0.0` | W | 0 reports generation only; positive shows net charge/discharge |

Live display switch: `SHOW_LIVE_POWER_PLOT` in [`config/visualization.py`](adcs_sim/config/visualization.py).

---

## 11. Data defined outside `config/`

| Data | File | Line | What it is |
|---|---|---:|---|
| `MISSION_CMD_RANGE` | [`flightsoftware/controller.py`](adcs_sim/flightsoftware/controller.py) | 29 | `(0, 11)` valid mission commands |
| `INPUT_SYMBOL` / `OUTPUT_SYMBOL` / `BLOCK_SIGNAL_SYMBOL` | [`flightsoftware/structs.py`](adcs_sim/flightsoftware/structs.py) | 21–23 | DLL exported symbol names |
| `ADCSInputs` / `ADCSOutputs` / `ADCSBlockSignals` | [`flightsoftware/structs.py`](adcs_sim/flightsoftware/structs.py) | — | **ctypes struct layouts — must match the generated C headers exactly** |
| `RATED_SPEED_RAD_S` | [`models/reaction_wheels.py`](adcs_sim/models/reaction_wheels.py) | 59 | derived `H_MAX / I_RW` = 628.32 rad/s |
| `COMPONENT_LABELS` | [`analysis/derived.py`](adcs_sim/analysis/derived.py) | 19 | `("x","y","z")` |
| `TorqueSeries` | [`analysis/derived.py`](adcs_sim/analysis/derived.py) | — | namedtuple `(values, title, ylabel)` |
| `DerivedTelemetry` | [`analysis/derived.py`](adcs_sim/analysis/derived.py) | — | dataclass of all analysis arrays |
| `FILE_BACKEND` | [`analysis/backend.py`](adcs_sim/analysis/backend.py) | 11 | `"Agg"` |
| `FIGURE_DPI` | [`analysis/plots.py`](adcs_sim/analysis/plots.py) | 19 | `200` |
| `PLOT_BUILDERS` | [`analysis/plots.py`](adcs_sim/analysis/plots.py) | 238 | figure-name → builder registry (11 figures) |
| Mission key map | [`scenario/live.py`](adcs_sim/scenario/live.py) | — | `0–9`, `a`→10, `b`→11, `p` pause, `q` quit |
| Recorder sample rates | [`scenario/recorders.py`](adcs_sim/scenario/recorders.py) | — | slow states at `PLOT_SAMPLE_SECONDS`, actuators at `CONTROLLER_STEP_SECONDS` |
| **Message wiring** | [`scenario/builder.py`](adcs_sim/scenario/builder.py) | — | `connect_messages()` — the whole signal graph |
| **Model registration order** | [`scenario/builder.py`](adcs_sim/scenario/builder.py) | — | `build_simulation()` — load-bearing |

### DLL interface data (`flightsoftware/structs.py`)

**Inputs → DLL** (`ExtU_ADCS_Software_v1_T`), all ECI unless noted:
`q_BI[4]` (scalar-first), `omega[3]`, `B[3]` (**body** axes, T), `RW_omega[3]` (rad/s),
`v_sat_i[3]`, `r_sat_i[3]`, `r_p_i[3]` (target), `v_sun_i[3]`, `r_sun_i[3]`.

**Outputs ← DLL** (`ExtY_ADCS_Software_v1_T`):
`actual_ephem`, `actual_ads`, `mode_degraded`, `RW_tau[3]`, `THR_tau[3]`,
`MTQ_tau[3]` (**a voltage command despite the name**). Torques are in **ECI** axes.

**Block signals** (`B_ADCS_Software_v1_T`, diagnostics only):
`desired_ephem`, `desired_ads`, `desired_frame`, `desired_acs`, `actual_frame`, `actual_acs`,
`Compare`, `Compare_i`, `NOT`.

---

## 12. Runtime overrides (no file edit)

### Command line — [`scenario/cli.py`](adcs_sim/scenario/cli.py)

| Flag | Overrides |
|---|---|
| `--mode {file,live}` | run mode (default `file`) |
| `--seconds T` | `SIMULATION_SECONDS` |
| `--output-dir DIR` | `OUTPUT_DIR` |
| `--show-plots` | display figures as well as saving |
| `--rw-momentum RW1 RW2 RW3` | `DEFAULT_INITIAL_RW_MOMENTUM_NMS` [N·m·s] |
| `--rw-omega-rpm RW1 RW2 RW3` | initial wheel speeds [RPM] — mutually exclusive with the above |
| `--no-vizard` | `AUTO_LAUNCH_VIZARD = False` |
| `--vizard-exe PATH` | `VIZARD_EXECUTABLE` |

### Environment

| Variable | Effect |
|---|---|
| `VIZARD_EXECUTABLE` | Path to `Vizard.exe`; strict — a bad path errors rather than falling back |

---

## 13. Derived values — do not edit directly

These are computed from other constants. Editing them is either impossible or will be silently
overwritten; change the input instead.

| Derived | Computed from |
|---|---|
| `HUB_INERTIA_B` | `HUB_INERTIA_SUPPLIED × HUB_INERTIA_SCALE` |
| `INITIAL_TUMBLE_RATE_B` | `INITIAL_TUMBLE_RATE_MAGNITUDE_RAD_S / √3` |
| `DEFAULT_INITIAL_RW_MOMENTUM_NMS` | `RW_PHYSICAL_H_MAX` |
| `MTQ_DIPOLE_PER_VOLT` | `MTQ_MAX_DIPOLE / MTQ_MAX_VOLTAGE` |
| `RATED_SPEED_RAD_S` | `RW_PHYSICAL_H_MAX / CONTROLLER_I_RW` |
| `PACKAGE_ROOT`, `SCENARIO_DIR`, `REPO_ROOT`, `ADCS_*`, `OUTPUT_DIR` | `__file__` |
| `_DEFAULT_RPM` (help text) | `DEFAULT_INITIAL_RW_MOMENTUM_NMS / CONTROLLER_I_RW / RPM` |

> **Coupling worth knowing:** `RW_PHYSICAL_H_MAX` sets *both* the hardware capacity *and* the
> default initial momentum. To start somewhere other than saturation, edit
> `DEFAULT_INITIAL_RW_MOMENTUM_NMS`, not `RW_PHYSICAL_H_MAX`.

---

## 14. Values that are placeholders or estimates

Cross-cutting list of every number that is **not** traceable to vendor data. Full context in the
[model inventory](ADCS_SIM_MODEL_INVENTORY.md).

| Value | File | Status |
|---|---|---|
| `MTQ_MAX_VOLTAGE` = 5.0 V | `config/actuators.py` | **Placeholder** — replace from torque-rod data sheet |
| `MTQ_MAX_DIPOLE` = 0.2 A·m² | `config/actuators.py` | **Placeholder** — same |
| `THR_MAX_BODY_TORQUE` = 0.05 N·m | `config/actuators.py` | **Placeholder** — stands in for absent thruster geometry |
| `RW_VISCOUS_FRICTION` = 7.0e-6 | `config/actuators.py` | **Estimate** for this wheel class, not Honeywell data |
| `RW_STATIC_FRICTION` = 1.0e-3 | `config/actuators.py` | **Estimate** — and inert in this Basilisk build |
| `RW_STRIBECK_BETA` = 0.5 | `config/actuators.py` | **Estimate** — and inert |
| `TARGET_RADIUS_M` | `config/target.py` | **Spherical Earth**, no WGS-84 flattening |
| `HUB_INERTIA_SCALE` = 0.5 | `config/spacecraft.py` | Vehicle model supplies `L`, scenario flies `L/2` — confirm this is intended |
| Sensor/actuator interface | `flightsoftware/structs.py` | **Interface definition** — the Simulink model must take body rate / body-relative-to-inertial quaternion and output body-frame spacecraft torques to match |
| `RW_COULOMB_FRICTION` = 5.0e-4 | `config/actuators.py` | From Basilisk's HR16 profile — the one friction value with provenance |

---

## 15. Regenerating this index

The constant tables were produced by scanning module-level assignments:

```bash
rg "^[A-Z_][A-Z0-9_]* = " Projects/ADCS_software_test/adcs_sim -n
```

Re-run it after adding parameters and update the affected section. If a new `config/` module is
added, give it a section here and a row in §1.
