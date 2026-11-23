# ADCS Simulation — Sensor & Actuator Model Inventory

**Scenario under review:** `ADCS_software_test/simulator/` — entry point `scenarioADCSSoftwareRW.py`, implementation in the `adcs_sim/` package ([package map](adcs_sim/README.md))
**Flight software under test:** `ADCS_Software_v1_grt_rtw/ADCS_Software_v1.dll` (Simulink-generated, driven over ctypes)
**Environment:** Basilisk 2.11.0, Python 3.12.10, Windows
**Baseline audit date:** 2026-07-30
**Last structural change:** 2026-07-30 — monolith split into `adcs_sim/`; verified numerically identical (byte-identical Vizard recording over a 250 s run)
**Maintainer:** _fill in_

---

## 1. Purpose

This document is the single reference for **what physics is actually simulated** around the
generated ADCS controller, **what is idealised or missing**, and **whether closing each gap
needs a Basilisk built-in module or new hand-written code**.

Every model below carries a **Change record** row. When a model is upgraded, update its
status, date and note *in place* — do not delete the original baseline description, so the
fidelity history of the simulation stays traceable.

This document covers **how good** each model is. For **where every constant is defined**, see
the companion [`ADCS_SIM_DATA_INDEX.md`](ADCS_SIM_DATA_INDEX.md).

Each model's ID (`A1`, `S7`, `E4`, `C2` …) also appears in the docstring of the module that
implements it, so a row here and the code it describes are always one hop apart. Two rules
that matter when editing that code are documented in [`adcs_sim/README.md`](adcs_sim/README.md):
**model registration order is load-bearing**, and **Basilisk holds raw C++ pointers, so every
`add_*` function must return the objects that need to outlive it**.

---

## 2. How to use this document

### Status legend

| Mark | Meaning |
|:---:|---|
| `[x]` | Implemented **and** verified against expected physics |
| `[~]` | Partially implemented — real model present but simplified / ideal in known ways |
| `[ ]` | Not implemented — absent, or a placeholder standing in for real physics |
| `[-]` | Deliberately out of scope for this simulation (justify in Notes) |

### Effort legend (the bracket answer to "built-in or custom code?")

| Mark | Meaning |
|:---:|---|
| **BI** | **Built-in.** A Basilisk module already exists. Closing the gap is wiring + parameters only — no new C++/Python model code. |
| **BI+G** | **Built-in + glue.** Basilisk module exists, but custom adapter code is required because the generated DLL's interface does not match the module's message (e.g. unit conversion, torque→force allocation). |
| **CUS** | **Custom.** No Basilisk equivalent — must be written and maintained by us. |

### Change record template

Copy this into the model's section when it is modified:

```
| Change record | Status | Date | By | Notes |
|---|:---:|---|---|---|
| Baseline audit      | [~] | 2026-07-30 | — | as-built description below |
| <what you changed>  | [ ] |            |   |                           |
```

---

## 3. Status dashboard

| # | Model | Domain | Status | Effort to close gap |
|:--:|---|---|:---:|:---:|
| A1 | Reaction wheels (3× Honeywell HR16) | Actuator | `[~]` | **BI** |
| A2 | Magnetic torque rods (3× orthogonal) | Actuator | `[~]` | **BI+G** |
| A3 | Thrusters / desaturation | Actuator | `[ ]` | **BI+G** |
| S1 | Attitude + rate + orbit navigation (`SimpleNav`) | Sensor | `[~]` | **BI** |
| S2 | Star tracker (discrete) | Sensor | `[ ]` | **BI** |
| S3 | Gyro / IMU (discrete) | Sensor | `[ ]` | **BI** |
| S4 | Magnetometer (TAM) | Sensor | `[ ]` | **BI** |
| S5 | Sun sensor (CSS) | Sensor | `[ ]` | **BI** |
| S6 | RW tachometer / encoder | Sensor | `[~]` | **BI** |
| S7 | Ground-target geometry | Sensor/geometry | `[~]` | **BI** |
| S8 | Star tracker camera image (`STR 1`) | Sensor | `[~]` | **G** |
| E1 | Rigid-body spacecraft dynamics | Environment | `[x]` | — |
| E2 | Gravity (Earth + Sun) | Environment | `[~]` | **BI** |
| E3 | Ephemeris / time (SPICE) | Environment | `[x]` | — |
| E4 | Earth magnetic field | Environment | `[~]` | **BI** |
| E5 | Aerodynamic drag | Disturbance | `[ ]` | **BI** |
| E6 | Solar radiation pressure | Disturbance | `[ ]` | **BI** |
| E7 | Gravity-gradient torque | Disturbance | `[ ]` | **BI** |
| E8 | Residual magnetic dipole | Disturbance | `[ ]` | **BI** |
| E9 | Eclipse / albedo | Disturbance | `[ ]` | **BI** |
| P1 | Solar array (power generation) | Power | `[~]` | **BI** |
| P2 | Eclipse illumination (Earth shadow) | Power | `[x]` | **BI** |
| C1 | Generated-DLL wrapper & sensor/actuator interface | Interface | `[x]` | **CUS** |

**Headline finding of the baseline audit:** the actuator chain is partially real, but
**every sensor path is error-free truth data**. The control loop is currently exercised with
perfect knowledge — no noise, bias, latency, quantisation or sensor failures. Closing that
gap requires **no new model code**, only enabling built-in Basilisk modules.

---

## 4. Actuator models

### A1 — Reaction wheels: 3× Honeywell HR16, orthogonal

**Where:** [`adcs_sim/models/reaction_wheels.py`](adcs_sim/models/reaction_wheels.py) · config [`adcs_sim/config/actuators.py`](adcs_sim/config/actuators.py) · wiring [`adcs_sim/scenario/builder.py`](adcs_sim/scenario/builder.py)
**Basilisk modules:** `simulation.reactionWheelStateEffector`, `utilities.simIncludeRW`

| Change record | Status | Date | By | Notes |
|---|:---:|---|---|---|
| Baseline audit | `[~]` | 2026-07-30 | — | as-built below |
| **Enabled Coulomb friction + minimum-torque deadband** (backlog #4) | `[~]` | 2026-07-30 | — | `RW_USE_FRICTION = True`, `RW_USE_MIN_TORQUE = True`. Verified on the devices: `fCoulomb` 0 → `5.0e-4 N·m`, `u_min` 0 → `1.0e-5 N·m`. Measured effect recorded below. Still `[~]`: jitter, viscous/Stribeck friction, motor electrical, power and encoder remain unmodelled |
|  |  |  |  |  |

**Physics that IS modelled**

| Item | Value / behaviour |
|---|---|
| Effector model | `RWModel = BalancedWheels` (rwFactory default) |
| Hub coupling | Full — wheel spin momentum enters the hub EOM, gyroscopic cross-coupling included |
| Spin inertia `Js` | `0.0795775 kg·m²`, derived by the factory from `maxMomentum / Omega_max` = `50 / (6000·RPM)` |
| Momentum capacity | `50 N·m·s` per wheel (`maxMomentum=50`, the HR16 "small" variant) |
| Motor torque saturation | `u_max = 0.2 N·m`, active (`useMaxTorque=True` passed explicitly) |
| Wheel speed limit | `Omega_max = 6000 RPM` (`628.32 rad/s`) configured by the HR16 factory |
| **Coulomb friction** | `fCoulomb = 5.0e-4 N·m`, opposing wheel rotation — **enabled 2026-07-30** |
| **Viscous friction** | `cViscous = 7.0e-6 N·m·s/rad` — **enabled 2026-07-31**. Dominant term at speed: `4.90e-3 N·m` drag at 6000 RPM (2.45% of `u_max`), `3.1 W` bearing loss, `Js/cViscous = 3.2 h` coast-down |
| **Minimum-torque deadband** | `u_min = 1.0e-5 N·m`; smaller commands are zeroed — **enabled 2026-07-30** |
| Spin axes | `[1,0,0]`, `[0,1,0]`, `[0,0,1]` body-fixed |
| Redundant clip | `reaction_wheels.limit_motor_torque()` also clips to `±u_max` before the message is written |

**Physics that is NOT modelled** — and why

Every row below is a one-line change in [`config/actuators.py`](adcs_sim/config/actuators.py); the
switch is already read by `reaction_wheels._factory_options()`.

| Missing item | Cause | Effort |
|---|---|:---:|
| ~~Coulomb friction~~ | ✅ **DONE 2026-07-30** — `RW_USE_FRICTION = True` restores the HR16 profile's `fCoulomb = 5.0e-4 N·m` | — |
| ~~Minimum torque deadband~~ | ✅ **DONE 2026-07-30** — `RW_USE_MIN_TORQUE = True` restores `u_min = 1.0e-5 N·m` | — |
| ~~Viscous friction~~ | ✅ **DONE 2026-07-31** — `RW_VISCOUS_FRICTION = 7.0e-6 N·m·s/rad` | — |
| **Static / Stribeck friction** | ⛔ **BLOCKED, not merely unset.** `fStatic`/`betaStatic` are accepted by `rwFactory` and land on the device, but Basilisk 2.11.0's `ReactionWheelStateEffector` **never applies a Stribeck term** — see the measurement below. Values are configured and will activate if a future Basilisk implements it | **CUS** — requires modifying the effector |
| Static/dynamic imbalance jitter | `RWModel` left at `BalancedWheels`; `U_s`/`U_d` populated by the factory but unused | **BI** — `RW_IMBALANCE_MODEL = messaging.JitterSimple` |
| Motor voltage / current dynamics | No voltage interface in the loop | **BI** — `fswAlgorithms.rwMotorVoltage` + `simulation.motorVoltageInterface` |
| Power draw / bus loading | Not modelled | **BI** — `simulation.ReactionWheelPower` |
| Speed-sensor quantisation | See S6 | **BI** — `simulation.encoder` |

**Friction coefficients — where to set them**

Basilisk's wheel friction torque is
`τ = −sign(Ω)·[fCoulomb + Stribeck(fStatic, betaStatic)] − cViscous·Ω`.
All coefficients are exposed in [`config/actuators.py`](adcs_sim/config/actuators.py); each
defaults to `None`, meaning *keep the HR16 device profile's value* — which is **not** the same
as zero.

| Config name | Device field | Meaning | Profile default |
|---|---|---|---|
| `RW_COULOMB_FRICTION` | `fCoulomb` | Dry friction torque [N·m] — **the main knob** | `5.0e-4` |
| `RW_STATIC_FRICTION` | `fStatic` | Breakaway/stiction magnitude [N·m] | `0.0` |
| `RW_STRIBECK_BETA` | `betaStatic` | Stribeck on (`>0`) / off (`<0`); never `0.0` | `-1.0` (off) |
| `RW_VISCOUS_FRICTION` | `cViscous` | Speed-proportional [N·m·s/rad] | `0.0` |
| `RW_LIMIT_CYCLE_SPEED` | `omegaLimitCycle` | Numerical limit-cycle guard [rad/s] | `1.0e-4` |

**Measured friction law** (Basilisk 2.11.0, read back from the effector's own
`frictionTorque` output at 11 speeds from `7.6e-8` to `628 rad/s`):

```
|Ω| ≥ omegaLimitCycle:   τ = −sign(Ω)·fCoulomb − cViscous·Ω
|Ω| <  omegaLimitCycle:  τ ramps linearly from 0 to that value
```

| Ω [rad/s] | Ω [RPM] | friction [N·m] | % of `u_max` |
|---:|---:|---:|---:|
| 1.3e-05 | 0.0 | 6.31e-05 | 0.03 |
| 1.9e-04 | 0.0 | 5.00e-04 | 0.25 |
| 0.50 | 4.8 | 5.03e-04 | 0.25 |
| 10.0 | 95 | 5.70e-04 | 0.28 |
| 100.0 | 955 | 1.20e-03 | 0.60 |
| 628.3 | 6000 | 4.90e-03 | 2.45 |

> **⛔ The Stribeck term does not exist in this build.** Friction never rises toward
> `fStatic` at *any* speed — including with `frictionStribeck` forced `True` and with
> `betaStatic = 0.5` confirmed present on the device. The values below `omegaLimitCycle` are a
> linear anti-chatter ramp, **not stiction**: a stopped wheel is not held by static friction.
> Anything depending on breakaway behaviour is not being modelled.

> **Trap, now guarded:** `rwFactory` applies `if not useRWfriction: fCoulomb = 0` *after*
> processing keyword arguments, so a coefficient set with `RW_USE_FRICTION = False` is silently
> discarded — and that gate zeroes `fCoulomb` **only**, leaving `fStatic`/`cViscous` active, so
> "friction off" would not have meant friction off. `reaction_wheels.validate_friction_config()`
> now raises on that combination. Every run also prints the active coefficients in its banner.

**Measured effect of enabling friction + deadband** (250 s runs, both cases)

| Quantity | Friction OFF | Friction ON | Reading |
|---|---|---|---|
| **Fine pointing** (`--rw-momentum 0 0 0`) | | | |
| Steady-state `RW_tau` RMS, per axis | `2.7e-09` / `4.6e-07` / — N·m | `4.32e-04` / `5.00e-04` / `5.00e-04` N·m | Commanded torque is now **pinned at `fCoulomb`** — the loop continuously pushes against friction to hold attitude. This is the textbook Coulomb signature and the single biggest behavioural change. |
| Body-rate magnitude at 200 s | `6.308845e-04` rad/s | `6.308871e-04` rad/s | Pointing performance **unchanged** — the loop absorbs the friction. |
| RW1 wheel-speed peak-to-peak | `3.111e-03` rad/s | `9.473e-05` rad/s | **33× less wheel-speed jitter**: the deadband suppresses low-level chatter. |
| **Desaturation** (default saturated start) | | | |
| RW1 / RW2 / RW3 Δspeed | `-109.51` / `-113.92` / `-112.04` rad/s | `-110.01` / `-114.32` / `-112.49` rad/s | Wheels spin down slightly **faster** — friction adds to the commanded deceleration. |
| Peak `RW_tau` | `4.992e-02` N·m | `4.940e-02` N·m | Controller commands slightly **less** torque, because friction is helping. |
| Body-rate magnitude at 200 s | `8.118e-02` rad/s | `8.037e-02` rad/s | Marginally lower. |

> **⚠ Regime note:** with steady-state torque now sitting at `5.0e-4 N·m`, the deadband
> (`u_min = 1.0e-5 N·m`) is **50× below the friction level** and is therefore almost never the
> binding constraint in steady state — friction is. Do not expect the deadband to matter until
> the friction model is refined or a much finer pointing mode is exercised.
>
> **Re-baseline owed:** all archived plots and any recorded pass/fail thresholds from before
> 2026-07-30 predate this change. `rw_tau_after_200s.png` in particular now shows a
> ~1000× larger steady-state torque in the unsaturated case.
> **Re-baselined?** `[ ]` — date: ______ — by: ______

---

### A2 — Magnetic torque rods: 3× orthogonal

**Where:** [`adcs_sim/models/magnetorquers.py`](adcs_sim/models/magnetorquers.py) — including `limit_voltage()` and `voltage_to_dipole()` · ratings [`adcs_sim/config/actuators.py`](adcs_sim/config/actuators.py)
**Basilisk modules:** `simulation.MtbEffector`, `messaging.MTBArrayConfigMsg`

| Change record | Status | Date | By | Notes |
|---|:---:|---|---|---|
| Baseline audit | `[~]` | 2026-07-30 | — | as-built below |
|  |  |  |  |  |

**Physics that IS modelled**

- `MtbEffector` applies the true physical torque `τ_B = (Gt · m) × B_B`, using the live
  magnetic field from E4 — i.e. authority correctly collapses when the commanded dipole is
  parallel to **B**.
- Alignment matrix `GtMatrix_B = I` (three orthogonal, body-aligned rods), `numMTB = 3`.
- Per-rod dipole saturation against `maxMtbDipoles = 0.2 A·m²`.

**Physics that is NOT modelled**

| Missing item | Cause | Effort |
|---|---|:---:|
| **Voltage→dipole is a hand-written constant** | The DLL emits **volts**; `MTQ_DIPOLE_PER_VOLT = 0.2/5.0` is a linear placeholder gain applied by `voltage_to_dipole()` | **CUS** (glue must exist) — but the *ratings* are the real issue, see below |
| Coil electrical dynamics (R, L, current rise time) | Not modelled — dipole responds instantly | **CUS** |
| Core hysteresis / remanence | Not modelled | **CUS** |
| Rod self-field corrupting the magnetometer | Not modelled | **CUS** |
| Duty-cycling / MTQ-vs-TAM time multiplexing | Not modelled — rods are always on | **CUS** |

> **⚠ Open action:** `MTQ_MAX_VOLTAGE = 5.0 V` and `MTQ_MAX_DIPOLE = 0.2 A·m²` in
> `config/actuators.py` are **placeholders**. Replace both with the flight torque-rod
> data-sheet values.
> **Replaced?** `[ ]` — date: ______ — by: ______

---

### A3 — Thrusters (reaction-wheel desaturation)

**Where:** [`adcs_sim/models/thrusters.py`](adcs_sim/models/thrusters.py) — including `limit_body_torque()` · limit [`adcs_sim/config/actuators.py`](adcs_sim/config/actuators.py)
**Currently used:** `simulation.extForceTorque` — a *generic ideal torque source*, **not** a thruster model

| Change record | Status | Date | By | Notes |
|---|:---:|---|---|---|
| Baseline audit | `[ ]` | 2026-07-30 | — | placeholder only |
|  |  |  |  |  |

**What happens today**

`ExtForceTorque` applies the DLL's commanded 3-axis body torque **directly to the hub**, after
`limit_body_torque()` clips it to `±THR_MAX_BODY_TORQUE = 0.05 N·m` per axis. The clip exists
purely to keep the integration stable: the generated DESAT law currently returns roughly
`−RW_omega`, i.e. **~628 N·m at 6000 RPM**, which would otherwise be applied literally.

**Everything about real thrusters is absent**

| Missing item | Effort |
|---|:---:|
| Thruster count, locations `r_B`, thrust directions | **BI** — `utilities.simIncludeThruster` |
| Force → torque via moment arm (torque currently conjured with no force) | **BI** |
| Thrust rise/decay transients | **BI** — `simulation.thrusterStateEffector` |
| Discrete on/off firing, minimum impulse bit | **BI** — `fswAlgorithms.thrFiringSchmitt` / `thrFiringRemainder` / `thrFiringRound` |
| Torque→per-thruster-force allocation | **BI** — `fswAlgorithms.thrForceMapping` / `forceTorqueThrForceMapping` |
| Propellant depletion, mass-property change | **BI** — `simulation.fuelTank` |
| Translational ΔV coupling (real thrusters perturb the orbit) | **BI** |
| Dedicated RW-desat logic | **BI** — `fswAlgorithms.thrustRWDesat`, `thrMomentumManagement`, `thrMomentumDumping` |

> **⚠ Open action:** replace the ideal-torque placeholder with real thruster geometry
> (force magnitude + moment arm) and a firing model.
> **Note the split:** the *physics* is **BI**, but the DLL outputs a torque vector while
> `thrusterDynamicEffector` needs per-thruster on-times — so a torque→force→on-time adapter
> in `flightsoftware/controller.py` is **required custom glue (BI+G)**.
> **Replaced?** `[ ]` — date: ______ — by: ______

---

## 5. Sensor models

> **Read this first.** No discrete sensor model exists anywhere in this scenario. The
> controller is fed **truth**: perfect attitude, perfect rates, perfect orbit, perfect
> magnetic field, perfect Sun vector, perfect wheel speeds. Any conclusion drawn from this
> simulation about pointing accuracy, estimator convergence, or fault response is therefore
> **optimistic by an unquantified margin**.

### S1 — Navigation: attitude, body rate, position, velocity (`SimpleNav`)

**Where:** [`adcs_sim/models/navigation.py`](adcs_sim/models/navigation.py) · config [`adcs_sim/config/sensors.py`](adcs_sim/config/sensors.py)
**Basilisk module:** `simulation.simpleNav`

| Change record | Status | Date | By | Notes |
|---|:---:|---|---|---|
| Baseline audit | `[~]` | 2026-07-30 | — | module present, error model disabled |
|  |  |  |  |  |

- **Capability:** `SimpleNav` supports a Gauss-Markov noise + bounded random-walk error model
  on all outputs.
- **As configured: `NAV_ERROR_ENABLED = False`, so `PMatrix` and `walkBounds` are never
  touched** and the error model is identically zero. The module is a pass-through of truth.
- Supplies `sigma_BN`, `omega_BN_B` (via `attOutMsg`) and `r_BN_N`, `v_BN_N` (via
  `transOutMsg`) — it single-handedly stands in for star tracker + gyro + GPS.
- `sunStateInMsg` is **not** connected, so `vehSunPntBdy` is unused; the Sun vector comes
  straight from SPICE instead (see S5).
- Each run prints which state it is in (`navigation error model: OFF (truth state)`), so a
  truth-state result can never be mistaken for a noisy one in the logs.

**To close:** populate `NAV_P_MATRIX` (18×18 noise std-dev) and `NAV_WALK_BOUNDS`, then set
`NAV_ERROR_ENABLED = True`. **BI — no new code.**

**Enabled?** `[ ]` — date: ______ — by: ______

---

### S2 — Star tracker · S3 — Gyro/IMU · S4 — Magnetometer · S5 — Sun sensor

None of these exist in the scenario. Each has a ready Basilisk module.

| ID | Sensor | What the DLL gets today | Missing error sources | Built-in replacement | Status |
|:--:|---|---|---|---|:---:|
| S2 | Star tracker | truth `sigma_BN` from S1 | noise, bias, misalignment, update rate, Sun/Earth exclusion cones, loss-of-lock | `simulation.starTracker` | `[ ]` **BI** |
| S3 | Gyro / IMU | truth `omega_BN_B` from S1 | ARW, rate-random-walk bias, scale factor, misalignment, saturation, quantisation | `simulation.imuSensor` | `[ ]` **BI** |
| S4 | Magnetometer (TAM) | truth `magField_N` from E4, rotated to body **by hand** in `controller._write_inputs` | bias, noise, saturation, sensor-frame misalignment, MTQ self-field | `simulation.magnetometer` (`MagnetometerTAM`) | `[ ]` **BI** |
| S5 | Sun sensor (CSS) | truth Sun position/velocity straight from SPICE, via `builder.connect_messages` | field-of-view limits, cosine response, noise, eclipse gating, albedo corruption | `simulation.coarseSunSensor` (+ `eclipse`, `albedo`); estimators `fswAlgorithms.sunlineEKF` / `sunlineUKF` / `sunlineSEKF` / `okeefeEKF` | `[ ]` **BI** |

> For S4: note that `magnetic_field_B = dcm_BN @ magnetic.magField_N` in
> `flightsoftware/controller.py` performs the inertial→body rotation *using truth attitude*.
> A real TAM measures in the sensor frame and the FSW must rotate using its *estimated*
> attitude — this is a structural shortcut, not just a missing noise term.

**Enabled?** S2 `[ ]` · S3 `[ ]` · S4 `[ ]` · S5 `[ ]` — date: ______ — by: ______

---

### S8 — Star tracker camera image (`STR 1`)

`models/star_tracker_camera.py` (`StarTrackerCamera`, config
`config/star_tracker_camera.py`) renders the flight software's `STR 1` frames
**inside the simulation**, from the true attitude, every
`FRAME_INTERVAL_S` — no Vizard, no files, repeatable run to run. Before
2026-10-12 the frames were Vizard renders read back from disk (still available
with `IMAGE_SOURCE = "vizard"` or `ADCS_STR_SOURCE=vizard`).

| Modelled | Not modelled |
|---|---|
| Hipparcos stars to V 6.99 at J2000 (independent of the FSW's 1102-star catalogue); pinhole optics from the shared FOV / resolution / `DCM_CB`; pixel-integrated Gaussian PSF; motion blur over the exposure; dark, sky, shot and read noise, gain, bias, 8-bit clipping; Earth occultation with a saturated day side; Sun-exclusion blinding | Moon and planets; stray light short of blinding; optical distortion; hot pixels; rolling shutter; aberration; proper motion beyond J2000 |

Check after any model or camera change: `python tools/star_tracker_render_check.py`
(feeds rendered frames to the DLL and reports the solve error). On 2026-10-12:
40/40 frames solved, median 2.9 arcsec; ~2-3 ms per frame.

> **Sky rotation.** The current FSW applies a fixed 90° rotation (`j` in
> `'<Root>/MATLAB Function'`) that compensates Vizard's skybox, which is drawn
> rotated 90° about the celestial pole. Against the true sky that makes
> `q_BI_est` exactly 90° wrong, so `SKY_ROTATION_Z_DEG = 90` rotates the
> rendered sky to match and every run prints a warning. Remove `j` from the
> model, then set it to 0.

**Status** `[~]` — rendered in-sim since 2026-10-12; `Camerafeed` (payload
camera) still comes from Vizard.

---

### S6 — Reaction-wheel speed feedback

**Where:** wiring in [`adcs_sim/scenario/builder.py`](adcs_sim/scenario/builder.py) · read into the DLL input `RW_omega` in `controller._write_inputs`

The controller reads `rw_effector.rwSpeedOutMsg` — the **exact integrator state** of each
wheel. No quantisation, no tachometer noise, no dropout, no latency.

**To close:** insert `simulation.encoder` between the RW effector and the controller
(one extra subscription in `connect_messages`).
`[ ]` **BI** — date: ______ — by: ______

---

### S7 — Ground-target geometry

**Where:** [`adcs_sim/geometry/ground_target.py`](adcs_sim/geometry/ground_target.py) · config [`adcs_sim/config/target.py`](adcs_sim/config/target.py)

| Change record | Status | Date | By | Notes |
|---|:---:|---|---|---|
| Baseline audit | `[~]` | 2026-07-30 | — | spherical Earth, hand-written |
|  |  |  |  |  |

- Target: lat `28.522431782788708°N`, lon `77.02428245359896°E`.
- **Uses a sphere of radius `6 378 137 m` — no WGS-84 flattening.** At this latitude the
  geodetic-vs-geocentric difference is on the order of tens of kilometres in position, which
  maps to a small but non-zero pointing bias.
- Converted to ECI with SPICE's `J20002Pfix` DCM — that part is correct.
- No access window / elevation-mask / range computation.

**To close:** replace with `simulation.groundLocation`, which handles the reference ellipsoid,
access checks, elevation mask and range. `[ ]` **BI** — date: ______ — by: ______

---

## 6. Environment and dynamics

Implemented in [`adcs_sim/models/hub.py`](adcs_sim/models/hub.py) and
[`adcs_sim/models/environment.py`](adcs_sim/models/environment.py); configured in
[`adcs_sim/config/spacecraft.py`](adcs_sim/config/spacecraft.py) and
[`adcs_sim/config/environment.py`](adcs_sim/config/environment.py).

| ID | Model | As configured | Status | Gap / upgrade path |
|:--:|---|---|:---:|---|
| E1 | Spacecraft hub — `simulation.spacecraft` | Rigid body, `m = 750 kg`, `r_BcB_B = 0`, inertia = supplied matrix ÷ 2 (`diag ≈ [101.3, 57.9, 80.7]` with products of inertia). Integrator `svIntegratorRKF45`, `relTol=1e-8`, `absTol=1e-10` | `[x]` | Adequate. Flexible appendages / fuel slosh available built-in (`hingedRigidBodyStateEffector`, `sphericalPendulum`, `linearSpringMassDamper`) if ever needed |
| E2 | Gravity — `utilities.simIncludeGravBody` | **Point-mass** Earth (central body) + point-mass Sun third-body. `createBodies()` default does **not** load spherical harmonics — **no J2** | `[~]` | `[ ]` **BI** — `USE_SPHERICAL_HARMONICS`; the switch exists and raises `NotImplementedError` until the coefficient load is written |
| E3 | Ephemeris & time — `simulation.spiceInterface` | Epoch `2026 JULY 13 06:30:00.0` UTC, `zeroBase = "Earth"` so all vectors are Earth-centred J2000. Planet-message indices are resolved by name via `environment.planet_state_index()` rather than hard-coded | `[x]` | — |
| E4 | Earth magnetic field — `magneticFieldCenteredDipole` | Tilted centred-dipole, Earth coefficients from `simSetPlanetEnvironment.centeredDipoleMagField` | `[~]` | `[ ]` **BI** — `USE_WMM_MAGNETIC_FIELD`; matters for MTQ authority and TAM-based attitude |

**Orbit:** `a = 10 000 km`, `e = 0.01`, `i = 33.3°`, `Ω = 48.2°`, `ω = 347.8°`, `f = 85.3°`
**Initial attitude:** LVLH-aligned (`+Z` nadir, `−Y` along orbit normal), `sigma_BN` from that DCM
**Initial body rate:** `INITIAL_TUMBLE_RATE_MAGNITUDE_RAD_S = 0.0` — **no tumble**. The original
monolith's comment claimed "exactly 10 rad/s total angular-rate magnitude", which was never
true of the shipped value; the misleading comment is gone. Detumble is **not** being exercised
in the current baseline.
**Detumble case re-enabled?** `[ ]` — date: ______ — by: ______

> All of the above lives in [`adcs_sim/config/spacecraft.py`](adcs_sim/config/spacecraft.py) and
> [`adcs_sim/models/initial_state.py`](adcs_sim/models/initial_state.py). Each run now prints its
> initial body rate next to the LVLH reference rate, so the two cannot silently diverge again.
> `omega_LVLH_B` — dead code in the monolith — is returned in `InitialState` and reported.

---

## 6b. Power generation — solar array

Implemented in [`adcs_sim/models/solar_panels.py`](adcs_sim/models/solar_panels.py);
configured in [`adcs_sim/config/power.py`](adcs_sim/config/power.py); recorded, plotted and
summarised alongside the attitude telemetry.

**Telemetry only.** The panels read the spacecraft state and the Sun ephemeris and write
power messages. No dynamic effector, no torque, no force, and nothing fed back to the flight
software — the DLL has no power input port. Setting `POWER_MODEL_ENABLED = False` leaves the
control loop bit-for-bit unchanged, which is the property to re-check if these models are
ever extended.

| ID | Model | As configured | Status | Gap / upgrade path |
|:--:|---|---|:---:|---|
| P1 | Solar array — `simulation.simpleSolarPanel` | 2 wings on ±Y, 2.0 m² each, 30 % efficiency, cells facing outward. Per panel: `netPower = 1361 W/m² · (AU/r)² · area · efficiency · max(0, cos θ) · sunlit fraction` (measured on Basilisk 2.11.1) | `[~]` | **Placeholder geometry** — replace area, efficiency and normals with the real array. Not modelled: cell temperature, degradation, string failures, MPPT behaviour, array articulation |
| P2 | Eclipse illumination — `simulation.eclipse` | Earth as the occulting body, penumbra included; drives the sunlit fraction on every panel (message field `shadowFactor` in this build: 1 = full sun, 0 = umbra) | `[x]` | — (albedo and Earth IR remain unmodelled; they are the E9 disturbance side) |

**Power monitor:** `simulation.simplePowerMonitor` sums the panels and integrates the energy.
With `BUS_LOAD_W = 0` (the default) it reports **generation only**, so a figure on the live
plot is unambiguous; set a positive load to see the net charge/discharge balance instead.

**Where it shows up:** third panel of the live strip chart (`--mode live`), the `solar_power`
figure in file mode, and the power lines of the printed run summary.

---

## 7. Disturbance torques — none are modelled

The spacecraft currently flies in a torque-free environment apart from the actuators. This
makes momentum build-up, and therefore the whole desaturation story, unrepresentative.

| ID | Disturbance | Built-in module(s) | Status |
|:--:|---|---|:---:|
| E5 | Aerodynamic drag | `exponentialAtmosphere` / `msisAtmosphere` / `tabularAtmosphere` + `dragDynamicEffector` / `facetDragDynamicEffector` / `cannonballDrag` | `[ ]` **BI** |
| E6 | Solar radiation pressure | `radiationPressure` / `facetSRPDynamicEffector` / `facetedSRPEffector` | `[ ]` **BI** |
| E7 | Gravity-gradient torque | `GravityGradientEffector` | `[ ]` **BI** |
| E8 | Residual magnetic dipole | *(no dedicated module — add a constant dipole to the MTB command, or a small `ExtForceTorque`)* | `[ ]` **BI+G** |
| E9 | Eclipse / albedo | `eclipse`, `albedo`, `earthRadiationModel` | `[ ]` **BI** |

> At `a = 10 000 km` (≈3 600 km altitude) drag is negligible, so **E5 is a defensible `[-]`**
> for this orbit — mark it so and record the justification. E6 and E7 are **not** negligible
> and are the dominant secular momentum sources that a desat cycle must handle.

**Reviewed and dispositioned?** `[ ]` — date: ______ — by: ______

---

## 8. Custom code that must remain custom

| ID | Item | Where | Why it cannot be a built-in |
|:--:|---|---|---|
| C1 | The DLL bridge | [`adcs_sim/flightsoftware/`](adcs_sim/flightsoftware/) — `structs.py`, `library.py`, `diagnostics.py`, `controller.py` | ctypes bridge to `ADCS_Software_v1.dll`; mirrors `ExtU_`/`ExtY_`/`B_` structs, marshals Basilisk messages in/out, exposes `ADCS_set_mission_cmd` etc. No Basilisk equivalent — **CUS**. `structs.py` **must** be updated whenever the Simulink model's port list changes |
| C2 | Sensor/actuator interface | Defined in [`adcs_sim/flightsoftware/structs.py`](adcs_sim/flightsoftware/structs.py), implemented by `controller._write_inputs` / `_read_outputs` | Since 2026-09-15 a realistic pass-through: `q_BI` = body-relative-to-inertial quaternion (Basilisk `q_BN`), `omega` = `omega_BN_B`, `RW_tau`/`THR_tau` applied as body torques in every mode. No frame conversion and no ACS-mode dependence. **CUS** — the Simulink model must match this interface |
| C3 | Actuator limit clipping at the controller boundary | `reaction_wheels.limit_motor_torque()`, `magnetorquers.limit_voltage()`, `thrusters.limit_body_torque()` | Defensive `nan_to_num` + clip so a diverging DLL command cannot destabilise the integrator. **CUS** — but should shrink as real actuator models take over saturation. Now co-located with the actuator each one guards |
| C4 | Vizard integration: keyboard interface, launcher, spacecraft model | [`live.py`](adcs_sim/scenario/live.py), [`vizard_app.py`](adcs_sim/scenario/vizard_app.py), [`spacecraft_model.py`](adcs_sim/scenario/spacecraft_model.py) | Two-way socket driving `set_mission_command`, auto-launch of the viewer, and the `m4v.glb` CAD model. **CUS** — Basilisk ships no Vizard launcher. See the start-up race note in `vizard_app` and the rotation-convention note in `spacecraft_model`. Visualisation only: none of this affects the dynamics |
| C5 | Plotting / telemetry summary | [`adcs_sim/analysis/`](adcs_sim/analysis/) — `derived.py`, `plots.py`, `summary.py` | Project-specific. **CUS**. Plots and summary now read the *same* `DerivedTelemetry`, so they cannot disagree about what they show |

> **C2 verification owed:** the empirical sign/conjugation constants were removed on
> 2026-09-15; the bridge now passes sensor values through unchanged and applies body-frame
> torques. Generated models up to 2.99 still expect the old interface and diverge in closed loop
> until the Simulink model does its own inertial↔body conversions. A dedicated interface test
> (known attitude and body rate in → known body torque out) should be written against the
> updated model.
> **Test written?** `[ ]` — date: ______ — by: ______

---

## 9. Prioritised upgrade backlog

Ordered by impact-per-unit-effort. Each row is independently deliverable.

| # | Action | Effort | Impact | Status | Date | By |
|:--:|---|:---:|---|:---:|---|---|
| 1 | Replace placeholder MTQ voltage/dipole ratings with flight data-sheet values | **BI+G** | Removes a known-wrong parameter | `[ ]` | | |
| 2 | Enable `SimpleNav` noise (`PMatrix`, `walkBounds`) | **BI** | First honest look at closed-loop performance | `[ ]` | | |
| 3 | Add gravity-gradient (E7) + SRP (E6) | **BI** | Makes momentum build-up and desat need realistic | `[ ]` | | |
| 4 | ~~Enable RW friction + min-torque deadband~~ | **BI** | Realistic wheel drift and low-torque behaviour | `[x]` | 2026-07-30 | — |
| 5 | Add J2 gravity | **BI** | Correct orbit precession over long runs | `[ ]` | | |
| 6 | Replace ideal thruster torque with `thrusterDynamicEffector` + firing logic | **BI+G** | The single largest fidelity gap | `[ ]` | | |
| 7 | Swap hand-rolled target geometry for `groundLocation` (WGS-84) | **BI** | Removes geodetic pointing bias | `[ ]` | | |
| 8 | Add discrete sensors: `starTracker`, `imuSensor`, `magnetometer`, `coarseSunSensor` + `eclipse` | **BI** | Exercises the estimator for the first time | `[ ]` | | |
| 9 | Upgrade magnetic field to `magneticFieldWMM` | **BI** | Realistic MTQ authority | `[ ]` | | |
| 10 | Add `encoder` on RW speed feedback | **BI** | Quantisation effects on the control law | `[ ]` | | |
| 11 | Write a frame-convention unit test for C2 | **CUS** | Protects the riskiest code in the package | `[ ]` | | |
| 12 | Restore a non-zero initial tumble to exercise detumble | **BI** | Currently a dead code path | `[ ]` | | |

> Items 2, 5, 7 and 9 are single-line changes in `adcs_sim/config/` — the switch is already
> declared and read (item 4 was exactly this, and took two lines). Items 1 and 12 are
> single-value changes. Items 3, 6, 8 and 10 need a new `models/` module plus one call in
> `scenario/builder.py`.

---

## 10. Verification checklist (re-run after any change above)

| Check | Pass? | Date | Notes |
|---|:---:|---|---|
| `--mode file` completes 5000 s without non-finite samples in any torque log | `[ ]` | | |
| `RW_tau`, `MTQ_tau`, `THR_tau` all report `non-finite samples = 0` | `[ ]` | | |
| Body-rate magnitude converges (final ≪ initial) | `[ ]` | | |
| Earth/Sun/target body-frame direction plots settle to the commanded alignment | `[ ]` | | |
| No wheel exceeds `Omega_max` (628.32 rad/s) or `h_max` (50 N·m·s) | `[ ]` | | |
| Run banner reports the expected fidelity switches (`RW friction: ON, RW min-torque deadband: ON`) | `[ ]` | | |
| Steady-state `RW_tau` sits near `fCoulomb` (5.0e-4 N·m) in fine pointing, not orders below it — if it is far below, friction silently got disabled | `[ ]` | | |
| ACS/ephemeris/ADS mode histories show the expected mode sequence | `[ ]` | | |
| `--mode live` still connects to Vizard on `tcp://127.0.0.1:5556` | `[ ]` | | |

---

## 11. Change log

| Date | Change | Sections touched | By |
|---|---|---|---|
| 2026-07-31 | **Model fidelity increased:** RW viscous friction enabled (`cViscous = 7.0e-6 N·m·s/rad`), sized so drag at the 6000 RPM rating is 2.45% of peak motor torque (3.1 W loss, 3.2 h coast-down). Static/Stribeck values also set but **measured to be inert in Basilisk 2.11.0** — documented rather than assumed working. Friction law measured from the effector's `frictionTorque` output. Results before this date are not comparable | A1, §9, §10 | — |
| 2026-07-30 | Baseline model inventory created from the monolithic `scenarioADCSSoftwareRW.py` | all | — |
| 2026-07-30 | Spacecraft drawn with the `3d models/m4v.glb` CAD model (offset `(0.75, -0.6, -0.75)` m, rotation `90 deg` about body x), applied in both live and file modes. **Visualisation only -- no dynamics or mass-properties change**; the inertia in `config/spacecraft.py` is still the supplied matrix and is unaffected by the model or its offset. Confirmed loaded via Vizard's log | C4, §11 | — |
| 2026-07-30 | `--mode live` now launches Vizard automatically (`Vizard.exe -directComm tcp://127.0.0.1:5556`) and streams with no UI interaction; verified over 3/3 cold starts. Added `--no-vizard` / `--vizard-exe`. **No model change.** Documented the start-up race: Basilisk waits forever, Vizard times out, so Vizard is launched and `InitializeSimulation` follows immediately rather than after a blocking socket wait | C4, §11 | — |
| 2026-07-30 | All five RW friction coefficients exposed in `config/actuators.py` (`fCoulomb`, `fStatic`, `betaStatic`, `cViscous`, `omegaLimitCycle`), each defaulting to `None` = keep profile value. **No behaviour change** — verified the devices still carry the profile values. Added `validate_friction_config()` to reject coefficients set while `RW_USE_FRICTION` is False, which `rwFactory` would silently half-discard; run banner now prints the active coefficients | A1 | — |
| 2026-07-30 | **Model fidelity increased (backlog #4):** RW Coulomb friction (`5.0e-4 N·m`) and minimum-torque deadband (`1.0e-5 N·m`) enabled. First change in this simulation that alters the physics — results before and after this date are not comparable. Measured effect tabulated in §A1 | A1, §3, §9, §10 | — |
| 2026-07-30 | Monolith split into the `adcs_sim/` package; no model changed. Verified equivalent: identical printed telemetry and a **byte-identical Vizard recording** over a 250 s run. Fidelity switches added (all defaulting to the previous behaviour); `--output-dir` and `--seconds` added; `--show-plots` now actually displays (it was a no-op under a forced `Agg` backend); summary no longer crashes on runs shorter than the 200 s analysis window | all "Where" references, §8, appendix | — |
|  |  |  |  |

---

## Appendix — module index

Replaces the previous line-number index; module paths are stable in a way line numbers were not.

| Topic | Module |
|---|---|
| Entry point / CLI | [`scenarioADCSSoftwareRW.py`](scenarioADCSSoftwareRW.py), [`adcs_sim/scenario/cli.py`](adcs_sim/scenario/cli.py) |
| Tunable constants & placeholders | [`adcs_sim/config/`](adcs_sim/config/) (one module per concern) |
| Sensor/actuator interface to the DLL | [`adcs_sim/flightsoftware/structs.py`](adcs_sim/flightsoftware/structs.py) |
| Ground-target geometry | [`adcs_sim/geometry/ground_target.py`](adcs_sim/geometry/ground_target.py) |
| Pointing vectors, solar elevation | [`adcs_sim/geometry/pointing.py`](adcs_sim/geometry/pointing.py) |
| DLL struct mirrors (`ExtU_`, `ExtY_`, `B_`) | [`adcs_sim/flightsoftware/structs.py`](adcs_sim/flightsoftware/structs.py) |
| DLL handle & tuning entry points | [`adcs_sim/flightsoftware/library.py`](adcs_sim/flightsoftware/library.py) |
| Mode / saturation bookkeeping | [`adcs_sim/flightsoftware/diagnostics.py`](adcs_sim/flightsoftware/diagnostics.py) |
| Per-step I/O marshalling | [`adcs_sim/flightsoftware/controller.py`](adcs_sim/flightsoftware/controller.py) |
| Spacecraft hub + integrator | [`adcs_sim/models/hub.py`](adcs_sim/models/hub.py) |
| Gravity, SPICE, magnetic field | [`adcs_sim/models/environment.py`](adcs_sim/models/environment.py) |
| Reaction wheels | [`adcs_sim/models/reaction_wheels.py`](adcs_sim/models/reaction_wheels.py) |
| Magnetic torque rods | [`adcs_sim/models/magnetorquers.py`](adcs_sim/models/magnetorquers.py) |
| Thruster placeholder | [`adcs_sim/models/thrusters.py`](adcs_sim/models/thrusters.py) |
| Navigation | [`adcs_sim/models/navigation.py`](adcs_sim/models/navigation.py) |
| Orbit + initial attitude/rate | [`adcs_sim/models/initial_state.py`](adcs_sim/models/initial_state.py) |
| Model registration order + message wiring | [`adcs_sim/scenario/builder.py`](adcs_sim/scenario/builder.py) |
| Recorders | [`adcs_sim/scenario/recorders.py`](adcs_sim/scenario/recorders.py) |
| Vizard setup | [`adcs_sim/scenario/visualization.py`](adcs_sim/scenario/visualization.py) |
| Live keyboard loop | [`adcs_sim/scenario/live.py`](adcs_sim/scenario/live.py) |
| Run orchestration | [`adcs_sim/scenario/runner.py`](adcs_sim/scenario/runner.py) |
| Derived telemetry, plots, summary | [`adcs_sim/analysis/`](adcs_sim/analysis/) |
