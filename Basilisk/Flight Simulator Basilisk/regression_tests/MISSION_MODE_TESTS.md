# Mission-Mode Test Scenarios

Regression suite for the generated ADCS flight software, covering the mission
modes exercised by `test_mission_modes.py`.

```powershell
cd Basilisk\ADCS_software_test
python -u test_mission_modes.py                  # every case
python -u test_mission_modes.py --cases 1 4      # selected mission commands
python -u test_mission_modes.py --no-desat       # skip saturated-start cases
python -u test_mission_modes.py --verbose        # every check and metric
```

Exit status is non-zero if any case fails, so the suite can gate a build.
Use `-u`: without it Python buffers stdout, so a long run shows nothing until
it finishes and any Basilisk warnings (which go to stderr, unbuffered) appear
detached from the case that produced them.

---

## 1. What this suite is, and is not

Each case builds an independent Basilisk simulation, drives it with one mission
command, and evaluates quantitative criteria against recorded telemetry. It is
a **software-in-the-loop** test: the real compiled controller
(`ADCS_Software_v1.dll`) runs at the 0.05 s control rate against simulated
dynamics.

It runs **headless** - no Vizard, no sockets, no keyboard. It is deterministic
and safe to run in CI.

**What it does not cover:**

| Not covered | Why |
| --- | --- |
| Vizard rendering, PNG discovery/decode | Command 9 uses a synthetic camera feed (section 4) |
| Sensor noise, bias, misalignment | No sensor models exist yet - inventory `S2`-`S5` are all `[ ]`; the controller is fed truth |
| The flight toolchain | Host-compiled x86-64 GRT build with `MAT_FILE=1` instrumentation, not the target MCU |
| Disturbance torques | `E5`-`E9` all disabled |

A green suite means the control laws and the Python/C bridge behave correctly
against an idealised plant. It does not mean the imaging chain or the flight
build works.

---

## 2. Mission command mapping

Read out of the generated `ADCS_Software_v1.c` mission dispatch - **not** from
`v1/build_adcs_fsm.m`, which is stale (it still places `M_POINT_MOON` at command
6 and spins about LVLH).

| Cmd | Mode | Frame | Expected ACS state |
| --- | --- | --- | --- |
| 1 | `M_DETUMBLE` | 0 IDLE | 1 `ACS_DETUMBLE` |
| 4 | `M_POINT_NADIR` | 3 LVLH | 2 `ACS_POINT` |
| 5 | `M_POINT_SUN` | 2 INERTIAL | 2 `ACS_POINT` |
| 6 | `M_POINT_EARTH_TARGET` | 4 EARTH_TARGET | 2 `ACS_POINT` |
| 9 | `M_INTERSAT_TRACK` | 2 INERTIAL | 8 `ACS_OPTICAL` |

ACS states: `0 IDLE, 1 DETUMBLE, 2 POINT, 3 HOLD, 4 SLEW_SLOW, 5 SLEW_FAST,
6 SPIN, 7 DESAT, 8 OPTICAL, 9 SAFE`.

The ACS state is read from `B.actual_acs` after the step, not inferred from the
commanded mode - the manager can drop into `DETUMBLE`, `DESAT` or `SAFE`
underneath any mission command, and the tests rely on seeing that.

> **The reaction-wheel torque switch does not handle every ACS state.**
> `<S8>/Switch Case3` implements cases 2, 3, 4, 6, 7, 8, 9 and `default`.
> **Cases 1 (`DETUMBLE`) and 5 (`SLEW_FAST`) are absent**, so they fall through
> to the IDLE branch and `RW_tau` is zero. Detumble is performed entirely by
> the thrusters (`THR_tau = kd_detumble * omega`); the wheels take no part.

---

## 3. Case list

### 3.1 Nominal cases

| Case | Cmd | Duration | Initial conditions |
| --- | --- | --- | --- |
| detumble | 1 | 900 s | body rate `(0.03, -0.02, 0.04)` rad/s |
| nadir pointing | 4 | 900 s | nominal RAM attitude, wheels empty |
| sun pointing | 5 | 900 s | nominal |
| earth-target tracking | 6 | 900 s | nominal |
| intersat tracking (centred) | 9 | 600 s | RAM-aligned, target on boresight |
| intersat acquisition | 9 | 600 s | as above, rolled **8 deg** off boresight |

### 3.2 Saturated-start cases

Every nominal case is repeated with all three wheels at `CONTROLLER_H_MAX`
(50 N m s), above the `DESAT_ENTRY_FRACTION` threshold of 0.95. Duration is
extended by 600 s so desaturation has time to finish before the mode gets its
turn.

The assertion is **ordering**, not just occupancy:

1. `ACS_DESAT` is entered at all;
2. the first `DESAT` step precedes the first step in the commanded mode;
3. wheel momentum decreases over the run.

Pointing accuracy is deliberately *not* asserted on these cases - the run
spends its first phase shedding momentum, not pointing.

---

## 4. Command 9 and the synthetic camera

Optical tracking consumes rendered frames, which normally come from Vizard.
For a deterministic headless test, `SyntheticCameraFeed` duck-types
`CameraFeedReader` and renders the companion satellite analytically:

- line of sight from the true states of both spacecraft,
- pinhole projection at `f = (width/2) / tan(FOV/2)` = **703.4 px/rad** for the
  configured 40 deg / 512 px,
- a square blob written column-major, matching the model's `Camerafeed` layout.

That focal length is consistent with the regression measured from real Vizard
frames in `flightsoftware/structs.py` (705.8 px/rad in column, 665.0 in row).

> **Scope limit.** This exercises the flight software's centroid, the optical
> control law and the bridge that feeds them. It does **not** exercise Vizard's
> renderer or the PNG discovery/decode path in `flightsoftware/camera_feed.py`.
> Run `../simulator/scenarioADCSSoftwareRW.py --mode live` for those.

### Why the target starts centred

The scenario's initial attitude is RAM-aligned (+Z along velocity) and the
companion sits **+60 m along-track**, i.e. along +Z. With command 9 active from
t=0, the target projects at pixel (256.5, 256.5) - dead centre. This is the
intended setup: the optical law can only work if the target is already in
frame, since nothing in the flight software performs a search.

Because the nominal case starts on-target, its criterion is **hold**, not
converge. The separate 8 deg acquisition case is what actually exercises
convergence - and therefore the sign of the optical law.

### Known finding: one-step stale companion state

The companion spacecraft is registered **after** the controller in the task
(`builder.build_simulation` does this deliberately to preserve model order), so
reading its state message from controller context returns a position one
control step old. At 6.3 km/s that lag is **~316 m** - five times the 60 m
separation, and on its own enough to place the target behind the camera from
the second step onward.

`SyntheticCameraFeed.line_of_sight_B()` compensates by propagating the
companion forward one step with its own velocity; the residual from neglected
orbital acceleration over 0.05 s is about 5 mm.

**This is not only a test artifact.** Any onboard code that reads the companion
state message from the controller sees the same lag. It does not affect the
live pipeline, because Vizard renders from its own message set.

---

## 5. Pass criteria

Applied to every case:

| Check | Threshold |
| --- | --- |
| reaches the expected ACS state | > 10% of steps |
| body rate bounded | peak < 0.5 rad/s, all finite |

Mode-specific:

| Mode | Check | Threshold |
| --- | --- | --- |
| Detumble | body rate reduced | final < 20% of initial |
| Pointing (4, 5, 6) | pointing settled | std of boresight angle over final 10% < 2 deg |
| Nadir (4) | boresight acquired | final error < 5 deg |
| Sun / target (5, 6) | boresight acquired | final error < 15 deg |
| Optical (9) | target held near centre | tail-mean radius < 25 px (2.0 deg) |
| Optical (9) | centroid bounded | peak radius < 150 px |
| Optical (9) | target never lost | visible on every step |
| Optical (9, offset) | acquisition | tail-mean < 50% of head-mean |
| Saturated start | DESAT entered, and precedes the mode | ordering of first occurrence |
| Saturated start | momentum shed | final < initial |

Nadir alignment is asserted rather than merely reported: the nominal run
settles at 0.06 deg with **+Z** the best-aligned axis, which confirms the
convention independently. `plots.py` documents sun and earth-target as aligning
+Z, and those thresholds are looser because the target moves.

---

## 6. Results as of 2026-08-06

Run against `ADCS_Software_v1.dll` built 14:12 from model 2.43.

### Nominal cases - 6/6 pass

| Case | Key metric |
| --- | --- |
| cmd1 detumble | rate decays, time constant ~190 s |
| cmd4 nadir | boresight 90.57 -> **0.059 deg**, best axis **+Z**, tail std 0.0001 deg |
| cmd5 sun | settled |
| cmd6 earth-target | settled |
| cmd9 intersat (centred) | centroid 3.6 -> 3.8 px, peak 4.6 px, never lost |
| cmd9 intersat (8 deg offset) | centroid **28.5 -> 3.8 px**, peak 99.4 px |

The 8 deg acquisition case is the one that validates the optical sign fix - an
inverted gain would grow the offset instead of pulling it in. Both cmd9 cases
converge to the same 3.8 px steady state, which is PD following error against a
moving target, not a sign fault.

### Saturated-start cases - FAILING

> **Desaturation does not work.** With all three wheels at `CONTROLLER_H_MAX`,
> the controller enters `ACS_DESAT` and **never leaves it**:
>
> | Metric | Result over 1500 s (cmd 4) |
> | --- | --- |
> | ACS occupancy | `DESAT` 100% of steps |
> | Wheel momentum | 50.0 -> **50.0008** N m s, nothing shed |
> | Commanded mode | never reached |
> | Body rate | drifts up to 0.052 rad/s |
> | Boresight | wanders to 139.5 deg |
>
> This is not a test artifact. The scenario is *designed* to start saturated
> ("the reaction wheels start at their momentum limit by default, so a run
> begins in desaturation" - `scenarioADCSSoftwareRW.py`), so this path is
> supposed to work.

**Why it cannot work as built.** Desaturation requires two things together: an
external torque to absorb momentum, *and* a wheel command to spin the wheels
down. Only the first exists:

- `THR_tau` in `ACS_DESAT` is roughly `-RW_omega` (~628 N m at rated speed),
  clipped to +/-0.05 N m by `thrusters.limit_body_torque`;
- `RW_tau` in `ACS_DESAT` is `kd_hold * omega_err` - rate damping, **not** a
  spin-down command.

So the thruster torques the *body* while nothing unloads the *wheels*, and wheel
momentum is conserved. The controller then latches in `DESAT` because `|h|`
never drops below the `DESAT_EXIT_FRACTION` band (12.5 N m s).

Compounding it, inventory item **A3** records that there is no thruster model at
all - `ExtForceTorque` applies the commanded body torque directly, with no
force, no moment arm and no firing logic. The 0.05 N m clip exists only to stop
the raw DESAT output destabilising the integrator.

**This needs a flight-software fix, not a test change.** Until then the
saturated-start cases are expected to fail and are the suite's most valuable
output.

Saturated variants of commands 1, 5, 6 and 9 have not been run to completion;
cmd 4 above is representative, since the desat law is mode-independent.

---

## 7. Interpreting a failure

| Symptom | Likely cause |
| --- | --- |
| Every case fails at "reaches ACS ..." | `ADCS_Software_v1.dll` is stale - check it is newer than `ADCS_Software_v1.c` |
| Optical centroid grows instead of shrinking | sign error in `<S18>` - check `Gain3`/`Gain4` and the `kd_point` branch of `Sum1` |
| Optical case: "target NOT in frame at t=0" | initial attitude is not RAM-aligned, or the companion is disabled in `config/companion.py` |
| Body rate diverges | damping entering with the wrong sign (positive rate feedback) |
| Saturated case: "DESAT precedes ..." fails | see section 6 - currently expected |

**Before investigating any failure, confirm the DLL is current.** Simulink's own
build produces `ADCS_Software_v1.exe`, not the DLL the scenario loads
(`PRODUCT_TYPE = "executable"` in `ADCS_Software_v1.mk`). Rebuild with:

```powershell
cd Basilisk\ADCS_Software_v1_grt_rtw
powershell -NoProfile -ExecutionPolicy Bypass -File .\build_dll.ps1 -NoSync `
  -MingwRoot "C:\ProgramData\MATLAB\SupportPackages\R2026a\3P.instrset\mingw_w64.instrset\bin"
```

Keep `-NoSync`: without it the script overwrites the local generated sources
from `FlightSW_testing\ADCS_Software_v1_grt_rtw`.

---

## 8. Extending the suite

Add a `MissionCase` to `build_cases()`. The fields that matter:

| Field | Purpose |
| --- | --- |
| `mission_cmd`, `expect_acs` | what to command and what ACS state to require |
| `seconds` | run length; must exceed the mode's settling time |
| `initial_rate_rad_s` | body rate at t=0 |
| `initial_momentum_nms` | per-wheel momentum at t=0; >= 47.5 triggers the desat checks automatically |
| `pointing_target` | `"nadir"`, `"sun"` or `"target"` for boresight metrics |
| `assert_pointing`, `max_pointing_error_deg` | promote the boresight angle from reported to pass/fail |
| `needs_camera` | attach the synthetic feed |
| `initial_attitude_offset_deg`, `offset_axis` | start off-boresight; keep inside the 20 deg half-FOV |
| `expect_acquisition` | require the centroid to halve |

Modes not yet covered: 0 (`M_IDLE`), 2 (`M_SAFE_SUN`), 3 (`M_Y_SPIN`),
7/8 (slews), 10 (`M_TDI`), 11 (`M_MINMAX_DRAG`).

> `M_Y_SPIN` is currently untestable as configured: `kd_spin = 0.0` kills the
> entire rate term, so the spin axis receives no command at all. Set
> `KD_SPIN` to about 5 in `config/gains.py` before adding a case for it.
