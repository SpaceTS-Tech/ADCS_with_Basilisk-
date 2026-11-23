# Regression tests

Headless pass/fail checks on the generated controller. Every mission mode runs
as an independent simulation and is then measured against quantitative
criteria. Nothing here touches Vizard, so the suite is deterministic and safe
to run unattended or in CI — the exit status is non-zero if any case fails.

## Run this

**File to run: [`test_mission_modes.py`](test_mission_modes.py)**

No terminal? **Double-click `Run_Regression_Tests.exe`** — it runs the whole
suite and holds the window open on the summary.

From a terminal:

```powershell
python -u test_mission_modes.py                  # every case
python -u test_mission_modes.py --cases 1 4      # selected mission commands
python -u test_mission_modes.py --no-desat       # skip the saturated-start set
python -u test_mission_modes.py --verbose        # every check and metric
```

The full suite runs many simulations back to back, so expect it to take a
while. `--cases` is the quick way to re-check one mode after a change.

## What is in here

| Path | Purpose |
| --- | --- |
| [`test_mission_modes.py`](test_mission_modes.py) | **The entry point.** The scenarios and their pass criteria. |
| [`MISSION_MODE_TESTS.md`](MISSION_MODE_TESTS.md) | What each mode is expected to do and why each threshold is what it is. |
| `Run_Regression_Tests.exe` | Double-click launcher for the entry point. |

## What gets checked

| Cmd | Mode | Expected ACS state |
| --- | --- | --- |
| 1 | `M_DETUMBLE` | 1 `ACS_DETUMBLE` |
| 4 | `M_POINT_NADIR` | 2 `ACS_POINT` (frame LVLH) |
| 5 | `M_POINT_SUN` | 2 `ACS_POINT` (frame INERTIAL) |
| 6 | `M_POINT_EARTH_TARGET` | 2 `ACS_POINT` (frame EARTH_TARGET) |
| 9 | `M_INTERSAT_TRACK` | 8 `ACS_OPTICAL` (frame INERTIAL) |

Each mode is run twice: once from rest, and once with the wheels started at
their momentum limit. The saturated case asserts an **ordering** — the
controller must enter `ACS_DESAT` first, shed momentum, and only then settle
into the commanded mode.

> **Note on command 9.** Optical tracking normally consumes frames rendered by
> Vizard. To keep this suite headless, a synthetic camera feed substitutes an
> analytically projected blob at the companion satellite's true line of sight.
> That exercises the flight software's centroid and control law, but **not**
> Vizard's renderer or the PNG decode path. A green command-9 case here does
> not prove the real imaging chain works — run
> [`../simulator/scenarioADCSSoftwareRW.py`](../simulator/scenarioADCSSoftwareRW.py)
> `--mode live` for that.

## Relationship to the other folders

This suite imports the simulation package from
[`../simulator/`](../simulator/) — it tests that simulator, it does not carry a
copy of it. [`../test_report/`](../test_report/) drives these same scenarios
again to produce formal evidence; use that when you need a document, and this
when you just need to know whether anything broke.
