# Legacy

The original single-file scenario, from before the simulation was split into
the `adcs_sim` package. Kept for reference and comparison.

> **Use [`../simulator/`](../simulator/) instead.** This folder is superseded.
> It is not maintained, and fixes made to the simulator do not reach it.

## Run this

**File to run: [`scenarioADCSSoftwareRW_working.py`](scenarioADCSSoftwareRW_working.py)**

No terminal? **Double-click `Run_Legacy_Scenario.exe`**.

From a terminal:

```powershell
python scenarioADCSSoftwareRW_working.py
```

It is self-contained — it does not import `adcs_sim` — and writes its figures
and `test2.bin` next to itself, in this folder, rather than into the
simulator's `output/`.

## What is in here

| Path | Purpose |
| --- | --- |
| [`scenarioADCSSoftwareRW_working.py`](scenarioADCSSoftwareRW_working.py) | The pre-refactor monolithic scenario. |
| [`Changes.txt`](Changes.txt) | Notes on the interface-level edits made to the generated `ADCS_Software_v1` model, and the Vizard socket address. |
| `Run_Legacy_Scenario.exe` | Double-click launcher for the entry point. |

## Why it is still here

The split into `adcs_sim/` was verified as numerically identical to this file —
a byte-identical Vizard recording over a 250 s run. Keeping the original makes
that claim re-checkable, and gives you something to fall back on if the
refactored scenario is ever suspected of introducing a difference.

`Changes.txt` is worth reading on its own: it records what was changed in the
generated model itself (vector-valued Sun state, two gain sign flips, the added
wrapper functions), which is not visible from the Python side at all.
