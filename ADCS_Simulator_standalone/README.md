# Simulator

The Basilisk simulation of the spacecraft flying under the generated
`ADCS_Software_v1` controller: orbit, attitude dynamics, navigation, magnetic
field, reaction wheels, magnetorquers and thrusters, with the real flight DLL
stepped in the loop.

## Run this

**File to run: [`scenarioADCSSoftwareRW.py`](scenarioADCSSoftwareRW.py)**

No terminal? **Double-click `Run_Simulator.exe`.** It asks how you want to run
before starting anything:

```text
  How do you want to run it?

    [1]  File mode  (default)
         runs for a fixed time, then writes the plots and the Vizard recording into output\

    [2]  Live mode
         streams to Vizard for interactive 3-D; the keyboard sets the mission command

    [3]  File mode, short run
         the same as [1] but 500 s instead of 5000 s, for a quick check

    [4]  Custom
         type your own arguments

    [q]  quit without running anything

  Choice [1]:
```

Press **2** for live mode, or just Enter for the default. Choice `4` lets you
type any of the arguments below without needing a terminal at all.

From a terminal:

```powershell
python scenarioADCSSoftwareRW.py
```

Either way the default is a 5000-second run that writes its figures and a
Vizard recording into [`output/`](output/).

## What else is in here

| Path | Purpose |
| --- | --- |
| [`scenarioADCSSoftwareRW.py`](scenarioADCSSoftwareRW.py) | **The entry point.** A thin wrapper — the real code is in `adcs_sim/`. |
| [`adcs_sim/`](adcs_sim/) | The simulation package. Start at [`adcs_sim/README.md`](adcs_sim/README.md) for the module map. |
| [`ADCS_SIM_MODEL_INVENTORY.md`](ADCS_SIM_MODEL_INVENTORY.md) | How faithful each sensor, actuator and environment model is. **Read this before trusting a result.** |
| [`ADCS_SIM_DATA_INDEX.md`](ADCS_SIM_DATA_INDEX.md) | Every constant and the file it lives in — the place to look when changing a number. |
| [`output/`](output/) | Figures and `test2.bin` from the last file-mode run. Overwritten each time. |
| `Run_Simulator.exe` | Double-click launcher for the entry point. |

## The two modes

### `--mode file` (default)

Runs for a fixed duration with no visualization, then writes 11 figures and a
Vizard recording (`test2.bin`) into `output/`. This is the mode to use for
analysis and for anything you want to keep.

```powershell
python scenarioADCSSoftwareRW.py                      # 5000 s
python scenarioADCSSoftwareRW.py --seconds 500        # shorter run
python scenarioADCSSoftwareRW.py --show-plots         # also display them
python scenarioADCSSoftwareRW.py --output-dir runs\a  # write somewhere else
```

### `--mode live`

Streams indefinitely to Vizard over a socket for interactive 3-D viewing, with
a strip chart of ACS mode and wheel momentum alongside. Vizard is launched
automatically unless you say otherwise.

Pick **[2]** in the launcher menu, or from a terminal:

```powershell
python scenarioADCSSoftwareRW.py --mode live
python scenarioADCSSoftwareRW.py --mode live --no-vizard    # start Vizard yourself
python scenarioADCSSoftwareRW.py --mode live --vizard-exe "D:\Vizard\Vizard.exe"
```

Keyboard control in the console window while live:

| Key | Effect |
| --- | --- |
| `0`–`9`, `a`, `b` | Set the mission command (`a` = 10, `b` = 11) |
| `p` | Pause / resume |
| `q` | Quit |

This works through `Run_Simulator.exe` too — the launcher hands its console
straight to the simulation, so keystrokes reach it normally.

## Initial wheel state

The wheels **start at their momentum limit by default**, so a run opens in
desaturation before it does anything else. That is deliberate: it exercises the
hardest case. To start elsewhere:

```powershell
python scenarioADCSSoftwareRW.py --rw-momentum 0 0 0        # unsaturated, N m s
python scenarioADCSSoftwareRW.py --rw-omega-rpm 0 0 0       # or by wheel speed
```

## Requirements

Python 3 with Basilisk, NumPy and Matplotlib, plus the controller DLL built at
`Basilisk\ADCS_Software_v1_grt_rtw\ADCS_Software_v1.dll`. The scenario locates
that DLL by searching upward from this folder, so the tree can be moved without
editing paths. Vizard is needed only for `--mode live`.
