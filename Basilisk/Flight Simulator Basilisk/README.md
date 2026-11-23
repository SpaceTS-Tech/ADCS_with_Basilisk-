# ADCS software test

Everything that exercises the generated `ADCS_Software_v1` controller, split
into one folder per job. Each folder has its own `README.md` naming the single
file you run, and a `.exe` that runs it for you.

## The four folders

| Folder | What it is | Run |
| --- | --- | --- |
| [`simulator/`](simulator/) | The Basilisk simulation itself — spacecraft, orbit, sensors, actuators, and the `adcs_sim` package behind them. | `Run_Simulator.exe` |
| [`regression_tests/`](regression_tests/) | Headless pass/fail checks over every mission mode. No Vizard, deterministic, safe for CI. | `Run_Regression_Tests.exe` |
| [`test_report/`](test_report/) | Formal test-case evidence: runs the declared cases and fills the results into a report document. | `Run_Test_Cases.exe` |
| [`legacy/`](legacy/) | The pre-refactor single-file scenario, kept for reference. Superseded by `simulator/`. | `Run_Legacy_Scenario.exe` |

[`tools/launcher/`](tools/launcher/) holds the source of those four `.exe`
files and the script that rebuilds them — see [below](#the-exe-launchers).

## No terminal? Just double-click

Every folder contains one `.exe`. Double-clicking it opens a console window,
finds Python, runs that folder's entry point, and **keeps the window open** at
the end so you can read the result. Nothing to type, nothing to install beyond
Python itself.

Start here:

| I want to… | Double-click |
| --- | --- |
| Watch the spacecraft fly | `simulator\Run_Simulator.exe` |
| Check the controller still passes | `regression_tests\Run_Regression_Tests.exe` |
| Produce the test-case report | `test_report\Run_Test_Cases.exe` |

`Run_Simulator.exe` first asks how you want to run — **[1]** file mode (writes
plots and a Vizard recording), **[2]** live mode (interactive 3-D in Vizard),
**[3]** a short file-mode run, **[4]** live mode with no flight software, or **[5]** arguments you type yourself. Press
Enter to take the default. The other three launchers have only one way to run,
so they start immediately.

## Prerequisites

- Python 3 with **Basilisk**, **NumPy** and **Matplotlib** installed
- The generated controller built at
  `Basilisk\ADCS_Software_v1_grt_rtw\ADCS_Software_v1.dll` — optional for
  the simulator: without it the simulator runs with **no flight software**
  (no control torque or force; it warns you). Ask for that explicitly with
  `--no-flight-software`, `ADCS_LIBRARY=none`, or menu choice [4] of
  `Run_Simulator.exe`. Regression tests and the test report still need it.
- Vizard, only for the simulator's live 3-D visualization

Check the Python side in one line:

```powershell
python -c "import Basilisk, numpy, matplotlib; print('Dependencies available')"
```

If a launcher reports that it cannot find Python, either add `python.exe` to
`PATH` or point the `ADCS_PYTHON` environment variable at it:

```powershell
setx ADCS_PYTHON "C:\Program Files\Python312\python.exe"
```

## Running from a terminal instead

The `.exe` files are a convenience, not a requirement — the scripts are ordinary
Python and take the same arguments either way. Both of these are equivalent:

```powershell
cd simulator
python scenarioADCSSoftwareRW.py --seconds 500
.\Run_Simulator.exe --seconds 500
```

Arguments passed to a launcher go straight through to the script, so anything
documented in a folder's README works from the `.exe` too. Passing arguments
also **skips the start-up menu** — a launcher only asks when it is given
nothing, and never asks when its input is piped or redirected, so it stays safe
to call from a script or a scheduled task.

## The .exe launchers

They are thin wrappers — roughly "ask if there is anything to ask, find Python,
run the script next to me, wait, pause" — compiled from
[`tools/launcher/Launcher.cs`](tools/launcher/Launcher.cs). They contain no
simulation logic, so the Python scripts remain the only thing you ever need to
read or change.

Each launcher's script, window title and menu are declared in one table at the
top of [`build_launchers.ps1`](tools/launcher/build_launchers.ps1). To give
another folder a menu, or to add a choice to the simulator's, edit that table —
an entry is `label|description|arguments`, and arguments of `?` prompt the user
to type their own.

Rebuild them after renaming an entry point, changing a menu, or editing the
launcher source:

```powershell
powershell -ExecutionPolicy Bypass -File tools\launcher\build_launchers.ps1
```

The build uses the C# compiler included with the .NET Framework, so there is
nothing to install.
