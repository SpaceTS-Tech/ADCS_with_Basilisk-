# ADCS Simulator: standalone bundle

Basilisk closed-loop simulator for the ADCS flight software (model 2.178).
This folder is self-contained: copy it anywhere and run it.

## Contents

| Path | What it is |
| --- | --- |
| `scenarioADCSSoftwareRW.py` | Entry point |
| `Run_Simulator.exe` | Menu launcher for the entry point (finds Python itself) |
| `adcs_sim/` | Simulator package (see `README.md`) |
| `flight_software/` | Flight-code package: the same C sources that go on the processor |
| `flight_software/build/ADCS_Software_v1.dll` | Prebuilt controller the simulator loads |
| `flight_software/optional/adcs_basilisk_wrapper.c` | Wrapper that turns the flight code into the simulator DLL |
| `assets/m4v.glb` | Spacecraft 3-D model for Vizard |
| `tools/star_tracker_render_check.py` | Star tracker image check |
| `output/` | Plots and Vizard recordings are written here |

## Setup (Windows 10/11, 64-bit)

1. Install Python 3.12 (64-bit).
2. `pip install -r requirements.txt`
3. Optional, for 3-D playback: install Vizard (Basilisk's visualiser) and set
   `VIZARD_EXECUTABLE` to its `Vizard.exe`, or pass `--vizard-exe`.

## Run

```
python scenarioADCSSoftwareRW.py --seconds 300                       # file mode, plots in output/
python scenarioADCSSoftwareRW.py --seconds 300 --rw-momentum 4 4 4   # start with saturated wheels
python scenarioADCSSoftwareRW.py --mode live                         # stream to Vizard
```

## Rebuilding the controller DLL

Only needed after the flight code changes.  With MinGW-w64 gcc on PATH:

```
cd flight_software
mingw32-make dll
```

To fly a different DLL without copying it here, set `ADCS_LIBRARY` to its full path.

The integration (wrapper, data exchange, gains) is explained in the
"ADCS Simulator and Flight Software Integration" document.
