# Organization notes

Clean-up done on 2026-11-20. Deleted items were sent to the Windows Recycle
Bin; most were also tracked in git and can be restored with
`git checkout HEAD -- <path>` until that deletion is committed.

## Current layout

```text
ADCS_Software_Simulink/
├── README.md, ORGANIZATION_NOTES.md, .gitignore
├── docs/
│   ├── design/      block diagrams, FSM docs (AOCS_FSM, ADCS_FSM_old), developer test cases
│   └── brochure/    ADCS software brochure (docx/png/svg)
├── v1/              integrated plant + ADCS model (model-in-the-loop)
├── FlightSW_testing/ software-only model ADCS_Software_v1.slx + code generation
└── Basilisk/
    ├── ADCS_Software_v1_grt_rtw/   generated code + wrapper + DLL (loaded by the simulator)
    ├── ADCS_software_test/         simulator, connector, regression tests, test report
    ├── ADCS_Simulator_standalone/  older shareable simulator snapshot (see to-do)
    ├── docs/                       integration guide, deployment guide, Notes, tracker
    ├── releases/                   ADCS_Simulator_standalone.zip, ADCS_Software_v1_standalone.zip
    └── 3d models/                  fallback m4v.glb
```

## What the folders actually are (verified)

| Folder | Finding |
| --- | --- |
| `v1/` | **Not** a copy of FlightSW_testing. It holds the integrated plant + ADCS model (`ADCS_v1.slx`), FSM/test harnesses and the ADS sub-project. No model references to FlightSW_testing. Its old `ADCS_Software_v1_grt_rtw/` (model v2.75, Sep 14) was stale and has been removed. |
| `FlightSW_testing/` | Source of truth for the flight software: `ADCS_Software_v1.slx` (model v2.178) and its codegen folder. `build_dll.ps1` syncs sources from here. |
| `Basilisk/ADCS_Software_v1_grt_rtw/` | **Not just a copy**: the `.c/.h` match FlightSW_testing, but this folder also holds `adcs_basilisk_wrapper.c`, `build_dll.ps1` and the DLL the simulator loads. `adcs_sim/config/paths.py` finds the repo root by searching upward for this folder name, so don't rename or move it. |
| `Basilisk/ADCS_Software_v1_grt_rtw_working/` | Was byte-identical to the live folder → deleted (git history is the backup). |
| `Basilisk/ADCS_software_test/simulator/` | Newest simulator (2026-11-20): adds a no-flight-software mode (`NullADCSLibrary`), a Vizard app launcher, `simulator_ports.json`, and the star-catalog tools. |
| `Basilisk/ADCS_Simulator_standalone/` | Older snapshot (2026-10-22) of the same simulator plus a bundled `flight_software/` package (Makefile, src, prebuilt DLL), `requirements.txt` and `START_HERE.md`. No connector. |

## Removed (~137 MB, 115 items)

- OneDrive conflict copies `*-Swapnil*` / `*-Swapnil-2*` (old Aug-10 / Sep-30 code generations).
- Old binaries: `*.dll.bak-*`, `*.lib.bak-*`, `*.locked-*.dll`, `ADCS_Software_v11.dll`; the stale FlightSW_testing DLL/lib (Sep 30, older than the Oct 7 codegen).
- `Basilisk/ADCS_Software_v1_grt_rtw_working/`. It also held a generated `ADCS_FSM_v1_grt_rtw/`, which can be regenerated from `v1/ADCS_FSM_v1.slx`.
- `v1/ADCS_Software_v1_grt_rtw/`, `v1/ADCS_Software_v1.exe`, `v1/ADCS_FSM_v1_grt_rtw/`: stale generated code.
- Old model snapshots: `ADCS_Software_v1.slx.original`, `*.slx.r2025b`, `ADCS_Software_v1_23a.slx` + `_23a_saveas.m`.
- Caches: all `slprj/`, `*.slxc`, `__pycache__/`, `parameters.asv`, empty `connector.log`.
- `Basilisk/test1.bin` (July Vizard recording), stale `Basilisk/adcs_basilisk_wrapper.c` (Aug copy; the live one is in `ADCS_Software_v1_grt_rtw/`).
- Empty `.git`/`.agents` folders in `Basilisk/` and `v1/`.

`.gitignore` now excludes caches and these leftover patterns so they don't come back.

## To-do (not done yet)

1. **Merge the standalone simulator into ADCS_software_test.** `ADCS_software_test/simulator` is the newer code. To make it self-contained:
   - bring over `flight_software/` (or build the DLL into `simulator/flight_software/build/`). `paths.py` already switches to bundle mode when that folder exists.
   - copy `requirements.txt` and `START_HERE.md`.
   - then delete `ADCS_Simulator_standalone/` and regenerate `releases/ADCS_Simulator_standalone.zip`.
2. **Rename `ADCS_software_test`** (e.g. `ADCS_Simulator`) only after:
   - updating `simulator/simulator_ports.json`, which contains absolute paths with the old name.
   - rebuilding the `Run_*.exe` launchers.
   - updating the READMEs and `MISSION_MODE_TESTS.md`.
3. **External dependency:** `build_dll.ps1` defaults `StarTrackerDir` to `..\..\..\Star Trcaker\startracker_c\matlab`, which is outside this repo (note the "Trcaker" typo). Vendor it in, or document it.
4. `paths.py` references `Basilisk/ADCS_Software_v1_standalone/`, which only exists as a zip in `releases/`.
5. `FlightSW_testing/README.md` still quotes model version 1.129 / R2025b. The current code is 2.178.
6. `FlightSW_testing/catalogue.mat` differs from `startracker_m/catalogue.mat`. Confirm which one the model loads and drop the other.
7. `v1/test.m` is a thruster-configuration analysis script, and `v1/misc/test.m` is an FSM test runner. Give both descriptive names.
8. Git: the working tree already had uncommitted deletions of `v2/` and `Eclipse_Calculator/` from before this clean-up. Review them, then commit everything (slprj caches will show as deleted).
