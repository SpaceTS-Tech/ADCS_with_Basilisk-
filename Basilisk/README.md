# Basilisk ADCS Software-in-the-Loop Simulation

## Purpose

This folder integrates the C code generated from the software-only Simulink
model with a Basilisk spacecraft simulation. It is the software-in-the-loop
stage of the repository workflow:

```text
../v1/                       Integrated plant and controller testing
        |
        v
../FlightSW_testing/         Software-only model and C-code generation
        |
        v
Basilisk/                    Generated controller in a Basilisk scenario
```

The primary scenario models the spacecraft, orbit, navigation data, magnetic
field, reaction wheels, magnetorquers, and external torque commands. A Python
adapter calls the generated ADCS shared library at the controller sample rate.

## Main files and directories

| Path | Purpose |
| --- | --- |
| `ADCS_software_test/` | Everything that exercises the controller, one folder per job — see its [README](ADCS_software_test/README.md). |
| `ADCS_software_test/simulator/scenarioADCSSoftwareRW.py` | Primary runnable Basilisk scenario. |
| `ADCS_software_test/regression_tests/` | Headless pass/fail checks over every mission mode. |
| `ADCS_software_test/test_report/` | Formal test-case evidence and the report document. |
| `ADCS_Software_v1_grt_rtw/` | Active generated controller source, wrapper (`adcs_basilisk_wrapper.c`), DLL, import library, and build artifacts. Earlier known-working builds are in git history. |
| `ADCS_Simulator_standalone/` | Older (2026-10-22) shareable snapshot of the simulator with a bundled flight-software package. |
| `ADCS_software_test/simulator/output/*.png` | Plots produced by file-mode simulations. |
| `ADCS_software_test/simulator/output/test2.bin` | Vizard playback file produced by file mode. |
| `docs/` | Integration guide, target deployment guide, development notes (`Notes.docx`) and the correction tracker. |
| `releases/` | Zipped standalone simulator and flight-software bundles. |
| `3d models/` | Fallback copy of the spacecraft model (`m4v.glb`). |

The active scenario expects the shared library at:

```text
Basilisk/ADCS_Software_v1_grt_rtw/ADCS_Software_v1.dll
```

## Requirements

- A supported Python installation
- Basilisk Python package
- NumPy
- Matplotlib
- A Windows DLL compatible with the Python process architecture
- Vizard, if visualization or live interaction is required

The scenario imports Basilisk simulation, architecture, and utility modules.
The Python interpreter, Basilisk installation, and generated DLL must all use
compatible architectures, normally 64-bit.

Verify the Python dependencies:

```powershell
python -c "import Basilisk, numpy, matplotlib; print('Dependencies available')"
```

## Running the scenario

From the scenario directory:

```powershell
cd ADCS_software_test\simulator
python scenarioADCSSoftwareRW.py
```

Or, without a terminal, double-click
`ADCS_software_test\simulator\Run_Simulator.exe`.

The default is file mode. The current script:

- Starts with mission command `4`
- Runs for `5000` simulated seconds
- Calls the generated controller every `0.05` seconds
- Initializes each reaction wheel at the configured momentum limit unless
  overridden
- Writes `test2.bin` for Vizard playback
- Writes diagnostic plots into the scenario directory

Display all command-line options:

```powershell
python scenarioADCSSoftwareRW.py --help
```

## File mode

Run a finite simulation and generate output files:

```powershell
python scenarioADCSSoftwareRW.py --mode file
```

To display plots after the simulation in addition to saving them:

```powershell
python scenarioADCSSoftwareRW.py --mode file --show-plots
```

The generated plot files are:

- `quaternion.png`
- `angular_rate.png`
- `rw_angular_velocity.png`
- `mtq_voltage.png`
- `adcs_torques.png`
- `rw_tau_after_200s.png`
- `earth_pointing.png`
- `sun_pointing.png`
- `earth_target_pointing.png`
- `target_sunlight.png`
- `adcs_modes.png`

The console also reports generated-controller state counts and the peak
reaction-wheel torque observed at the controller update rate.

## Reaction-wheel initial conditions

Specify the three initial wheel speeds in RPM:

```powershell
python scenarioADCSSoftwareRW.py --rw-omega-rpm 0 0 0
```

Or specify the three initial wheel momenta in N m s:

```powershell
python scenarioADCSSoftwareRW.py --rw-momentum 10 10 10
```

These options are mutually exclusive. If neither is supplied, the current
scenario initializes each wheel at `50 N m s`, the configured physical
momentum limit.

## Live Vizard mode

Start an indefinite two-way Vizard stream:

```powershell
python scenarioADCSSoftwareRW.py --mode live
```

The current configuration connects to Vizard over TCP at
`127.0.0.1:5556`. Start and configure Vizard so that the socket connection is
available.

Live keyboard controls are:

| Key | Action |
| --- | --- |
| `0` through `9` | Select mission commands 0 through 9. |
| `A` | Select mission command 10. |
| `B` | Select mission command 11. |
| `P` | Pause or resume the simulation. |
| `Q` | Stop the live simulation. |

## Generated-controller integration

`scenarioADCSSoftwareRW.py` loads the controller with Python `ctypes` and
implements a `GeneratedADCSController` Basilisk model. At each controller
update, the adapter:

1. Reads spacecraft, environment, and actuator state messages.
2. Converts values into the frames, signs, units, and C layouts expected by the
   generated model.
3. Populates the generated external-input structure.
4. Calls `ADCS_Software_v1_step()`.
5. Converts controller outputs into Basilisk actuator messages.
6. Records mode and torque histories for analysis.

The integration relies on the layout declared in
`ADCS_Software_v1_grt_rtw/ADCS_Software_v1.h`. Regenerate and review the Python
`ctypes` declarations whenever the Simulink root interface or generated data
structures change.

## C wrapper

`ADCS_Software_v1_grt_rtw/adcs_basilisk_wrapper.c` exposes small C ABI helper functions for runtime
parameters and internal status needed by the Python scenario. The current
helpers:

- Set the mission command
- Set reaction-wheel inertia
- Set maximum wheel momentum
- Read the configured momentum limit
- Read wheel-saturation status
- Read desaturation-complete status
- Set fine-ADS availability

This file is integration source code, not Simulink-generated code. Preserve it
when replacing the generated controller directory and compile it into
`ADCS_Software_v1.dll`.

## Updating the controller DLL

When new C code is generated in `../FlightSW_testing/`:

1. Back up or retain the last known-working generated build.
2. Synchronize the new generated `.c`, `.h`, metadata, and build files into
   `ADCS_Software_v1_grt_rtw/`.
3. Add or preserve `adcs_basilisk_wrapper.c`.
4. Confirm that Python `ctypes` structures match the newly generated header.
5. Rebuild `ADCS_Software_v1.dll` and `ADCS_Software_v1.lib`.
6. Run a short smoke test before the full 5000-second scenario.
7. Compare modes, actuator commands, pointing errors, and wheel behavior with
   the known-working build.

Do not replace the active DLL without keeping a reproducible path back to the
previous validated build.

## Important configuration constants

The beginning of `scenarioADCSSoftwareRW.py` contains the main integration and
scenario constants, including:

- Mission command and simulation duration
- Controller and plotting sample times
- Reaction-wheel inertia, momentum limit, and torque limit
- Thruster body-torque limit
- Magnetorquer voltage-to-dipole conversion
- Ground-target coordinates
- SPICE epoch
- Quaternion, angular-rate, and torque sign conventions
- Vizard socket address, port, and acceleration factor

Several comments identify temporary or placeholder flight-hardware values.
Replace those values only from controlled spacecraft data and rerun regression
tests after any change.

## Troubleshooting

### DLL cannot be loaded

- Confirm `ADCS_Software_v1.dll` exists in the active generated-code directory.
- Confirm Python and the DLL use the same 32/64-bit architecture.
- Confirm dependent compiler runtime libraries are available.
- Confirm the path in `ADCS_LIBRARY` has not changed.

### Missing symbol or `ctypes` error

- Compare the exported DLL functions with the generated header and wrapper.
- Recompile the wrapper into the DLL.
- Update Python structure definitions after any generated-interface change.

### Unexpected attitude or actuator signs

- Check the quaternion convention and `CONJUGATE_Q_BI`.
- Check `ANGULAR_RATE_SIGN` and `RW_MOTOR_TORQUE_SIGN`.
- Check inertial/body frame conventions at every adapter boundary.
- Verify reaction-wheel axis definitions and torque mapping.

### Live mode does not connect

- Start Vizard before the scenario if required by the local setup.
- Confirm the TCP endpoint is `127.0.0.1:5556`.
- Check that the port is not blocked or already in use.

### Simulation is slow

The default run is 5000 simulated seconds with a 0.05-second controller step.
For development, temporarily reduce `SIMULATION_SECONDS`, validate the
integration, and then restore the full regression duration.
