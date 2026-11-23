# ADCS Software and Simulation

This repository contains the Simulink models and simulation tools used to
develop, test, generate, and validate the spacecraft Attitude Determination and
Control System (ADCS) software.

## Development workflow

```text
v1
 +-- Test ADCS control algorithms with the spacecraft plant model
        |
        v
FlightSW_testing
 +-- Generate C code from the software-only Simulink model
        |
        v
Basilisk
 +-- Run the generated ADCS software in a spacecraft simulation scenario
```

## Repository structure

### `v1/`

Contains both the spacecraft plant model and the ADCS software model. This
folder is intended for model-in-the-loop development and testing of the control
algorithms.

Notable files include:

- `ADCS_v1.slx` - integrated plant and ADCS Simulink model.
- `parameters.m` and `satellite_params.m` - model and spacecraft parameters.
- `run_ADCS_FSM_test_suite.m` - ADCS finite-state-machine test suite.
- Additional test models, test cases, and reports.

### `FlightSW_testing/`

Contains the software-only Simulink model used to test the flight-software
implementation and generate C code.

Notable files include:

- `ADCS_Software_v1.slx` - software-only ADCS model.
- `ADCS_Software_v1_grt_rtw/` - generated C source, headers, and build files.
- `ADCS_Software_v1.exe` - generated executable, when available.

### `Basilisk/`

Contains the Python scenario used to simulate the spacecraft with the generated
ADCS C code. This provides a higher-fidelity software-in-the-loop validation
environment.

Notable files include:

- `ADCS_software_test/simulator/scenarioADCSSoftwareRW.py` - primary Basilisk
  simulation scenario.
- `ADCS_software_test/` - simulator, connector, regression tests and test report.
- `ADCS_Software_v1_grt_rtw/` - generated ADCS code, wrapper and the DLL loaded
  by the Python scenario (built with `build_dll.ps1`).
- `ADCS_Simulator_standalone/` - older shareable snapshot of the simulator
  (pending merge, see `ORGANIZATION_NOTES.md`).
- `docs/` - integration guide, deployment guide and notes.
- `releases/` - zipped standalone bundles.

### `docs/`

Project-level documents: `design/` (block diagrams, FSM descriptions, developer
test cases) and `brochure/`.

See `ORGANIZATION_NOTES.md` for the folder clean-up log and open to-dos.

## Typical usage

1. Develop and verify the control algorithms using the integrated models in
   `v1/`.
2. Transfer the verified software changes to the software-only model in
   `FlightSW_testing/`.
3. Generate C code from `ADCS_Software_v1.slx` using Simulink Coder.
4. Build the generated code as the library or executable required by the
   Basilisk interface.
5. Place the generated artifacts in the expected location under `Basilisk/`
   and run the Basilisk scenario.

For example:

```powershell
cd Basilisk\ADCS_software_test\simulator
python scenarioADCSSoftwareRW.py
```

Use the scenario's built-in help to see the available run modes and options:

```powershell
python scenarioADCSSoftwareRW.py --help
```

## Requirements

- MATLAB and Simulink
- Simulink Coder for C-code generation
- A supported C compiler
- Python with Basilisk, NumPy, and Matplotlib
- Vizard (optional, for visualization)

Exact MATLAB, Basilisk, Python, and compiler versions should remain consistent
with those used to generate the checked-in model and build artifacts.
