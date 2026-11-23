# ADS — Attitude Determination System (`ADS_v1.slx`)

Simulink implementation of design document *ADCS-ADS-DD-001*
(`ADS_Mode_Architecture_and_Estimation_Algorithms.docx`).

## ADS modes

| ID | Mode | Estimator |
| --- | --- | --- |
| 0 | `ADS_IDLE` | none |
| 1 | `ADS_RATE` | gyro rate or MAG-rate Kalman filter |
| 2 | `ADS_COARSE` | USQUE (UKF): MAG + sun vector + gyro |
| 3 | `ADS_FINE_SUN` | MEKF: FSS + MAG + gyro |
| 4 | `ADS_FINE_STELLAR` | MEKF: star tracker + gyro |
| 5 | `ADS_GYROLESS` | USQUE with Euler dynamics |

## Model structure

| Block | Content |
| --- | --- |
| `Reference_Models` | Sun, eclipse, IGRF field, LVLH (Section 3.7) |
| `Sensor_Processing` | Calibration, frame transformation, validity checks (Sections 3, 12.1) |
| `Mode_Conditions` | Persistence/hysteresis-filtered guard conditions (Section 4.3) |
| `ADS_Mode_Manager` | Stateflow FSM. Guard functions `T1()`…`T15()` match document Table 6 (`f` = fault-driven, `g` = geometric). `last_transition` outputs the transition ID that fired. |
| `ADS_Estimators` | Active-mode filter and hand-over rules (Sections 5–11, Table 8) |
| `ADS_Outputs` | Outputs and flags (Table 9) |

The algorithms are in `lib/` as code-generation-compatible MATLAB functions. The MATLAB Function blocks only call them.
`ads_setup.m` runs automatically when the model loads and initialises. It adds `lib/` to the path and creates `ADS_P` and the bus objects.

## Usage

```matlab
build_ads_model(true)   % regenerate ADS_v1.slx from the scripts
run_ads_demo            % closed-loop scenario: mode log, accuracy, tests/ads_demo.png
run('tests/test_ads_library.m')   % unit checks of the library
```

Edit `lib/ads_params.m` for sensor and threshold values. Load the official IGRF table
before using real data: `ADS_P.igrf = ads_load_igrf('igrf14coeffs.txt', 10);`. The
built-in default is only degree 2.
