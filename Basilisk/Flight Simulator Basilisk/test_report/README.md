# Test report

Formal test-case evidence for the generated ADCS controller. The document
[`ADCS_TEST_CASES.html`](ADCS_TEST_CASES.html) declares the cases; running them
executes each one, produces the figures that evidence its criteria, and writes
the measured outcome back into the same document.

## Run this

**File to run: [`run_test_cases.py`](run_test_cases.py)**

No terminal? **Double-click `Run_Test_Cases.exe`** — it runs every declared
case in order and fills the document in.

From a terminal:

```powershell
python run_test_cases.py                  # every case, in order
python run_test_cases.py --list           # show what would run, run nothing
python run_test_cases.py --case TC_01     # one case (repeatable)
python run_test_cases.py --from TC_07     # resume from a case onward
```

Ten cases, several of them thousands of simulated seconds, so a full pass takes
a while. The HTML is rewritten **after each case completes**, so you can leave
the document open in a browser and refresh it to watch results land one at a
time.

Then open [`ADCS_TEST_CASES.html`](ADCS_TEST_CASES.html) in a browser to read
the filled-in report.

## What is in here

| Path | Purpose |
| --- | --- |
| [`run_test_cases.py`](run_test_cases.py) | **The entry point.** Runs the declared cases and writes the results back. |
| [`ADCS_TEST_CASES.html`](ADCS_TEST_CASES.html) | The report, and the **source of truth** for what gets run. |
| [`fill_document.py`](fill_document.py) | Fills the surrounding fields — metadata and the requirements matrix. Run once, see below. |
| [`plots/`](plots/) | Evidence figures, one folder per case (`plots/TC_01/…`). Regenerated per run. |
| [`ACS_v1_test_case_report.pdf`](ACS_v1_test_case_report.pdf) | Exported snapshot of an earlier report. |
| `Run_Test_Cases.exe` | Double-click launcher for the entry point. |

## The document drives the run

`ADCS_TEST_CASES.html` is not just output. `run_test_cases.py` parses the
`CASES` array inside it and reconstructs each case's initial conditions from
the state labels declared there:

| Label | Applied to the simulation |
| --- | --- |
| `TUMBLED` | Body rate = (0.03, −0.02, 0.04) rad/s |
| `DETUMBLED` | Body rate = 0 |
| `OFF-POINTING` | Detumbled, attitude rotated 8° about body X |
| `SATURATED` | Every wheel at the momentum limit (50 N m s) |
| `DESATURATED` | Every wheel at rest |

**Change a label in the HTML and the run changes with it.** Keep the field
order and structure of the `CASES` array intact when editing.

Each case gets only the figures that evidence its own criteria, not a fixed
set — a saturated pointing case needs the ACS-mode trace and the momentum
trace, a detumble case needs the body rate.

## `fill_document.py`

`run_test_cases.py` owns the measured outcomes — status, actual result, run
date, plots. `fill_document.py` fills everything *around* them: document
metadata, the requirements traceability matrix, and the narrative fields that
explain a result rather than record it.

```powershell
python fill_document.py
```

These are **defaults, not overrides**: the page prefers whatever a human typed
into it (kept in browser storage), so re-running never overwrites your edits.

> **Before using this as evidence:** the requirement statements it inserts were
> derived from the modes the software actually implements, **not** from a
> controlled requirements document. They exist so the matrix is not empty.
> Replace the IDs and text with the real ones, then confirm every requirement
> still maps to at least one case.

## Relationship to the other folders

Cases are executed through the scenarios in
[`../regression_tests/`](../regression_tests/) against the simulator in
[`../simulator/`](../simulator/). Both must be present and working for a run to
succeed. Use the regression tests for a quick "did anything break"; use this
folder when you need a document to hand over.
