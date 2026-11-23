"""Filesystem locations used by the scenario.

All paths are derived from this file's own location, so the scenario can be
moved or checked out anywhere without editing constants.
"""

import os
from pathlib import Path

#: Directory name of the generated Simulink code — also the marker used to
#: locate the repository root, so the package can sit at any depth below it.
GENERATED_CODE_DIR_NAME = "ADCS_Software_v1_grt_rtw"


def _find_repo_root(start):
    """Return the first ancestor of *start* that holds the generated code.

    Walking up beats a fixed number of ``.parent`` hops: the scenario keeps
    working when the folder tree around it is rearranged.
    """
    for candidate in (start, *start.parents):
        if (candidate / GENERATED_CODE_DIR_NAME).is_dir():
            return candidate
    return start.parent


#: ``.../ADCS_software_test/simulator/adcs_sim``
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
#: ``.../ADCS_software_test/simulator`` — where the entry-point script lives.
SCENARIO_DIR = PACKAGE_ROOT.parent
#: ``.../Basilisk`` — the repository root.
REPO_ROOT = _find_repo_root(SCENARIO_DIR)

#: Generated Simulink code directory and the shared library built from it.
ADCS_DIR = REPO_ROOT / GENERATED_CODE_DIR_NAME

#: Standalone flight-code package; ``make dll`` there builds the same DLL from
#: the sources that go onto the processor.
STANDALONE_DIR = REPO_ROOT / "ADCS_Software_v1_standalone"

#: Shareable simulator layout (``ADCS_Simulator_standalone``): the flight-code
#: package sits at ``flight_software/`` and the 3-D model at ``assets/``, both
#: next to the entry-point script, so the folder runs wherever it is copied.
BUNDLED_FLIGHT_SOFTWARE_DIR = SCENARIO_DIR / "flight_software"
IS_STANDALONE_BUNDLE = BUNDLED_FLIGHT_SOFTWARE_DIR.is_dir()

if IS_STANDALONE_BUNDLE:
    _DEFAULT_LIBRARY = (BUNDLED_FLIGHT_SOFTWARE_DIR / "build"
                        / "ADCS_Software_v1.dll")
    SPACECRAFT_MODEL_FILE = SCENARIO_DIR / "assets" / "m4v.glb"
else:
    _DEFAULT_LIBRARY = ADCS_DIR / "ADCS_Software_v1.dll"
    SPACECRAFT_MODEL_FILE = REPO_ROOT / "3d models" / "m4v.glb"

#: The DLL the scenario flies.  Set the ``ADCS_LIBRARY`` environment variable
#: to fly another build, e.g. the standalone package's
#: ``ADCS_Software_v1_standalone/build/ADCS_Software_v1.dll``.
ADCS_LIBRARY = Path(os.environ.get("ADCS_LIBRARY", _DEFAULT_LIBRARY))

#: Run artifacts (plots and the Vizard recording) are collected here instead
#: of being scattered next to the source.  Override per-run with ``--output-dir``.
OUTPUT_DIR = SCENARIO_DIR / "output"
VIZ_FILE_NAME = "test2.bin"

#: Model-fidelity tracker that this package is documented against.
MODEL_INVENTORY = SCENARIO_DIR / "ADCS_SIM_MODEL_INVENTORY.md"


def resolve_output_dir(output_dir=None):
    """Return the run's output directory, creating it if necessary."""
    directory = Path(output_dir) if output_dir is not None else OUTPUT_DIR
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def viz_file(output_dir=None):
    """Return the Vizard binary path inside the run's output directory."""
    return resolve_output_dir(output_dir) / VIZ_FILE_NAME
