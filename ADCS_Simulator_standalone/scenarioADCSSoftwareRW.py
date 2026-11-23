"""Run Basilisk with the generated ADCS_Software_v1 reaction-wheel controller.

This file is a thin entry point.  The scenario itself lives in the
:mod:`adcs_sim` package next to it -- see ``adcs_sim/README.md`` for the module
map and ``ADCS_SIM_MODEL_INVENTORY.md`` for the fidelity status of every
simulated sensor, actuator and environment model.

Usage
-----
``--mode file`` (default)
    Run for a fixed duration, record a Vizard file and write the analysis
    figures into ``output/``.

``--mode live``
    Stream indefinitely to Vizard over a socket, with keyboard control of the
    mission command (``0``-``9``, ``a``, ``b``), pause (``p``) and quit (``q``).

The reaction wheels start at their momentum limit by default, so a run begins
in desaturation; use ``--rw-omega-rpm`` or ``--rw-momentum`` to start elsewhere.

Examples
--------
.. code-block:: text

    python scenarioADCSSoftwareRW.py
    python scenarioADCSSoftwareRW.py --seconds 500 --show-plots
    python scenarioADCSSoftwareRW.py --rw-momentum 0 0 0
    python scenarioADCSSoftwareRW.py --mode live
"""

import sys
from pathlib import Path

# Make ``adcs_sim`` importable when this file is imported from elsewhere (a
# test runner, another scenario) rather than executed directly.
_SCENARIO_DIR = Path(__file__).resolve().parent
if str(_SCENARIO_DIR) not in sys.path:
    sys.path.insert(0, str(_SCENARIO_DIR))

from adcs_sim.scenario.cli import main  # noqa: E402
from adcs_sim.scenario.runner import run  # noqa: E402

__all__ = ["main", "run"]


if __name__ == "__main__":
    main()
