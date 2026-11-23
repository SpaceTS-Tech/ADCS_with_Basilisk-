"""Modular Basilisk scenario for the generated ``ADCS_Software_v1`` flight code.

Package map (see ``README.md`` in this directory for the full guide):

======================  ====================================================
Sub-package             Responsibility
======================  ====================================================
``config``              Every tunable number and switch. No Basilisk calls.
``geometry``            Mission geometry (ground targets, pointing vectors).
``flightsoftware``      ctypes bridge to the generated DLL + its diagnostics.
``models``              One module per simulated sensor/actuator/environment.
``scenario``            Assembly, wiring, recorders, visualization, runner.
``analysis``            Derived telemetry, plots, printed summary.
======================  ====================================================

Fidelity status of every model is tracked in
``../ADCS_SIM_MODEL_INVENTORY.md``.  Sub-packages are intentionally *not*
imported here so that importing :mod:`adcs_sim` stays cheap and free of
matplotlib side effects.
"""

__all__ = [
    "analysis",
    "config",
    "flightsoftware",
    "geometry",
    "models",
    "scenario",
]
