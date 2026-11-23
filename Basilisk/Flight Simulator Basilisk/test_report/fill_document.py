"""Fill the non-test-result fields of ``ADCS_TEST_CASES.html``.

``run_test_cases.py`` owns the measured outcome of each case -- status, actual
result, run date, binary stamp, plots.  This script fills everything *around*
those: document metadata, the requirements traceability matrix, and the
narrative fields that explain a result rather than record it.

Those fields are **defaults, not overrides**.  The page prefers whatever a human
typed into it (kept in browser storage) and falls back to these, so re-running
this never overwrites your edits.

.. warning::
   The requirement statements below were derived from the modes the software
   actually implements, **not** from a controlled requirements document.  They
   are a starting point so the RTM is not empty.  Replace the IDs and text with
   the real ones before this document is used as evidence -- and then check that
   every requirement still maps to at least one case, which is the whole point
   of an RTM.

Usage::

    python fill_document.py
"""

import sys
from datetime import date
from pathlib import Path

_DIR = Path(__file__).resolve().parent
if str(_DIR) not in sys.path:
    sys.path.insert(0, str(_DIR))

from run_test_cases import HTML_FILE, load_results, write_results  # noqa: E402

# ---------------------------------------------------------------------------
# Requirements traceability
# ---------------------------------------------------------------------------
REQUIREMENTS = {
    "REQ_01":
        "The ADCS shall reduce an initial body rate to below 20% of its "
        "starting magnitude using the detumble control law, without exceeding "
        "actuator limits.",
    "REQ_02":
        "The ADCS shall acquire and hold nadir pointing, aligning the +Z "
        "payload boresight with the nadir direction to within 5 deg and "
        "holding it against orbital motion.",
    "REQ_03":
        "The ADCS shall acquire and hold Sun pointing, aligning the +Z "
        "boresight with the Sun direction in the inertial frame to within "
        "15 deg.",
    "REQ_04":
        "The ADCS shall track a specified Earth-fixed ground target, keeping "
        "the +Z boresight on the target to within 15 deg as the Earth "
        "rotates beneath the orbit.",
    "REQ_05":
        "The ADCS shall track a co-orbiting satellite optically, holding the "
        "target within 25 px (2.0 deg) of the camera frame centre, and shall "
        "acquire a target that is initially off-boresight but within the "
        "camera field of view.",
    "REQ_06":
        "When reaction-wheel stored momentum reaches the desaturation entry "
        "threshold (0.95 x h_max), the ADCS shall shed momentum below the exit "
        "threshold (0.25 x h_max) before executing the commanded pointing "
        "mode.",
}

# ---------------------------------------------------------------------------
# Document metadata
# ---------------------------------------------------------------------------
MODEL_VERSION = "2.57"
BASILISK_VERSION = "2.11.1"
PYTHON_VERSION = "3.12.10"

METADATA = {
    "meta.docid": "ADCS-STP-001",
    "meta.rev": "1.0 (draft)",
    "meta.date": date.today().isoformat(),
    "meta.sut":
        f"ADCS_Software_v1.dll, built from Simulink model version "
        f"{MODEL_VERSION} (grt.tlc, x86-64). Contains hand-applied patches to "
        f"the generated C -- see Observations on TC_08.",
    "meta.env":
        f"Basilisk {BASILISK_VERSION}, Python {PYTHON_VERSION} (AMD64), "
        f"Windows 11. Headless: no Vizard, synthetic camera feed for the "
        f"optical cases.",
}

# ---------------------------------------------------------------------------
# Narrative fields -- what a number does not say on its own
# ---------------------------------------------------------------------------
NOTES = {
    "TC_07.notes":
        "FAIL is a test-definition issue, not a software defect -- pending a "
        "specification decision.\n\n"
        "Both mission objectives are met in full. Over 4000 s (80 000 control "
        "steps, roughly 40% of an orbit) the body rate falls 5.385e-02 -> "
        "1.750e-06 rad/s and wheel momentum drains 4.0 -> 0.0 N m s, well "
        "below the 1.0 N m s exit band. Yet ACS_DETUMBLE occupies 100% of "
        "steps and ACS_DESAT is entered 0 times, so the 'DESAT precedes the "
        "commanded mode' criterion cannot pass.\n\n"
        "WHY DESAT CANNOT FIRE (measured against model 2.63, which DOES "
        "contain a '(desired_acs == 7) || Compare_i -> ACS_DESAT' branch in "
        "the DETUMBLE state). The branch is present and correct; the two "
        "windows it depends on never overlap:\n"
        "  - Saturated (Compare_i = 1) only for roughly the first 75 s. Wheel "
        "friction pulls momentum below the 3.8 N m s entry band by t = 100 s "
        "(h1: 3.969 at t=10, 3.849 at t=50, 3.703 at t=100).\n"
        "  - The DETUMBLE state only evaluates its transitions when "
        "'(desired_acs != 1) || (rate_measure <= CompareToConstant1_const)'. "
        "Under mission command 1 desired_acs is pinned at 1, so transitions "
        "wait until the body rate reaches ~0, which happens around t = 3000 s.\n"
        "  - By then h = 0.000 N m s and Compare_i = 0, so the DESAT branch is "
        "correctly skipped -- there is nothing left to desaturate.\n\n"
        "Saturation therefore ends about 40x before the state is permitted to "
        "act on it. Making TC_07 pass as written would require saturation to "
        "PREEMPT detumble (evaluating the DESAT transition regardless of "
        "detumble completion), i.e. interrupting stabilisation of a tumbling "
        "vehicle to unload wheels that are draining on their own.\n\n"
        "Momentum drains through wheel friction while in DETUMBLE -- RW_tau is "
        "zero in that mode (the RW torque switch has no case 1 and falls "
        "through to the IDLE branch), so the wheels simply coast down against "
        "Coulomb and viscous drag: 4.0 -> 0.0 N m s in about 2500 s.\n\n"
        "ACTION: confirm whether detumble is specified to outrank "
        "desaturation. If it is, this case should assert that priority "
        "(ACS_DETUMBLE occupied, ACS_DESAT absent) instead of the DESAT-first "
        "ordering, which would turn it into a genuine test of the rule. The "
        "criterion has deliberately not been relaxed without that decision.",

    "TC_08.notes":
        "Passes only with two defects corrected in the desaturation law. Both "
        "were required together -- fixing either alone makes the behaviour "
        "worse.\n\n"
        "1. SIGN. '<S25>/Gain2' generated THR_tau = +k_desat*RW_omega. "
        "Desaturation needs an external torque OPPOSING stored momentum, so "
        "the gain must be negative. As generated the thrusters pushed with the "
        "momentum, driving the wheels further into saturation.\n\n"
        "2. FRAME. The bridge rotated THR_tau by dcm_BN unconditionally. In "
        "DESAT that command is built from RW_omega and is therefore already in "
        "body axes, so the rotation swept the torque direction around the body "
        "once per orbit; measured on 2026-08-10 the components reversed near "
        "t = 1200 s and the wheels spun back up and pinned at the speed limit. "
        "Corrected by DESAT_BODY_FRAME in adcs_sim/config/conventions.py.\n\n"
        "The sign fix is currently a HAND PATCH to the generated C. "
        "Regenerating from Simulink will revert it -- set '<S25>/Gain2' to "
        "-k_desat in the model to make it permanent.",

    "summary.assessment":
        "9 of 10 cases pass.\n\n"
        "All five mission modes verified: detumble, nadir pointing, Sun "
        "pointing, Earth-target tracking and inter-satellite optical tracking, "
        "each from a nominal start. Desaturation before pointing is verified "
        "for nadir, Sun and Earth-target (TC_08 - TC_10).\n\n"
        "TC_07 fails on a criterion now believed to be incorrectly specified; "
        "see its Observations. A 4000 s run confirms this is structural rather "
        "than a timing effect -- the wheels empty completely and the vehicle "
        "comes to rest without DESAT ever being entered. No software defect is "
        "currently open against it.\n\n"
        "CAVEATS ON THIS EVIDENCE:\n"
        "- The binary contains hand-applied patches to generated C in the "
        "optical (<S18>) and desaturation (<S25>) laws. Until those changes "
        "are made in the Simulink model, any regeneration invalidates these "
        "results.\n"
        "- TC_01 - TC_06 were last run against an earlier binary and an "
        "earlier reaction-wheel configuration (50 N m s catalogue wheel rather "
        "than the 4 N m s flight wheel). They should be re-run so the whole "
        "document reflects a single build before it is used as evidence.\n"
        "- The controller is fed truth state: no sensor models are "
        "implemented, and no disturbance torques are enabled. Pointing "
        "accuracy here is therefore an upper bound.",
}


def main():
    data = load_results(HTML_FILE)
    data.update(METADATA)
    data.update(NOTES)
    for req_id, text in REQUIREMENTS.items():
        data[f"rtm.{req_id}"] = text
    write_results(data, HTML_FILE)

    print(f"Filled {HTML_FILE.name}:")
    print(f"  {len(METADATA)} metadata fields")
    print(f"  {len(REQUIREMENTS)} requirement statements")
    print(f"  {len(NOTES)} narrative fields")
    print("\nThese are defaults; anything typed into the page wins over them.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
