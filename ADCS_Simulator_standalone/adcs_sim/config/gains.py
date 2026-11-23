"""Control-law gains and plant constants of the generated ADCS.

These are the user-tunable numbers of the flight software.  In the generated
code they are plain ``extern real_T`` globals in ``ADCS_Software_v1.c``, not
entries in the parameter struct, so they can be written straight through the
DLL's exported symbols -- no Simulink change and no rebuild needed to retune.

Every value here is applied at controller reset by
:meth:`~adcs_sim.flightsoftware.controller.GeneratedADCSController.Reset`, and
the applied set is printed in the run banner.  Set an entry to ``None`` to
leave the model's compiled-in default in place.

.. warning::
   The values compiled into the model are largely **placeholders** -- most
   gains are ``1.0`` and ``I_RW`` is ``1.0`` against a real wheel inertia of
   ``0.0796 kg m^2``.  Defining them here is what makes a run reproducible;
   relying on the compiled-in defaults means the flight software is flying
   numbers nobody chose.
"""

from adcs_sim.config import actuators as act_cfg

# ---------------------------------------------------------------------------
# Plant constants the control law needs
# ---------------------------------------------------------------------------
#: Reaction-wheel spin inertia as seen by the flight software [kg m^2].
#:
#: Must match the simulated wheel or the controller's momentum bookkeeping is
#: wrong.  The model ships with ``1.0``, which is 12.6x the actual HR16 value,
#: so this is not an optional override.
#:
#: This is the same quantity the ``ADCS_set_rw_inertia`` wrapper call used to
#: set.  That entry point became a no-op when the model was regenerated
#: (``I_RW`` left the tunable-parameter struct); writing the global directly
#: is the route that still works.
I_RW = act_cfg.CONTROLLER_I_RW

#: Desaturation law gain.
K_DESAT = 0.1

# ---------------------------------------------------------------------------
# Attitude control gains, per ACS mode
# ---------------------------------------------------------------------------
# Each mode has a proportional (attitude error) and derivative (rate error)
# gain.  The generated law forms the wheel torque as roughly
# ``tau = -kp * q_err_vector + kd * omega_err``.

#: Precision pointing -- nadir, Sun and ground-target modes, AND the optical
#: tracking law (``<S18>/Gain2``), which share this symbol.
#:
#: Must stay positive.  It was left at ``-0.1`` while the optical sign error
#: was being chased; with the signs now correct in ``<S18>`` a negative value
#: re-inverts optical tracking *and* inverts nadir/Sun pointing along with it,
#: because both read this same gain.
#:
#: 1.0 against the block's 0.005 N m/px keeps the optical command inside the
#: 0.2 N m wheel limit out to 40 px of centroid error.
KP_POINT = 1.0
KD_POINT = 20.0

#: Attitude hold.
KP_HOLD = 1.0
KD_HOLD = 20.0

#: Large-angle slews.
KP_SLEW_FAST = 1.0

KD_SLEW_FAST = 1.0

KP_SLEW_SLOW = 1.0
KD_SLEW_SLOW = 1.0

#: Controlled spin.
KP_SPIN = 1.0
KD_SPIN = 20.0

#: Detumble is rate-only, so there is no proportional term.
KD_DETUMBLE = 1.0


#: Maps each configuration name above onto the exported symbol in the DLL.
#: Anything added to the generated model must be added here to be reachable.
SYMBOLS = {
    "I_RW": "I_RW",
    "K_DESAT": "k_desat",
    "KP_POINT": "kp_point",
    "KD_POINT": "kd_point",
    "KP_SPIN": "kp_spin",
    "KD_SPIN": "kd_spin",
    "KD_DETUMBLE": "kd_detumble",
}


def configured():
    """Return ``{dll_symbol: value}`` for every gain that is not ``None``."""
    values = globals()
    return {symbol: values[name]
            for name, symbol in SYMBOLS.items()
            if values.get(name) is not None}
