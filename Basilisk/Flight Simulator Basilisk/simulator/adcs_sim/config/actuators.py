"""Actuator ratings and fidelity switches.

Inventory items **A1** (reaction wheels), **A2** (magnetorquers),
**A3** (thrusters).  Every switch below defaults to the value that reproduces
the original monolithic scenario, so enabling a higher-fidelity option is a
one-line change here.
"""

import numpy as np

# ===========================================================================
# A1 -- Reaction wheels: 3x custom flight wheel, orthogonal, body-aligned
# ===========================================================================
RW_MODEL_TAG = "RW_cluster"

#: Basilisk RW factory device type.
#:
#: ``"custom"``, **not** a catalogue name.  ``rwFactory`` treats
#: ``Honeywell_HR16`` / ``HR14`` / ``HR12`` / ``BCT_RWP015`` as fixed data
#: sheets and validates against them -- HR16 accepts a momentum capacity of
#: only 100, 75 or 50 N m s and rejects anything else with
#: "does not have a correct wheel momentum ... Provided 4.0 Nm", before any of
#: the values below are used.  ``"custom"`` takes the wheel's real parameters.
#:
#: Supported types: ``Honeywell_HR16``, ``Honeywell_HR14``, ``Honeywell_HR12``,
#: ``BCT_RWP015``, ``custom``.
RW_DEVICE_TYPE = "custom"

#: Body-fixed spin axes, one per wheel.
RW_SPIN_AXES = ((1, 0, 0), (0, 1, 0), (0, 0, 1))

#: Momentum capacity per wheel [N m s].  Flight wheel value.
RW_PHYSICAL_H_MAX = 4.0

#: Wheel spin inertia [kg m^2].  Passed to ``rwFactory`` as ``Js`` for a custom
#: device, and to the generated controller as ``I_RW`` so the flight software
#: and the simulated wheel agree on the same wheel.
CONTROLLER_I_RW = 0.0109

#: Rated wheel speed [RPM], i.e. the speed at which the wheel holds
#: ``RW_PHYSICAL_H_MAX``.  A custom device needs this: ``rwFactory`` has no data
#: sheet to fall back on.  Derived from the two values above so the three stay
#: consistent -- override with a measured figure if the data sheet disagrees,
#: but then check that ``H = Js * Omega_max`` still holds.
RW_MAX_SPEED_RPM = (RW_PHYSICAL_H_MAX / CONTROLLER_I_RW) * 60.0 / (2.0 * np.pi)

#: ``h_max`` parameter inside the generated controller [N m s].
#:
#: Must track the physical wheel.  The generated law enters desaturation at
#: ``0.95 * h_max`` and leaves at ``0.25 * h_max``; leaving this at a value the
#: wheel cannot reach means the controller never sees saturation while the
#: wheel silently pins at its speed limit -- the failure is invisible in the
#: mode trace, which is the worst kind.
CONTROLLER_H_MAX = RW_PHYSICAL_H_MAX

#: Desaturation hysteresis band of the generated control law, as fractions of
#: ``CONTROLLER_H_MAX``.  Mirrored here so the analysis output can state which
#: side of the band a run started on; the authoritative values live inside the
#: generated model.
DESAT_ENTRY_FRACTION = 0.95
DESAT_EXIT_FRACTION = 0.25

#: Peak motor torque [N m].  Applied by the effector and, defensively, at the
#: controller boundary.
RW_MAX_MOTOR_TORQUE = 0.12

#: Initial wheel momentum when neither CLI override is given [N m s].
#: Defaults to zero (wheels at rest) so a run starts from a nominal,
#: unsaturated state.  Pass ``--rw-momentum 4 4 4`` to start saturated.
DEFAULT_INITIAL_RW_MOMENTUM_NMS = np.zeros(3)

# --- fidelity switches (see inventory A1 for what each one restores) -------
#: Enforce ``RW_MAX_MOTOR_TORQUE`` inside the effector.
RW_USE_MAX_TORQUE = True

# ---------------------------------------------------------------------------
# Friction model (inventory A1)
# ---------------------------------------------------------------------------
# MEASURED friction law in Basilisk 2.11.0 (see the warning on
# ``RW_STATIC_FRICTION`` -- the Stribeck term documented by the API is not
# actually applied by this build's effector):
#
#     |Omega| >= omegaLimitCycle:  tau = -sign(Omega) * fCoulomb - cViscous*Omega
#     |Omega| <  omegaLimitCycle:  tau ramps linearly from 0 to that value
#
# The near-zero ramp is a numerical anti-chatter device, not stiction: a
# stopped wheel is not held by static friction.
#
# All four coefficients are set below.  ``None`` means "leave the HR16 device
# profile's own value alone".
#
# .. warning::
#    PROVENANCE -- only ``RW_COULOMB_FRICTION`` (5.0e-4 N m) comes from
#    Basilisk's HR16 device profile.  ``RW_STATIC_FRICTION``,
#    ``RW_STRIBECK_BETA`` and ``RW_VISCOUS_FRICTION`` are **engineering
#    estimates for this wheel class**, not Honeywell data: they were chosen so
#    that total drag at the 6000 RPM rating is ~2.5% of peak motor torque, with
#    a breakaway/Coulomb ratio and a Stribeck velocity typical of preloaded
#    ball bearings.  Each individual figure is defensible; none is measured.
#    Replace them from the flight wheel's drag-torque curve when it is
#    available -- inventory item A1.

#: Master gate for Coulomb friction.  ``rwFactory`` *forces* ``fCoulomb = 0``
#: when this is False, discarding both the profile value and anything set in
#: ``RW_COULOMB_FRICTION`` below.
#: ENABLED 2026-07-30 (inventory A1 / backlog #4).
#:
#: .. warning::
#:    This gate gates **fCoulomb only**.  ``RW_STATIC_FRICTION`` and
#:    ``RW_VISCOUS_FRICTION`` are *not* zeroed by it and would still act with
#:    the gate off, which is why
#:    :func:`adcs_sim.models.reaction_wheels.validate_friction_config` rejects
#:    that combination rather than letting it pass silently.
#:
#: .. note::
#:    Briefly set False on 2026-08-10 to isolate the DESAT transition in
#:    TC_07.  With friction on the wheels drain 4.0 -> 0.0 N m s unaided
#:    (``RW_tau`` is zero in DETUMBLE), so the 0.2 N m s margin between 4.0 and
#:    the 3.8 entry band is gone in ~65 s while detumble is thruster-limited to
#:    >=218 s -- saturation ends before the DETUMBLE state may act on it.  That
#:    is the representative behaviour and is restored here; the FSM guard fix
#:    was verified separately with friction off.
RW_USE_FRICTION = True

#: Coulomb (dry) friction torque coefficient [N m], or ``None`` to keep the
#: HR16 profile value.  **This is the knob to turn to change "the friction
#: coefficient."**  Requires ``RW_USE_FRICTION = True``.
#:
#: Set to Basilisk's own HR16 profile value.  Stated explicitly rather than
#: left at ``None`` so the whole friction curve is visible in one place.
RW_COULOMB_FRICTION = 5.0e-4

#: Static / breakaway (stiction) friction magnitude [N m].  2x Coulomb, the
#: usual ratio for a preloaded ball bearing.
#:
#: .. warning::
#:    **INERT in Basilisk 2.11.0.**  This value and ``RW_STRIBECK_BETA`` are
#:    accepted by ``rwFactory`` and land on the device (verified), but the
#:    shipped ``ReactionWheelStateEffector`` never applies a Stribeck term.
#:    Measured across 11 wheel speeds from 7.6e-8 to 628 rad/s, including with
#:    ``frictionStribeck`` forced True, the reported ``frictionTorque`` is
#:    always exactly ``fCoulomb + cViscous*Omega`` -- it never rises toward
#:    ``fStatic`` at any speed.
#:
#:    They are kept because they are correct values that cost nothing and will
#:    take effect if a future Basilisk implements the term.  Do not rely on
#:    stiction being modelled: if breakaway behaviour matters to a result, it
#:    must be added to the effector.
RW_STATIC_FRICTION = 1.0e-3

#: Wheel speed below which Basilisk suppresses friction-induced numerical limit
#: cycling [rad/s], or ``None`` to keep the profile value (``1.0e-4``).  Raise
#: this if a near-zero-speed wheel chatters at the friction level.
RW_LIMIT_CYCLE_SPEED = None

#: Restore the HR16 minimum-torque deadband: motor-torque commands smaller than
#: ``RW_MIN_MOTOR_TORQUE`` are zeroed by the effector.  ``rwFactory`` forces
#: ``u_min = 0`` when this is False.
#: ENABLED 2026-07-30 (inventory A1 / backlog #4).
#:
#: .. warning::
#:    ``rwFactory.create`` calls ``exit(1)`` -- a hard process exit, not an
#:    exception -- if ``RW_USE_MIN_TORQUE`` is True while
#:    ``RW_MIN_MOTOR_TORQUE`` is not strictly positive.  Keep them consistent.
RW_USE_MIN_TORQUE = True

#: HR16 profile value [N m].  Corresponds to roughly 100 RPM of wheel speed at
#: the peak motor torque.
RW_MIN_MOTOR_TORQUE = 1.0e-5

#: Viscous friction coefficient [N m s/rad], or ``None`` to leave it
#: unmodelled (profile default ``0.0``).  Scales linearly with wheel speed, so
#: unlike Coulomb friction its effect is largest at high RPM and it is what
#: sets the wheel's coast-down time.
#:
#: Chosen so total drag at the 6000 RPM rating is about 5 mN m -- roughly 2.5%
#: of ``RW_MAX_MOTOR_TORQUE``, typical for this wheel class:
#:
#:     tau(628.3 rad/s) = 5.0e-4 + 7.0e-6 * 628.3 = 4.9e-3 N m
#:
#: Cross-checks: bearing drag power ``tau * Omega`` = 3.1 W at full speed, and
#: the viscous coast-down constant ``Js / cViscous`` = 11 400 s (3.2 h), both
#: in the right range for a 50 N m s wheel.
RW_VISCOUS_FRICTION = 7.0e-6

#: Stribeck friction coefficient: positive enables the breakaway model,
#: negative disables it.  ``None`` leaves the profile default (-1.0, disabled).
#: Must never be exactly ``0.0`` -- ``rwFactory`` calls ``exit(1)`` on that,
#: and the effector rejects it too ("Stribeck coefficent currently zero and
#: should be positive to active this friction model").
#: Pair with ``RW_STATIC_FRICTION``, which is the magnitude the model uses.
#:
#: This is the Stribeck *velocity* [rad/s]: the wheel-speed scale over which
#: breakaway friction decays to the Coulomb plateau.  Precision bearings
#: transition well below 1 rad/s.
#:
#: **INERT in Basilisk 2.11.0** -- see the warning on ``RW_STATIC_FRICTION``.
RW_STRIBECK_BETA = 0.5

#: Wheel imbalance model.  ``None`` keeps the factory default
#: (``messaging.BalancedWheels``); set to ``messaging.JitterSimple`` or
#: ``messaging.JitterFullyCoupled`` to model static/dynamic imbalance jitter.
RW_IMBALANCE_MODEL = None


# ===========================================================================
# A2 -- Magnetic torque rods: 3x orthogonal, body-aligned
# ===========================================================================
MTQ_MODEL_TAG = "MTQ_effector"

MTQ_COUNT = 3

#: Rod alignment matrix, row-major, one column per rod.
MTQ_ALIGNMENT_MATRIX_B = [
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
]

# .. warning::
#    PLACEHOLDER RATINGS -- inventory action A2 / backlog #1.  Replace both
#    values with the flight torque-rod data sheet.  The generated ADCS output
#    is a *voltage* command, while ``MtbEffector`` needs a magnetic dipole in
#    A m^2, so these two numbers define the conversion gain.
MTQ_MAX_VOLTAGE = 5.0   #: Applied voltage saturation [V]
MTQ_MAX_DIPOLE = 0.2    #: Dipole produced at full voltage [A m^2]

#: Linear voltage-to-dipole gain [A m^2 / V].  No coil R/L dynamics, no core
#: hysteresis, no remanence -- see :mod:`adcs_sim.models.magnetorquers`.
MTQ_DIPOLE_PER_VOLT = MTQ_MAX_DIPOLE / MTQ_MAX_VOLTAGE


# ===========================================================================
# A3 -- Thrusters (reaction-wheel desaturation)
# ===========================================================================
THR_MODEL_TAG = "Thruster_torque_effector"

# .. warning::
#    PLACEHOLDER MODEL -- inventory action A3 / backlog #6.  There is no
#    thruster here at all: ``ExtForceTorque`` applies the commanded body torque
#    directly, with no force, no moment arm and no firing logic.  Replace with
#    ``thrusterDynamicEffector`` + ``simIncludeThruster`` geometry and a
#    torque->force->on-time allocation.
#
#    The bound below exists because the generated DESAT law currently returns
#    roughly ``-RW_omega`` (~628 N m at 6000 RPM), which ``ExtForceTorque``
#    would apply literally and destabilise the integration.
THR_MAX_BODY_TORQUE = 0.05   #: Per body axis [N m]
