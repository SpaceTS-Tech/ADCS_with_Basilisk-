"""Reaction-wheel cluster: 3x Honeywell HR16, orthogonal and body-aligned.

Inventory item **A1**.  What *is* modelled: full wheel/hub gyroscopic coupling,
motor-torque saturation, wheel-speed limit.  What is *not*: friction, imbalance
jitter, motor electrical dynamics, power draw, encoder quantisation -- all of
which are Basilisk built-in options gated by switches in
:mod:`adcs_sim.config.actuators`.

.. note::
   ``rwFactory.create`` does not merely leave friction unset when
   ``useRWfriction`` is False -- it **overwrites** the HR16 profile's
   ``fCoulomb`` (0.0005 N m) with zero, and likewise forces ``u_min`` to zero
   when ``useMinTorque`` is False.  That overwrite happens *after* keyword
   arguments are processed, so it also discards any explicit ``fCoulomb``.
   :func:`validate_friction_config` turns that trap into a loud error.

Friction coefficients
---------------------
Set them in :mod:`adcs_sim.config.actuators`:

=========================  ===================  ==========================
Config name                Device field         Applied by Basilisk 2.11.0?
=========================  ===================  ==========================
``RW_COULOMB_FRICTION``    ``fCoulomb``         yes
``RW_VISCOUS_FRICTION``    ``cViscous``         yes
``RW_LIMIT_CYCLE_SPEED``   ``omegaLimitCycle``  yes
``RW_STATIC_FRICTION``     ``fStatic``          **no -- inert**
``RW_STRIBECK_BETA``       ``betaStatic``       **no -- inert**
=========================  ===================  ==========================

``None`` means "keep the HR16 device profile's value", which is **not** the
same as zero.

The API advertises a Stribeck breakaway term, but this build's effector does
not apply one.  Measured ``frictionTorque`` at 11 speeds from 7.6e-8 to
628 rad/s -- including with ``frictionStribeck`` forced True -- is always:

.. code-block:: text

    |Omega| >= omegaLimitCycle:  tau = -sign(Omega)*fCoulomb - cViscous*Omega
    |Omega| <  omegaLimitCycle:  tau ramps linearly from 0 to that value

It never rises toward ``fStatic`` anywhere.  The near-zero ramp is a numerical
anti-chatter device, not stiction: a stopped wheel is not held.  If breakaway
friction matters to a result, it has to be added to the effector.
"""

from dataclasses import dataclass

import numpy as np
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.utilities import macros, simIncludeRW

from adcs_sim.config import actuators as act_cfg
from adcs_sim.config import simulation as sim_cfg

#: Rated wheel speed of the HR16 profile [rad/s] (6000 RPM), used for drag and
#: momentum reporting.  ``Js`` is derived from it by ``rwFactory``.
RATED_SPEED_RAD_S = act_cfg.RW_PHYSICAL_H_MAX / act_cfg.CONTROLLER_I_RW


@dataclass
class ReactionWheels:
    """The wheel effector and the factory that built its configuration."""

    effector: object
    #: Retained so the RW configuration objects outlive the call.
    factory: object


# ---------------------------------------------------------------------------
# Initial state
# ---------------------------------------------------------------------------

def initial_speed_rpm(initial_omega_rpm=None, initial_momentum_nms=None):
    """Return the three initial wheel speeds in the RPM Basilisk expects.

    Exactly one of the two overrides may be given.  With neither, the wheels
    start at :data:`~adcs_sim.config.actuators.DEFAULT_INITIAL_RW_MOMENTUM_NMS`
    -- the saturation limit, so a run begins in DESAT.

    Raises
    ------
    ValueError
        If both overrides are supplied, or the result is not three finite
        numbers.
    """
    if initial_omega_rpm is not None and initial_momentum_nms is not None:
        raise ValueError("Specify initial RW speed or momentum, not both.")

    if initial_omega_rpm is not None:
        omega_rpm = np.asarray(initial_omega_rpm, dtype=float)
    else:
        momentum_nms = (act_cfg.DEFAULT_INITIAL_RW_MOMENTUM_NMS
                        if initial_momentum_nms is None
                        else np.asarray(initial_momentum_nms, dtype=float))
        omega_rad_s = momentum_nms / act_cfg.CONTROLLER_I_RW
        omega_rpm = omega_rad_s / macros.RPM

    if omega_rpm.shape != (3,) or not np.all(np.isfinite(omega_rpm)):
        raise ValueError("Initial RW values must be three finite numbers.")
    return omega_rpm


def momentum_nms(omega_rpm):
    """Convert wheel speeds in RPM to stored momentum [N m s]."""
    return act_cfg.CONTROLLER_I_RW * np.asarray(omega_rpm) * macros.RPM


def describe_initial_state(omega_rpm):
    """Return printable lines describing the initial wheel state."""
    lines = ["Initial reaction-wheel state:"]
    for index, (omega, momentum) in enumerate(
            zip(omega_rpm, momentum_nms(omega_rpm)), start=1):
        saturated = ("SATURATED"
                     if abs(momentum) >= act_cfg.RW_PHYSICAL_H_MAX else "")
        lines.append(f"  RW{index}: {omega:+.6f} RPM, "
                     f"h={momentum:+.6f} N m s {saturated}".rstrip())
    return lines


# ---------------------------------------------------------------------------
# Command conditioning (inventory item C3)
# ---------------------------------------------------------------------------

def limit_motor_torque(motor_torque_B):
    """Clamp a generated motor-torque command to the HR16 rating.

    The same limit is enforced inside ``ReactionWheelStateEffector``; applying
    it here as well keeps an unstable or non-finite generated command out of
    the coupled spacecraft-wheel dynamics entirely.
    """
    limit = act_cfg.RW_MAX_MOTOR_TORQUE
    return np.clip(
        np.nan_to_num(motor_torque_B, nan=0.0, posinf=limit, neginf=-limit),
        -limit, limit)


# ---------------------------------------------------------------------------
# Construction
# ---------------------------------------------------------------------------

def validate_friction_config():
    """Reject friction settings that ``rwFactory`` would silently discard.

    ``rwFactory.create`` applies ``if not useRWfriction: RW.fCoulomb = 0.0``
    *after* processing keyword arguments, so a ``RW_COULOMB_FRICTION`` value set
    while ``RW_USE_FRICTION`` is False is thrown away without a word.  Worse,
    that gate zeroes **fCoulomb only** -- ``fStatic`` and ``cViscous`` survive
    it, so "friction off" would not actually mean friction off.  Fail loudly
    instead.
    """
    if act_cfg.RW_USE_FRICTION:
        return

    ignored = [name for name, value in (
        ("RW_COULOMB_FRICTION", act_cfg.RW_COULOMB_FRICTION),
        ("RW_STATIC_FRICTION", act_cfg.RW_STATIC_FRICTION),
        ("RW_VISCOUS_FRICTION", act_cfg.RW_VISCOUS_FRICTION),
        ("RW_STRIBECK_BETA", act_cfg.RW_STRIBECK_BETA),
    ) if value is not None]
    if ignored:
        raise ValueError(
            f"RW_USE_FRICTION is False but {', '.join(ignored)} "
            f"{'is' if len(ignored) == 1 else 'are'} set in config.actuators. "
            "rwFactory would discard the Coulomb value and yet still apply the "
            "static/viscous terms. Set RW_USE_FRICTION = True, or clear these "
            "coefficients back to None."
        )


def _factory_options(omega_rpm):
    """Build the ``rwFactory.create`` keyword arguments for one wheel."""
    options = {
        "maxMomentum": float(act_cfg.RW_PHYSICAL_H_MAX),
        "Omega": float(omega_rpm),
        "useMaxTorque": act_cfg.RW_USE_MAX_TORQUE,
        "u_max": float(act_cfg.RW_MAX_MOTOR_TORQUE),
    }
    # A catalogue device (Honeywell_HR16 etc.) carries its own rated speed and
    # spin inertia, and rejects a momentum capacity that is not one of its
    # listed variants.  A custom device has no data sheet, so both have to be
    # supplied here or Basilisk cannot size the wheel.
    #
    # Supply the rated speed only, never ``Js`` as well: the factory rejects
    # "Js both directly and through maxMomentum and Omega_max".  It derives
    # ``Js = maxMomentum / Omega_max``, which reproduces CONTROLLER_I_RW
    # exactly because RW_MAX_SPEED_RPM is defined from those same two numbers.
    if act_cfg.RW_DEVICE_TYPE == "custom":
        options["Omega_max"] = float(act_cfg.RW_MAX_SPEED_RPM)
    # Only pass the fidelity options that are actually enabled: the factory
    # validates them and rejects some sentinel values (e.g. betaStatic == 0).
    if act_cfg.RW_USE_FRICTION:
        options["useRWfriction"] = True
    if act_cfg.RW_USE_MIN_TORQUE:
        options["useMinTorque"] = True
        options["u_min"] = float(act_cfg.RW_MIN_MOTOR_TORQUE)
    # Friction coefficients. Every one of these defaults to None, meaning
    # "keep the device profile's own value", so omitting them is not the same
    # as setting them to zero.
    if act_cfg.RW_COULOMB_FRICTION is not None:
        options["fCoulomb"] = float(act_cfg.RW_COULOMB_FRICTION)
    if act_cfg.RW_STATIC_FRICTION is not None:
        options["fStatic"] = float(act_cfg.RW_STATIC_FRICTION)
    if act_cfg.RW_VISCOUS_FRICTION is not None:
        options["cViscous"] = float(act_cfg.RW_VISCOUS_FRICTION)
    if act_cfg.RW_STRIBECK_BETA is not None:
        options["betaStatic"] = float(act_cfg.RW_STRIBECK_BETA)
    if act_cfg.RW_IMBALANCE_MODEL is not None:
        options["RWModel"] = act_cfg.RW_IMBALANCE_MODEL
    return options


def _apply_post_create_overrides(device):
    """Set device parameters that ``rwFactory.create`` exposes no keyword for.

    ``create`` returns the very object it stored in ``rwList`` (not a copy), so
    mutating it here is effective -- provided it happens before
    ``addToSpacecraft`` copies the configuration into the effector.
    """
    if act_cfg.RW_LIMIT_CYCLE_SPEED is not None:
        device.omegaLimitCycle = float(act_cfg.RW_LIMIT_CYCLE_SPEED)


def peak_drag_torque():
    """Total friction torque at the wheel's rated speed [N m].

    ``fCoulomb + cViscous * Omega_max`` -- the two terms this Basilisk build
    actually applies.  Reported so the drag can be sanity-checked against
    ``RW_MAX_MOTOR_TORQUE`` at a glance.
    """
    coulomb = act_cfg.RW_COULOMB_FRICTION or 0.0
    viscous = act_cfg.RW_VISCOUS_FRICTION or 0.0
    return coulomb + viscous * RATED_SPEED_RAD_S


def describe_friction():
    """Return printable lines describing the friction actually in effect.

    Deliberately reports the *measured* law, not the configured parameters:
    the Stribeck term is configured but inert in Basilisk 2.11.0, and printing
    "Stribeck ON" would misrepresent the run.
    """
    if not act_cfg.RW_USE_FRICTION:
        return ["  RW friction: OFF (fCoulomb forced to 0 by rwFactory)"]

    def shown(value, profile_note):
        return f"{value:.6g}" if value is not None else f"profile ({profile_note})"

    drag = peak_drag_torque()
    lines = [
        "  RW friction: ON"
        f" | Coulomb={shown(act_cfg.RW_COULOMB_FRICTION, '5e-4')} N m"
        f" | viscous={shown(act_cfg.RW_VISCOUS_FRICTION, '0')} N m s/rad"
        f" -> drag at rated speed {drag:.3e} N m "
        f"({100.0 * drag / act_cfg.RW_MAX_MOTOR_TORQUE:.1f}% of u_max)"
    ]
    if (act_cfg.RW_STRIBECK_BETA is not None
            and act_cfg.RW_STRIBECK_BETA > 0):
        lines.append(
            f"    static/Stribeck: fStatic="
            f"{shown(act_cfg.RW_STATIC_FRICTION, '0')} N m, beta="
            f"{act_cfg.RW_STRIBECK_BETA:.6g} rad/s"
            " -- CONFIGURED BUT INERT (this Basilisk applies Coulomb+viscous only)")
    return lines


def add_reaction_wheels(sim, task_name, hub_spacecraft, omega_rpm):
    """Create the HR16 cluster, attach it to the hub, and register it."""
    if len(omega_rpm) != len(act_cfg.RW_SPIN_AXES):
        raise ValueError(
            f"Expected {len(act_cfg.RW_SPIN_AXES)} initial wheel speeds, "
            f"got {len(omega_rpm)}")
    validate_friction_config()

    factory = simIncludeRW.rwFactory()
    for axis, omega in zip(act_cfg.RW_SPIN_AXES, omega_rpm):
        device = factory.create(act_cfg.RW_DEVICE_TYPE, list(axis),
                                **_factory_options(omega))
        _apply_post_create_overrides(device)

    effector = reactionWheelStateEffector.ReactionWheelStateEffector()
    effector.ModelTag = act_cfg.RW_MODEL_TAG
    factory.addToSpacecraft(hub_spacecraft.ModelTag, effector, hub_spacecraft)
    sim.AddModelToTask(task_name, effector,
                       sim_cfg.REACTION_WHEEL_PRIORITY)
    return ReactionWheels(effector=effector, factory=factory)
