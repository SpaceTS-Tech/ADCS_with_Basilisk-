"""Gravity, ephemeris and space-environment configuration.

Inventory items **E2** (gravity), **E3** (ephemeris), **E4** (magnetic field)
and **E5**-**E9** (disturbances, none of which are modelled yet).
"""

# ===========================================================================
# E2 -- Gravity
# ===========================================================================
#: Bodies created by ``gravBodyFactory``.  Order matters: it fixes the index
#: of each body in ``spiceInterface.planetStateOutMsgs``.  Use
#: :func:`adcs_sim.models.environment.planet_state_index` rather than
#: hard-coding indices.
GRAVITY_BODIES = ("earth", "sun")

#: Which of the above is the integration centre.
CENTRAL_BODY = "earth"

#: Point-mass gravity only -- no J2 or higher harmonics.  Backlog #5 covers
#: loading a spherical-harmonics model for the central body.
USE_SPHERICAL_HARMONICS = False


# ===========================================================================
# E3 -- Ephemeris and time
# ===========================================================================
#: Simulation epoch.  06:30 UTC == 12:00 IST.
SPICE_EPOCH = "2026 JULY 13 06:30:00.0"

#: Re-centre all SPICE output on this body, making every vector handed to the
#: controller Earth-centred J2000 / ECI.
SPICE_ZERO_BASE = "Earth"


# ===========================================================================
# E4 -- Earth magnetic field
# ===========================================================================
MAGNETIC_FIELD_MODEL_TAG = "CenteredDipoleMagneticField"

#: Planet whose centred-dipole coefficients are loaded.
MAGNETIC_FIELD_PLANET = "earth"

#: Tilted centred-dipole model.  Backlog #9 covers upgrading to
#: ``magneticFieldWMM`` for realistic magnetorquer authority.
USE_WMM_MAGNETIC_FIELD = False


# ===========================================================================
# E5-E9 -- Disturbance torques (none enabled)
# ===========================================================================
# The spacecraft currently flies torque-free apart from its own actuators,
# which makes momentum build-up -- and therefore the whole desaturation story
# -- unrepresentative.  All of these are Basilisk built-ins.
#
#   E5 drag              -> exponentialAtmosphere/msisAtmosphere +
#                           dragDynamicEffector.  Negligible at this altitude
#                           (~3600 km); a defensible permanent exclusion.
#   E6 solar pressure    -> radiationPressure / facetSRPDynamicEffector
#   E7 gravity gradient  -> GravityGradientEffector
#   E8 residual dipole   -> constant offset on the MTB dipole command
#   E9 eclipse / albedo  -> eclipse, albedo, earthRadiationModel
#
# E6 and E7 are the dominant secular momentum sources a desat cycle must
# handle; backlog #3 covers adding them.
DRAG_ENABLED = False
SOLAR_RADIATION_PRESSURE_ENABLED = False
GRAVITY_GRADIENT_ENABLED = False
RESIDUAL_DIPOLE_ENABLED = False
ECLIPSE_ENABLED = False
