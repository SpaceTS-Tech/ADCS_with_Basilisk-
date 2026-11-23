"""Electrical power: solar-array geometry, eclipse and the power monitor.

Inventory items **P1** (solar array) and **P2** (eclipse illumination).

Power only -- never dynamics
----------------------------
Every model built from this block *reads* the spacecraft state and the Sun
ephemeris and *writes* power messages.  None of them applies a force or a
torque, and nothing here is fed back to the flight software, so switching the
whole block off with :data:`POWER_MODEL_ENABLED` leaves the control loop
bit-for-bit unchanged.

Solar radiation pressure (inventory E6) and albedo (E9) are separate,
still-unmodelled *disturbances*; the eclipse model enabled here only decides how
much sunlight the cells see.
"""

from collections import namedtuple

#: One flat solar-cell surface.
#:
#: ``normal_B``  outward normal of the cell face in **body** coordinates; it is
#:               normalised before use, so it may be given as a plain axis.
#: ``area_m2``   cell area of that face [m^2].
#: ``efficiency`` conversion efficiency, 0-1.
SolarPanel = namedtuple("SolarPanel", "name normal_B area_m2 efficiency")

#: Master switch.  ``False`` builds no panels, no eclipse and no monitor.
POWER_MODEL_ENABLED = True

# ===========================================================================
# P1 -- Solar array
# ===========================================================================
#: .. warning::
#:    PLACEHOLDER ARRAY GEOMETRY -- replace with the real panel layout, area
#:    and cell efficiency, exactly as the magnetorquer ratings still need the
#:    rod data sheet.  Every number below is a plausible value for a 380 kg
#:    spacecraft, not this spacecraft's.
#:
#: Two wings on the +/-Y faces, cells facing outward on each side.  Body +Z is
#: the payload boresight (see ``config/visualization.PAYLOAD_BORESIGHT_B``), so
#: keeping the cells off +Z is what lets the array generate while the payload
#: points at Earth or at a ground target.
#:
#: A face generates only when the Sun is on its lit side: Basilisk clamps the
#: cosine at zero, so the two wings never generate at the same time and the
#: pair behaves like one wing that can be lit from either side.
SOLAR_PANELS = (
    SolarPanel(name="wing +Y", normal_B=(0.0, 0.0, +1.0),
               area_m2=1.0, efficiency=0.30),
    SolarPanel(name="wing -Y", normal_B=(0.0, 0.0, +1.0),
               area_m2=1.0, efficiency=0.30),
)

#: Model-tag prefix; each panel gets ``<prefix>_<index>``.
SOLAR_PANEL_MODEL_TAG_PREFIX = "SolarPanel"

#: Solar irradiance Basilisk's ``SimpleSolarPanel`` assumes at 1 AU [W/m^2].
#:
#: Mirrored here only so plots can draw the theoretical maximum; the module
#: holds its own copy and does not read this one.  Measured on Basilisk 2.11.1:
#:
#:     netPower = SOLAR_CONSTANT_1AU_W_M2 * (AU / |r_sun - r_sc|)^2
#:                * area * efficiency * max(0, cos(theta)) * sunlit_fraction
#:
#: where the sunlit fraction is the eclipse model's output (1 in full sun, 0 in
#: umbra) -- carried in the message field this build names ``shadowFactor``.
SOLAR_CONSTANT_1AU_W_M2 = 1361.0


# ===========================================================================
# P2 -- Eclipse illumination
# ===========================================================================
#: Model Earth's shadow.  With this off every panel sees full sunlight for the
#: whole orbit, which overstates the energy collected by roughly the eclipse
#: fraction of the orbit.
#:
#: Distinct from ``config/environment.ECLIPSE_ENABLED``, which is the
#: *disturbance* side of inventory item E9 (albedo and Earth IR) and is still
#: unmodelled.
POWER_ECLIPSE_ENABLED = True

ECLIPSE_MODEL_TAG = "EclipseEarth"

#: Occulting bodies, by SPICE name.  Each must be one of
#: ``config/environment.GRAVITY_BODIES`` so its state message exists.
ECLIPSE_OCCULTING_BODIES = ("earth",)


# ===========================================================================
# Power monitor (net power and accumulated energy)
# ===========================================================================
POWER_MONITOR_MODEL_TAG = "PowerMonitor"

#: Energy in the store at t = 0 [W s].  0 makes the monitor read as "energy
#: generated since the run started", which is what the live trace shows.
INITIAL_STORED_ENERGY_WS = 0.0

#: Constant housekeeping load subtracted from the generated power [W].
#:
#: 0.0 (the default) means the monitor reports **generation only**, so a number
#: on the live plot is unambiguous.  Set a positive value to see the net
#: charge/discharge balance instead; the monitor then integrates the net, and
#: its energy trace falls during eclipse.
BUS_LOAD_W = 0.0

POWER_SINK_MODEL_TAG = "BusLoad"
