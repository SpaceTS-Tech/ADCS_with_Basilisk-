"""Companion satellite flying in close proximity to the main spacecraft.

A second, passive spacecraft added purely so the formation can be visualised:
it has mass properties and orbital dynamics, but **no controller, no
actuators and no sensors**, and it is not connected to the ADCS flight code.

It does not perturb the main spacecraft.  Basilisk integrates each
``Spacecraft`` independently in the central body's field -- there is no
inter-spacecraft gravity -- so enabling this changes the main satellite's
trajectory not at all.  It only costs integration time.

Visual identity: leaving ``COMPANION_MODEL_PATH`` at ``None`` means no custom
model is applied, so Vizard draws it with its **own built-in satellite mesh**,
which distinguishes it at a glance from the M4V ``.glb``.
"""

import numpy as np

#: Add the companion satellite to the simulation and the Vizard scene.
ENABLE_COMPANION_SATELLITE = True

#: Basilisk model tag.  Must differ from ``config.spacecraft.MODEL_TAG``:
#: Vizard rejects duplicate spacecraft names.
COMPANION_MODEL_TAG = "companion-Sat"

# ---------------------------------------------------------------------------
# Mass properties
# ---------------------------------------------------------------------------
#: A small free-flying companion.  Values are nominal -- nothing depends on
#: them except the companion's own rotational motion.
COMPANION_MASS_KG = 50.0

#: Row-major 3x3 inertia [kg m^2].
COMPANION_INERTIA_B = [
    5.0, 0.0, 0.0,
    0.0, 5.0, 0.0,
    0.0, 0.0, 5.0,
]

COMPANION_COM_OFFSET_B = [[0.0], [0.0], [0.0]]

# ---------------------------------------------------------------------------
# Relative placement
# ---------------------------------------------------------------------------
#: Along-track separation from the main spacecraft [m].  Positive puts the
#: companion **ahead** in the orbit, negative trails it.
#:
#: Implemented as a small offset in true anomaly on an otherwise identical
#: orbit.  That matters: identical semi-major axis means identical period, so
#: the separation is stable and the pair stays together indefinitely.
#:
#: .. note::
#:    Radial and cross-track offsets are deliberately **not** offered.  Simply
#:    displacing the position would change the orbit and the companion would
#:    drift away or oscillate.  A bounded relative orbit in those directions
#:    needs a proper Clohessy-Wiltshire initialisation, which is real relative
#:    -motion work rather than a visualisation tweak -- add it explicitly if
#:    the formation geometry ever matters.
COMPANION_ALONG_TRACK_OFFSET_M = 60.0

# ---------------------------------------------------------------------------
# Attitude
# ---------------------------------------------------------------------------
#: Start LVLH-aligned like the main spacecraft, then tumble freely.
COMPANION_ALIGN_INITIAL_ATTITUDE_TO_LVLH = True

#: Initial body rate [rad/s].  A slow spin reads clearly as an uncontrolled
#: free body next to the actively stabilised main satellite.  Set to zeros for
#: a static companion.
COMPANION_INITIAL_RATE_B = np.array([0.0, 0.0, 0.02])

# ---------------------------------------------------------------------------
# Appearance
# ---------------------------------------------------------------------------
#: ``None`` applies no custom model, so Vizard uses its **built-in satellite**.
#: Other built-in options accepted here are the primitives ``"CUBE"``,
#: ``"CYLINDER"`` and ``"SPHERE"``; a filesystem path would load a CAD model.
COMPANION_MODEL_PATH = None

#: Only used when ``COMPANION_MODEL_PATH`` is not ``None``.
COMPANION_MODEL_OFFSET_M = (0.0, 0.0, 0.0)
COMPANION_MODEL_ROTATION_DEG = (0.0, 0.0, 0.0)
COMPANION_MODEL_SCALE = (1.0, 1.0, 1.0)

#: Albedo tint as RGBA 0-255, or ``None`` for the model's own colours.  A tint
#: is an easy way to tell the two spacecraft apart when both use built-in
#: shapes.  Ignored unless a custom model or primitive is set above.
COMPANION_MODEL_COLOR = None
