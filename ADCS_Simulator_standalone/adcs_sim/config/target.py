"""Ground-target definition.

Inventory item **S7**.
"""

#: Fixed Indian ground target.
TARGET_LATITUDE_DEG = 28.522431782788708
TARGET_LONGITUDE_DEG = 77.02428245359896

# .. warning::
#    SPHERICAL EARTH -- inventory action S7 / backlog #7.  The target position
#    is placed on a sphere of this radius with no WGS-84 flattening.  At 28.5 N
#    the geodetic-vs-geocentric difference is tens of kilometres of position
#    error, which feeds straight into the pointing reference.  Basilisk's
#    ``groundLocation`` module handles the reference ellipsoid properly, plus
#    access windows, elevation masks and range.
TARGET_RADIUS_M = 6_378_137.0

#: Set True once the target is placed with a proper ellipsoid model, so the
#: inventory status and this flag cannot drift apart.
USE_REFERENCE_ELLIPSOID = False
