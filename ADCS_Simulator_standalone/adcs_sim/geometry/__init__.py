"""Mission geometry.

Pure functions over vectors and rotations -- no Basilisk modules, no
simulation state -- so everything here is directly unit-testable.

=========================  =================================================
Module                     Contents
=========================  =================================================
:mod:`~.ground_target`     Ground-target position in planet-fixed and ECI.
:mod:`~.pointing`          Body-frame direction and alignment-error helpers.
=========================  =================================================
"""

__all__ = ["ground_target", "pointing"]
