"""Bridge between Basilisk and the generated ``ADCS_Software_v1`` DLL.

This is inventory item **C1**: irreducibly custom code with no Basilisk
equivalent.  It is split so that the three concerns can be tested and changed
independently:

=========================  =================================================
Module                     Contents
=========================  =================================================
:mod:`~.structs`           ctypes mirrors of the generated ``ExtU``/``ExtY``/
                           ``B`` structures.  Must track the generated code.
:mod:`~.library`           Typed handle for the DLL: loading, tuning entry
                           points, ``initialize``/``step``.
:mod:`~.diagnostics`       Mode-history and saturation bookkeeping.
:mod:`~.controller`        The ``SysModel`` that marshals Basilisk messages
                           into and out of the DLL each step.
=========================  =================================================
"""

__all__ = ["controller", "diagnostics", "library", "structs"]
