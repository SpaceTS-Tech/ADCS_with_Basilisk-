"""Typed ctypes handle for the generated ``ADCS_Software_v1`` shared library.

Keeping the DLL surface in its own class means the Basilisk-facing controller
never touches ``ctypes`` directly, and the set of tuning entry points the
generated code exposes is documented in exactly one place.
"""

import ctypes
from pathlib import Path

import numpy as np

from adcs_sim.flightsoftware import structs


class GeneratedADCSLibrary:
    """Load ``ADCS_Software_v1.dll`` and expose its entry points and state.

    Parameters
    ----------
    library_path : str or pathlib.Path
        Path to the shared library built from ``ADCS_Software_v1_grt_rtw``.

    Attributes
    ----------
    inputs : structs.ADCSInputs
        Live view of the generated ``ExtU`` struct.  Writing to it sets the
        controller inputs for the next :meth:`step`.
    outputs : structs.ADCSOutputs
        Live view of the generated ``ExtY`` struct, valid after :meth:`step`.
    block_signals : structs.ADCSBlockSignals
        Live view of internal block signals used for mode diagnostics.
    """

    def __init__(self, library_path):
        library_path = Path(library_path)
        if not library_path.is_file():
            raise FileNotFoundError(
                f"Generated ADCS library not found: {library_path}\n"
                "Build it first: 'mingw32-make dll' in the flight-code package "
                "(flight_software/ or ADCS_Software_v1_standalone/), or set "
                "ADCS_LIBRARY to an existing DLL."
            )

        self.path = library_path
        self._library = ctypes.CDLL(str(library_path))
        self._declare_signatures()

        self._check_camera_feed_size()
        self._check_star_tracker_feed_size()
        self._check_struct_sizes()

        self.inputs = structs.ADCSInputs.in_dll(
            self._library, structs.INPUT_SYMBOL)
        self.outputs = structs.ADCSOutputs.in_dll(
            self._library, structs.OUTPUT_SYMBOL)
        self.block_signals = structs.ADCSBlockSignals.in_dll(
            self._library, structs.BLOCK_SIGNAL_SYMBOL)

    def _declare_signatures(self):
        """Declare argument/return types for every entry point we call."""
        library = self._library
        library.ADCS_set_mission_cmd.argtypes = [ctypes.c_uint8]
        library.ADCS_set_rw_inertia.argtypes = [ctypes.c_double]
        library.ADCS_set_h_max.argtypes = [ctypes.c_double]
        library.ADCS_get_h_max.restype = ctypes.c_double
        library.ADCS_get_rw_saturated.restype = ctypes.c_uint8
        library.ADCS_get_desat_complete.restype = ctypes.c_uint8
        library.ADCS_set_ads_fine_available.argtypes = [ctypes.c_uint8]
        # Wheel inertia became non-tunable when the model was regenerated on
        # 2026-07-31; the wrapper now reports whether the value was applied.
        library.ADCS_set_rw_inertia.restype = ctypes.c_int
        library.ADCS_has_rw_inertia_parameter.restype = ctypes.c_int
        # Camera feed.
        library.ADCS_get_camera_feed_size.restype = ctypes.c_uint
        library.ADCS_set_camera_feed.argtypes = [ctypes.c_void_p, ctypes.c_uint]
        library.ADCS_set_camera_feed.restype = ctypes.c_int
        # Star tracker feed and attitude estimate (model 2.125).
        library.ADCS_get_star_tracker_feed_size.restype = ctypes.c_uint
        library.ADCS_set_star_tracker_feed.argtypes = [ctypes.c_void_p,
                                                       ctypes.c_uint]
        library.ADCS_set_star_tracker_feed.restype = ctypes.c_int
        library.ADCS_get_q_BI_est.argtypes = [ctypes.POINTER(ctypes.c_double)]
        library.ADCS_get_q_BI_est.restype = ctypes.c_int
        # True sizes of the generated structs, for the mirror check.
        library.ADCS_get_inputs_size.restype = ctypes.c_uint
        library.ADCS_get_outputs_size.restype = ctypes.c_uint
        library.ADCS_get_block_signals_size.restype = ctypes.c_uint

    def _check_struct_sizes(self):
        """Fail if a ctypes mirror is not the size of the generated struct.

        ``in_dll`` never checks a layout, so a mirror left behind by a
        regeneration reads adjacent memory without an error.  Model 2.125 did
        this to the block signals.  The wrapper exports ``sizeof`` of each
        generated struct; any added, removed or resized field changes it.
        """
        pairs = [
            ("ExtU (inputs)", structs.ADCSInputs,
             self._library.ADCS_get_inputs_size),
            ("ExtY (outputs)", structs.ADCSOutputs,
             self._library.ADCS_get_outputs_size),
            ("B (block signals)", structs.ADCSBlockSignals,
             self._library.ADCS_get_block_signals_size),
        ]
        mismatched = [
            f"{name}: DLL {int(reported())} bytes, mirror {ctypes.sizeof(mirror)} bytes"
            for name, mirror, reported in pairs
            if int(reported()) != ctypes.sizeof(mirror)]
        if mismatched:
            raise RuntimeError(
                "ctypes mirrors in structs.py do not match the generated "
                f"structs in {self.path.name}: " + "; ".join(mismatched)
                + ". Update structs.py from ADCS_Software_v1.h.")

    def _check_star_tracker_feed_size(self):
        """Fail if the DLL's star tracker inport is not the assumed size."""
        reported = int(self._library.ADCS_get_star_tracker_feed_size())
        if reported != structs.STAR_TRACKER_FEED_SIZE:
            raise RuntimeError(
                f"Star tracker inport size mismatch: the DLL reports {reported} "
                f"elements but structs.py assumes {structs.STAR_TRACKER_FEED_SIZE} "
                f"({structs.STAR_TRACKER_FEED_WIDTH}x"
                f"{structs.STAR_TRACKER_FEED_HEIGHT}).")

    def _check_camera_feed_size(self):
        """Fail if the DLL's camera inport is not the size the mirror assumes.

        :class:`~adcs_sim.flightsoftware.structs.ADCSInputs` hard-codes the
        512x512 array length because ctypes needs it at class-definition time.
        The DLL reports its own size, so any resolution change in the Simulink
        model is caught here rather than silently mis-mapping the struct.
        """
        reported = int(self._library.ADCS_get_camera_feed_size())
        if reported != structs.CAMERA_FEED_SIZE:
            raise RuntimeError(
                f"Camera inport size mismatch: the DLL reports {reported} "
                f"elements but structs.py assumes {structs.CAMERA_FEED_SIZE} "
                f"({structs.CAMERA_FEED_WIDTH}x{structs.CAMERA_FEED_HEIGHT}). "
                "Update CAMERA_FEED_WIDTH/HEIGHT to match the regenerated "
                "model, or the ctypes mirror will misread memory.")

    # -- lifecycle ---------------------------------------------------------

    def initialize(self):
        """Run the generated model's initialize function."""
        self._library.ADCS_Software_v1_initialize()

    def step(self):
        """Advance the generated model by one base sample time."""
        self._library.ADCS_Software_v1_step()

    # -- tuning entry points ----------------------------------------------

    def set_mission_cmd(self, mission_cmd):
        self._library.ADCS_set_mission_cmd(ctypes.c_uint8(int(mission_cmd)))

    def set_rw_inertia(self, wheel_inertia):
        """Set the controller's wheel inertia.  Returns True if it took effect.

        The model regenerated on 2026-07-31 dropped ``I_RW`` from its tunable
        parameters, so on that build this is a no-op and returns False.  The
        inertia the control law uses is then whatever is compiled into the
        model, and nothing will report a mismatch with the simulated wheel.
        """
        return bool(self._library.ADCS_set_rw_inertia(
            ctypes.c_double(float(wheel_inertia))))

    def has_rw_inertia_parameter(self):
        """Whether this DLL exposes a tunable wheel inertia at all."""
        return bool(self._library.ADCS_has_rw_inertia_parameter())

    def set_h_max(self, max_wheel_momentum):
        self._library.ADCS_set_h_max(ctypes.c_double(float(max_wheel_momentum)))

    def get_h_max(self):
        return self._library.ADCS_get_h_max()

    def set_ads_fine_available(self, available):
        self._library.ADCS_set_ads_fine_available(
            ctypes.c_uint8(1 if available else 0))

    # -- control-law gains -------------------------------------------------
    #
    # The gains are plain `extern real_T` globals in the generated code, not
    # entries in the parameter struct, so they are reached through the DLL's
    # exported symbols rather than through a setter.

    def get_gain(self, symbol):
        """Read one exported ``real_T`` global by name."""
        try:
            return ctypes.c_double.in_dll(self._library, symbol).value
        except ValueError as error:
            raise KeyError(
                f"'{symbol}' is not an exported symbol of {self.path.name}. "
                "Check the name against ADCS_Software_v1.h, and that the DLL "
                "was linked with --export-all-symbols.") from error

    def set_gain(self, symbol, value):
        """Write one exported ``real_T`` global by name."""
        try:
            ctypes.c_double.in_dll(self._library, symbol).value = float(value)
        except ValueError as error:
            raise KeyError(
                f"'{symbol}' is not an exported symbol of {self.path.name}. "
                "Check the name against ADCS_Software_v1.h, and that the DLL "
                "was linked with --export-all-symbols.") from error

    def apply_gains(self, gains):
        """Write a ``{symbol: value}`` mapping and read every value back.

        Returns
        -------
        dict
            ``{symbol: readback}``.  Read back rather than assumed, so a
            symbol that silently failed to take cannot pass unnoticed.

        Raises
        ------
        RuntimeError
            If any value does not read back as written.
        """
        applied = {}
        for symbol, value in gains.items():
            self.set_gain(symbol, value)
            applied[symbol] = self.get_gain(symbol)

        mismatched = {symbol: (gains[symbol], applied[symbol])
                      for symbol in gains
                      if applied[symbol] != float(gains[symbol])}
        if mismatched:
            raise RuntimeError(f"Gains did not take effect: {mismatched}")
        return applied

    # -- camera feed -------------------------------------------------------

    def camera_feed_size(self):
        """Number of ``uint8`` elements the camera inport holds."""
        return int(self._library.ADCS_get_camera_feed_size())

    def set_camera_feed(self, frame):
        """Copy one grayscale frame into the camera inport.

        Parameters
        ----------
        frame : numpy.ndarray
            ``uint8`` array of exactly ``CAMERA_FEED_SIZE`` elements, already
            flattened **column-major** (``order="F"``) and C-contiguous.

        Returns
        -------
        bool
            True if the frame was accepted.  False means the DLL rejected the
            length -- which should be impossible after the constructor's size
            check, so treat it as a bug rather than a routine outcome.

        Notes
        -----
        Goes through the wrapper's ``memcpy`` rather than ctypes element
        assignment: 262144 per-element writes per 0.05 s step is not viable.
        """
        buffer = np.ascontiguousarray(frame, dtype=np.uint8)
        if buffer.size != structs.CAMERA_FEED_SIZE:
            raise ValueError(
                f"Camera frame has {buffer.size} elements, expected "
                f"{structs.CAMERA_FEED_SIZE}")
        return bool(self._library.ADCS_set_camera_feed(
            buffer.ctypes.data_as(ctypes.c_void_p), buffer.size))

    def clear_camera_feed(self):
        """Blank the camera inport, so a stale frame is not held over."""
        self._library.ADCS_clear_camera_feed()

    # -- star tracker feed and attitude estimate -----------------------------

    def star_tracker_feed_size(self):
        """Number of ``uint8`` elements the ``STR 1`` inport holds."""
        return int(self._library.ADCS_get_star_tracker_feed_size())

    def set_star_tracker_feed(self, frame):
        """Copy one grayscale frame into the ``STR 1`` star tracker inport.

        Same contract as :meth:`set_camera_feed`: ``uint8``, exactly
        ``STAR_TRACKER_FEED_SIZE`` elements, flattened column-major.
        """
        buffer = np.ascontiguousarray(frame, dtype=np.uint8)
        if buffer.size != structs.STAR_TRACKER_FEED_SIZE:
            raise ValueError(
                f"Star tracker frame has {buffer.size} elements, expected "
                f"{structs.STAR_TRACKER_FEED_SIZE}")
        return bool(self._library.ADCS_set_star_tracker_feed(
            buffer.ctypes.data_as(ctypes.c_void_p), buffer.size))

    def clear_star_tracker_feed(self):
        """Blank the ``STR 1`` inport, so a stale frame is not held over."""
        self._library.ADCS_clear_star_tracker_feed()

    def get_q_BI_est(self):
        """The flight software's attitude estimate ``q_BI_est`` (4 elements).

        Valid after :meth:`step`.  Model 2.125 never writes this output, so it
        returns zeros on that build.
        """
        q_est = (ctypes.c_double * 4)()
        if not self._library.ADCS_get_q_BI_est(q_est):
            raise RuntimeError("ADCS_get_q_BI_est rejected its output buffer")
        return np.array(q_est, dtype=float)

    # -- status flags ------------------------------------------------------

    def get_rw_saturated(self):
        """Non-zero while the controller considers the wheels saturated."""
        return self._library.ADCS_get_rw_saturated()

    def get_desat_complete(self):
        """Non-zero once the controller declares desaturation complete."""
        return self._library.ADCS_get_desat_complete()


class NullADCSLibrary:
    """Stand-in for the DLL when no flight software is connected.

    Same interface as :class:`GeneratedADCSLibrary`, but nothing is loaded and
    nothing is computed: every input written is accepted and ignored, and every
    output -- ``RW_tau``, ``MTQ_tau``, ``THR_tau`` and the mode/status
    signals -- stays zero.  The simulator therefore flies the spacecraft with
    **no control torque or force**, so only the environment (gravity
    gradient, drag, SRP, residual dipole, wheel friction...) acts on it.

    Use it to run the simulator on its own, without any ADCS software; select
    it with ``--no-flight-software`` or ``ADCS_LIBRARY=none``.
    """

    #: Shown wherever the DLL path would be.
    path = None

    def __init__(self):
        # Plain ctypes structs, not views into a DLL: same field layout, so the
        # bridge and the diagnostics read them exactly as they read the DLL's.
        self.inputs = structs.ADCSInputs()
        self.outputs = structs.ADCSOutputs()
        self.block_signals = structs.ADCSBlockSignals()
        self._h_max = 0.0

    # -- lifecycle ---------------------------------------------------------

    def initialize(self):
        ctypes.memset(ctypes.byref(self.outputs), 0, ctypes.sizeof(self.outputs))
        ctypes.memset(ctypes.byref(self.block_signals), 0,
                      ctypes.sizeof(self.block_signals))

    def step(self):
        """No software: the outputs stay zero."""

    # -- tuning entry points: accepted, nothing to tune ---------------------

    def set_mission_cmd(self, mission_cmd):
        del mission_cmd

    def set_rw_inertia(self, wheel_inertia):
        del wheel_inertia
        return False

    def has_rw_inertia_parameter(self):
        return False

    def set_h_max(self, max_wheel_momentum):
        self._h_max = float(max_wheel_momentum)

    def get_h_max(self):
        return self._h_max

    def set_ads_fine_available(self, available):
        del available

    def apply_gains(self, gains):
        del gains
        return {}

    # -- image inports: frames are dropped --------------------------------

    def camera_feed_size(self):
        return structs.CAMERA_FEED_SIZE

    def set_camera_feed(self, frame):
        del frame
        return True

    def clear_camera_feed(self):
        pass

    def star_tracker_feed_size(self):
        return structs.STAR_TRACKER_FEED_SIZE

    def set_star_tracker_feed(self, frame):
        del frame
        return True

    def clear_star_tracker_feed(self):
        pass

    def get_q_BI_est(self):
        return np.zeros(4)

    def get_rw_saturated(self):
        return 0

    def get_desat_complete(self):
        return 0


#: Values of ``library_path`` / ``ADCS_LIBRARY`` that mean "no flight software".
NO_FLIGHT_SOFTWARE = ("none", "off", "no", "")


def is_no_flight_software(library_path):
    """True when *library_path* selects :class:`NullADCSLibrary`."""
    return (library_path is None
            or str(library_path).strip().lower() in NO_FLIGHT_SOFTWARE)


def open_library(library_path):
    """The DLL at *library_path*, or the zero-output stand-in for ``None``."""
    if is_no_flight_software(library_path):
        return NullADCSLibrary()
    return GeneratedADCSLibrary(library_path)
