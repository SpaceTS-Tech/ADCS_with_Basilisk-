"""ctypes mirrors of the generated ``ADCS_Software_v1`` data structures.

.. important::
   These layouts must match the generated C headers exactly.  If the Simulink
   model's inport/outport list changes, regenerate the code **and** update
   these classes in the same commit -- a mismatch silently reads adjacent
   memory rather than raising.

   ============================  ====================================
   Class                         Generated type
   ============================  ====================================
   :class:`ADCSInputs`           ``ExtU_ADCS_Software_v1_T``
   :class:`ADCSOutputs`          ``ExtY_ADCS_Software_v1_T``
   :class:`ADCSBlockSignals`     ``B_ADCS_Software_v1_T``
   ============================  ====================================

Sensor/actuator interface
-------------------------
The simulator hands the flight software exactly what its sensors measure and
applies its commands exactly as the actuators would.  **No frame is converted
on either side of the DLL and nothing depends on the ACS mode** (since
2026-09-15); any inertial/body conversion is the flight software's own job.

==============  ===============================================================
``q_BI``        Quaternion of the **body relative to inertial**, scalar first
                -- Basilisk ``q_BN = rbk.MRP2EP(sigma_BN)``; ``C(q)`` maps
                inertial components to body components.
``omega``       Body angular rate relative to inertial, **body axes** (gyro).
``B``           Magnetic field, body axes (magnetometer).
``r_sat_i`` ..  Position/velocity, Sun and ground target in ECI (J2000,
                Earth-centred) -- GPS and ephemeris products.
``Camerafeed``  Camera frame = body frame.  Measured 2026-08-03 over 24
                rendered frames: increasing image column is **+X_body**
                (+705.8 px/rad), increasing row is **+Y_body** (+665.0 px/rad).
``STR1``        Star tracker frame from the +X_body camera (instrument camera
                3), same 512x512 column-major layout as ``Camerafeed``.  Model
                2.125 declares this inport but does not read it yet.
``RW_tau``      Torque on the spacecraft, body axes.  The bridge commands the
                wheel motor torque ``-RW_tau``.
``THR_tau``     Torque on the spacecraft, body axes.
``MTQ_tau``     Magnetorquer rod voltages, one per body axis.
``q_BI_est``    Estimated attitude of the body relative to inertial, assumed
                scalar first like ``q_BI``.  Model 2.125 never writes it, so it
                reads all zeros.
==============  ===============================================================
"""

import ctypes

#: Symbol names exported by the DLL for the three structures above.
INPUT_SYMBOL = "ADCS_Software_v1_U"
OUTPUT_SYMBOL = "ADCS_Software_v1_Y"
BLOCK_SIGNAL_SYMBOL = "ADCS_Software_v1_B"

#: Dimensions of the ``Camera feed`` inport.  The generated struct declares a
#: flat ``uint8_T Camerafeed[262144]``; the 2-D shape is not recoverable from
#: the C, so it is stated here and cross-checked against the DLL at load time
#: by :class:`~adcs_sim.flightsoftware.library.GeneratedADCSLibrary` -- a
#: mismatch raises instead of silently mis-sizing the mirror.
CAMERA_FEED_WIDTH = 512
CAMERA_FEED_HEIGHT = 512
CAMERA_FEED_SIZE = CAMERA_FEED_WIDTH * CAMERA_FEED_HEIGHT

#: Dimensions of the ``STR 1`` star tracker inport, ``uint8_T STR1[262144]``.
#: Cross-checked against the DLL at load time in the same way.
STAR_TRACKER_FEED_WIDTH = 1024
STAR_TRACKER_FEED_HEIGHT = 1024
STAR_TRACKER_FEED_SIZE = STAR_TRACKER_FEED_WIDTH * STAR_TRACKER_FEED_HEIGHT


class ADCSInputs(ctypes.Structure):
    """ctypes mirror of ``ExtU_ADCS_Software_v1_T``.

    All vectors are ECI / J2000 unless noted:

    ==============  ====================================================
    Field           Meaning
    ==============  ====================================================
    ``q_BI``        Attitude quaternion of the body relative to inertial,
                    scalar first (Basilisk ``q_BN``).
    ``omega``       Body angular rate relative to inertial, in body axes.
    ``B``           Magnetic field in **body** axes [T].
    ``RW_omega``    Reaction-wheel speeds [rad/s].
    ``v_sat_i``     Spacecraft velocity [m/s].
    ``r_sat_i``     Spacecraft position [m].
    ``r_p_i``       Ground-target position [m].
    ``v_sun_i``     Sun velocity [m/s].
    ``r_sun_i``     Sun position [m].
    ``Camerafeed``  512x512 grayscale frame, one ``uint8`` per pixel.
    ``STR1``        512x512 grayscale star tracker frame, one ``uint8``
                    per pixel.
    ==============  ====================================================

    .. warning::
       ``Camerafeed`` and ``STR1`` are stored **column-major**, the order
       Simulink flattens a 2-D signal into.  C and numpy images are row-major,
       so a frame must be flattened with ``order="F"`` before it is copied in
       or the model receives it transposed.  Prefer
       :meth:`~adcs_sim.flightsoftware.library.GeneratedADCSLibrary.set_camera_feed`
       and ``set_star_tracker_feed``, which do a single ``memcpy`` -- assigning
       262144 elements one at a time through ctypes is far too slow for the
       20 Hz control loop.
    """

    _fields_ = [
        ("q_BI", ctypes.c_double * 4),
        ("omega", ctypes.c_double * 3),
        ("B", ctypes.c_double * 3),
        ("RW_omega", ctypes.c_double * 3),
        ("v_sat_i", ctypes.c_double * 3),
        ("r_sat_i", ctypes.c_double * 3),
        ("r_p_i", ctypes.c_double * 3),
        ("v_sun_i", ctypes.c_double * 3),
        ("r_sun_i", ctypes.c_double * 3),
        ("Camerafeed", ctypes.c_uint8 * CAMERA_FEED_SIZE),
        ("STR1", ctypes.c_uint8 * STAR_TRACKER_FEED_SIZE),
    ]


class ADCSOutputs(ctypes.Structure):
    """ctypes mirror of ``ExtY_ADCS_Software_v1_T``.

    ``RW_tau`` and ``THR_tau`` are torques on the spacecraft in **body** axes
    (see the interface table in the module docstring).  ``MTQ_tau`` is a
    per-rod *voltage* command despite the name.  ``q_BI_est`` is the flight
    software's attitude estimate; model 2.125 leaves it at zero.
    """

    _fields_ = [
        ("actual_ephem", ctypes.c_uint8),
        ("actual_ads", ctypes.c_uint8),
        ("mode_degraded", ctypes.c_bool),
        ("RW_tau", ctypes.c_double * 3),
        ("THR_tau", ctypes.c_double * 3),
        ("MTQ_tau", ctypes.c_double * 3),
        ("q_BI_est", ctypes.c_double * 4),
    ]


class ADCSBlockSignals(ctypes.Structure):
    """ctypes mirror of ``B_ADCS_Software_v1_T``, for mode diagnostics.

    These are internal block signals, exposed so the scenario can plot and log
    the desired-vs-actual mode transitions of the ADCS manager.  They are
    diagnostics only and never influence a command.

    .. warning::
       The image arrays must stay first.  It is the image-processing intermediate of
       ``<S21>/MATLAB Function2`` (the thresholded frame, then the largest
       connected blob), and the generated code declares it **ahead** of the
       mode signals.  Getting it wrong does not fail: ``in_dll`` happily binds
       at the symbol's address and every mode field is read from the wrong
       place.  It has happened twice -- the image arrays were missing until
       2026-08-03 (``actual_acs`` read 0 while the model was in 8), and model
       2.125 replaced ``int32_T y[], x[]`` with ``boolean_T x[]``, moving the
       mode fields 1.8 MB.  :class:`~adcs_sim.flightsoftware.library.GeneratedADCSLibrary`
       now compares ``ctypes.sizeof`` of every mirror with the size the DLL
       reports and refuses to load on a mismatch.

    Model 2.128 added the star tracker ``<Root>/C Function`` (``st_sl_step``)
    and its three non-outport outputs between ``x`` and the mode signals:
    attitude [RA, Dec, roll] in deg, the valid flag (1.0 = fix), and the
    ``ST_INFO_*`` diagnostics.  Its quaternion goes straight to ``Y.q_BI_est``.

    Model 2.133 dropped those three ``CFunction_o*`` signals and instead
    declares five image-processing work arrays ahead of the mode signals:
    ``int32_T stack[]``, ``boolean_T x[]``, ``boolean_T visited[]`` (one per
    camera pixel), ``real_T found[15000]`` and ``real_T predicted[3306]``.

    Model 2.135 widened ``STR 1`` to 1024x1024.  ``stack`` and ``visited``
    are the star detector's work arrays, so they follow the star tracker
    size, and the generator now orders them ``stack, visited, x`` -- ``x``
    stays at the payload camera size.

    Model 2.139 no longer generates the star tracker at all -- ``STR 1`` is
    declared but unread, and ``stack``, ``visited``, ``found`` and
    ``predicted`` are gone, leaving only ``x`` ahead of the mode signals.
    Model 2.141 restored the star tracker and with it the 2.135 layout.
    """

    _fields_ = [
        ("stack", ctypes.c_int32 * STAR_TRACKER_FEED_SIZE),
        ("visited", ctypes.c_bool * STAR_TRACKER_FEED_SIZE),
        ("x", ctypes.c_bool * CAMERA_FEED_SIZE),
        ("found", ctypes.c_double * 15000),
        ("predicted", ctypes.c_double * 3306),
        ("desired_ephem", ctypes.c_uint8),
        ("desired_ads", ctypes.c_uint8),
        ("desired_frame", ctypes.c_uint8),
        ("desired_acs", ctypes.c_uint8),
        ("actual_frame", ctypes.c_uint8),
        ("actual_acs", ctypes.c_uint8),
        ("Compare", ctypes.c_bool),
        ("Compare_i", ctypes.c_bool),
        ("Compare_h", ctypes.c_bool),
    ]
