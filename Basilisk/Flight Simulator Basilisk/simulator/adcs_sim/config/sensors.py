"""Sensor configuration.

Inventory items **S1**-**S7**.

.. warning::
   As shipped, **every sensor path is error-free truth data**.  ``SimpleNav``
   supports a Gauss-Markov noise + bounded random-walk error model, but with
   ``NAV_ERROR_ENABLED = False`` its error matrices are left unset and it is a
   pass-through of the true spacecraft state.  Any pointing-accuracy conclusion
   drawn from a run in this configuration is optimistic by an unquantified
   margin.  Backlog #2 covers switching this on.
"""

import numpy as np

# ===========================================================================
# S1 -- Navigation (attitude, body rate, position, velocity)
# ===========================================================================
NAV_MODEL_TAG = "SimpleNavigation"

#: Master switch for the ``SimpleNav`` error model.
NAV_ERROR_ENABLED = False

#: 18x18 error standard-deviation matrix used when the error model is enabled.
#: Row/column order is [r_BN_N(3), v_BN_N(3), sigma_BN(3), omega_BN_B(3),
#: sunPnt(3), accumDV(3)].  Populate before flipping ``NAV_ERROR_ENABLED``.
NAV_P_MATRIX = np.zeros((18, 18))

#: Random-walk bounds, one per state above.
NAV_WALK_BOUNDS = np.zeros(18)


# ===========================================================================
# S2-S6 -- Discrete sensors (not yet instantiated)
# ===========================================================================
# Each of these is a Basilisk built-in that only needs wiring; see the
# inventory for the error sources each one would introduce.
#
#   S2 star tracker      -> Basilisk.simulation.starTracker
#   S3 gyro / IMU        -> Basilisk.simulation.imuSensor
#   S4 magnetometer      -> Basilisk.simulation.magnetometer (MagnetometerTAM)
#   S5 sun sensor        -> Basilisk.simulation.coarseSunSensor (+ eclipse,
#                           albedo; estimators in fswAlgorithms.sunline*)
#   S6 RW tachometer     -> Basilisk.simulation.encoder
#
# When one is added, give it a switch and a parameter block here, a module in
# adcs_sim/models/, and update its inventory row.
STAR_TRACKER_ENABLED = False
IMU_ENABLED = False
MAGNETOMETER_ENABLED = False
COARSE_SUN_SENSOR_ENABLED = False
RW_ENCODER_ENABLED = False
