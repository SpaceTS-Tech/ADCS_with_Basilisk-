/* Small C ABI helpers used by the Basilisk Python scenario. */
#include <string.h>
#include "ADCS_Software_v1.h"

#ifdef _WIN32
#define ADCS_EXPORT __declspec(dllexport)
#else
#define ADCS_EXPORT
#endif

/* Element count of the '<Root>/Camera feed' inport, derived from the
 * generated struct rather than hard-coded, so a resolution change in the
 * Simulink model cannot leave this file silently out of step. */
#define ADCS_CAMERA_FEED_SIZE \
    (sizeof(ADCS_Software_v1_U.Camerafeed) / \
     sizeof(ADCS_Software_v1_U.Camerafeed[0]))

/* Element count of the '<Root>/STR 1' star tracker image inport, derived the
 * same way. */
#define ADCS_STAR_TRACKER_FEED_SIZE \
    (sizeof(ADCS_Software_v1_U.STR1) / sizeof(ADCS_Software_v1_U.STR1[0]))

/* ------------------------------------------------------------------ */
/* Struct sizes                                                         */
/* ------------------------------------------------------------------ */
/* The Python scenario mirrors ExtU, ExtY and B with ctypes.  A mirror that
 * no longer matches the generated struct does not fail: it binds at the
 * symbol's address and reads adjacent memory.  Model 2.125 did exactly that
 * to B, replacing int32_T y[262144] and x[262144] with boolean_T x[262144],
 * which put every mode readback 1.8 MB past the real fields.  Exporting the
 * true sizes lets the caller compare them with its mirrors at load time. */

ADCS_EXPORT unsigned int ADCS_get_inputs_size(void)
{
    return (unsigned int)sizeof(ADCS_Software_v1_U);
}

ADCS_EXPORT unsigned int ADCS_get_outputs_size(void)
{
    return (unsigned int)sizeof(ADCS_Software_v1_Y);
}

ADCS_EXPORT unsigned int ADCS_get_block_signals_size(void)
{
    return (unsigned int)sizeof(ADCS_Software_v1_B);
}

/* ------------------------------------------------------------------ */
/* Tuning parameters                                                    */
/* ------------------------------------------------------------------ */

ADCS_EXPORT void ADCS_set_mission_cmd(unsigned char mission_cmd)
{
    ADCS_Software_v1_P.Constant15_Value = mission_cmd;
}

/* Wheel inertia is NO LONGER TUNABLE.
 *
 * The model regenerated on 2026-07-31 dropped `I_RW` from
 * P_ADCS_Software_v1_T; `h_max` is still there.  Whatever wheel inertia the
 * control law uses is now baked into the generated code, so this setter has
 * nothing to write to.
 *
 * It is kept as an exported symbol so the ABI does not change under callers
 * that already bind it, but it now reports failure instead of pretending to
 * work: returns 1 if the value was applied, 0 if this model build has no
 * tunable inertia.  Callers must check.
 *
 * To restore it: mark the wheel-inertia workspace variable tunable in the
 * Simulink model (as `h_max` is) and regenerate.  Until then, verify by hand
 * that the value compiled into the model matches the wheel the simulation
 * flies -- a mismatch will not be reported by anything. */
ADCS_EXPORT int ADCS_set_rw_inertia(double wheel_inertia)
{
    (void)wheel_inertia;
    return 0;
}

/* Non-zero if this model build exposes a tunable wheel inertia.  Lets a
 * caller decide up front rather than inferring it from a failed set. */
ADCS_EXPORT int ADCS_has_rw_inertia_parameter(void)
{
    return 0;
}

ADCS_EXPORT void ADCS_set_h_max(double max_momentum)
{
    ADCS_Software_v1_P.h_max = max_momentum;
}

/* Returns 1 only if BOTH values were applied; 0 means the inertia was
 * dropped (h_max is still set either way). */
ADCS_EXPORT int ADCS_set_rw_parameters(double wheel_inertia, double max_momentum)
{
    int inertia_applied = ADCS_set_rw_inertia(wheel_inertia);
    ADCS_set_h_max(max_momentum);
    return inertia_applied;
}

ADCS_EXPORT double ADCS_get_h_max(void)
{
    return ADCS_Software_v1_P.h_max;
}

ADCS_EXPORT unsigned char ADCS_get_rw_saturated(void)
{
    return ADCS_Software_v1_B.Compare_i ? 1U : 0U;
}

/* Desaturation is complete once stored momentum falls below the exit band.
 *
 * Model version 2.53 renamed this signal.  It used to be `B.NOT` -- a NOT
 * block negating the "above the exit threshold" comparison.  The model now
 * generates the comparison in its final sense directly, as
 *
 *     B.Compare_h = (|h| <= h_max * 0.25)     '<S75>/Compare' (was <S66>)
 *
 * so it is read straight through with no inversion.  The 0.25 matches
 * DESAT_EXIT_FRACTION in adcs_sim/config/actuators.py.
 *
 * Do not assume this name survives a regeneration: Simulink derives these
 * identifiers from block structure, so restructuring `Subsystem3` will rename
 * them again and this file will stop compiling.  That build break is the
 * intended behaviour -- it is far safer than silently reading the wrong flag.
 * Check `B_ADCS_Software_v1_T` in ADCS_Software_v1.h and re-map:
 *
 *     '<S72>/Compare'  |h| >= h_max*0.95  -> saturated      (ADCS_get_rw_saturated)
 *     '<S75>/Compare'  |h| <= h_max*0.25  -> desat complete (this function)
 *
 * Model 2.125 renumbered these blocks (<S63> -> <S72>, <S66> -> <S75>) but kept
 * the names and the blocks they belong to: Subsystem1 and Subsystem3
 * 'Compare To Constant'.
 */
ADCS_EXPORT unsigned char ADCS_get_desat_complete(void)
{
    return ADCS_Software_v1_B.Compare_h ? 1U : 0U;
}

ADCS_EXPORT void ADCS_set_ads_fine_available(unsigned char available)
{
    /* Constant13 is the generated model's fine-ADS failure flag. */
    ADCS_Software_v1_P.Constant13_Value = available ? false : true;
}

/* ------------------------------------------------------------------ */
/* Camera feed                                                          */
/* ------------------------------------------------------------------ */
/* '<Root>/Camera feed' is uint8_T Camerafeed[262144] -- a 512 x 512
 * grayscale frame, one byte per pixel.
 *
 * Why these helpers exist: the Python scenario mirrors ExtU with ctypes and
 * assigns fields element by element.  That is fine for a 3-vector, but the
 * camera inport holds 262144 elements and the controller runs at 20 Hz, so
 * per-element assignment from Python is far too slow.  ADCS_set_camera_feed
 * moves the whole frame with a single memcpy.
 *
 * LAYOUT: Simulink flattens a 2-D signal in COLUMN-major order, whereas C and
 * numpy images are row-major.  A row-major HxW buffer must therefore be
 * transposed before it is passed in, or the model receives the image
 * transposed.  This function copies bytes verbatim and cannot detect that --
 * the caller owns the ordering. */

/* Number of bytes the camera inport expects.  Query this instead of hard-
 * coding 262144 on the caller side. */
ADCS_EXPORT unsigned int ADCS_get_camera_feed_size(void)
{
    return (unsigned int)ADCS_CAMERA_FEED_SIZE;
}

/* Copy one frame into the camera inport.
 *
 * image  : pointer to `length` contiguous uint8 pixels, column-major.
 * length : must equal ADCS_get_camera_feed_size().
 *
 * Returns 1 on success, 0 if the pointer is NULL or the length does not
 * match.  The length is checked rather than trusted: a short buffer would
 * otherwise read past the caller's allocation, and a long one would overrun
 * the inport. */
ADCS_EXPORT int ADCS_set_camera_feed(const unsigned char *image,
                                     unsigned int length)
{
    if ((image == NULL) || (length != (unsigned int)ADCS_CAMERA_FEED_SIZE)) {
        return 0;
    }

    memcpy(ADCS_Software_v1_U.Camerafeed, image, (size_t)ADCS_CAMERA_FEED_SIZE);
    return 1;
}

/* Blank the camera inport.  Use when no frame is available for this step, so
 * the model sees an explicit empty frame rather than the previous one held
 * over indefinitely. */
ADCS_EXPORT void ADCS_clear_camera_feed(void)
{
    memset(ADCS_Software_v1_U.Camerafeed, 0, (size_t)ADCS_CAMERA_FEED_SIZE);
}

/* Direct pointer to the camera inport, for callers that want to render or
 * decode straight into it and skip the copy entirely.  Valid for
 * ADCS_get_camera_feed_size() bytes. */
ADCS_EXPORT unsigned char *ADCS_get_camera_feed_ptr(void)
{
    return &ADCS_Software_v1_U.Camerafeed[0];
}

/* ------------------------------------------------------------------ */
/* Star tracker feed                                                    */
/* ------------------------------------------------------------------ */
/* '<Root>/STR 1' is uint8_T STR1[262144] -- a 512 x 512 grayscale star
 * tracker frame, added in model 2.125.  Same contract as the camera feed
 * above: whole-frame memcpy, COLUMN-major, length checked, caller owns the
 * ordering.
 *
 * Model 2.125 declares this inport but its step function does not read it
 * yet, so writing a frame has no effect on the control law in that build. */

ADCS_EXPORT unsigned int ADCS_get_star_tracker_feed_size(void)
{
    return (unsigned int)ADCS_STAR_TRACKER_FEED_SIZE;
}

/* Returns 1 on success, 0 if the pointer is NULL or the length is not
 * ADCS_get_star_tracker_feed_size(). */
ADCS_EXPORT int ADCS_set_star_tracker_feed(const unsigned char *image,
                                           unsigned int length)
{
    if ((image == NULL) ||
        (length != (unsigned int)ADCS_STAR_TRACKER_FEED_SIZE)) {
        return 0;
    }

    memcpy(ADCS_Software_v1_U.STR1, image, (size_t)ADCS_STAR_TRACKER_FEED_SIZE);
    return 1;
}

ADCS_EXPORT void ADCS_clear_star_tracker_feed(void)
{
    memset(ADCS_Software_v1_U.STR1, 0, (size_t)ADCS_STAR_TRACKER_FEED_SIZE);
}

ADCS_EXPORT unsigned char *ADCS_get_star_tracker_feed_ptr(void)
{
    return &ADCS_Software_v1_U.STR1[0];
}

/* ------------------------------------------------------------------ */
/* Estimated attitude                                                   */
/* ------------------------------------------------------------------ */
/* '<Root>/q_BI_est' is real_T q_BI_est[4], added in model 2.125: the flight
 * software's estimate of the body attitude relative to inertial.  Assumed to
 * follow the q_BI inport's convention (scalar first); the model does not say.
 *
 * Model 2.125 only initialises it to zero and never writes it in the step
 * function, so it reads [0 0 0 0] until the estimator is implemented.
 *
 * Copies the four components into q_out.  Returns 1 on success, 0 if q_out
 * is NULL. */
ADCS_EXPORT int ADCS_get_q_BI_est(double *q_out)
{
    if (q_out == NULL) {
        return 0;
    }

    memcpy(q_out, ADCS_Software_v1_Y.q_BI_est, sizeof(ADCS_Software_v1_Y.q_BI_est));
    return 1;
}
