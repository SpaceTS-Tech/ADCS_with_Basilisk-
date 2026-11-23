/*
 * main_baremetal.c -- integration template for the flight processor
 *
 * Replaces MATLAB's rt_main.c (the grt desktop main, which needs the MATLAB
 * install and stdio).  The model is single-rate: call
 * ADCS_Software_v1_step() once every 0.05 s (20 Hz) from a timer interrupt
 * or an RTOS periodic task.
 *
 * The three board_* hooks are placeholders: fill them in with the OBC's
 * sensor drivers, actuator drivers and timer.
 */
#include "ADCS_Software_v1.h"

#define ADCS_STEP_PERIOD_S 0.05 /* must equal ADCS_Software_v1_M->Timing.stepSize0 */

/* ---- board hooks: implement these for the target ---------------------- */
extern void board_wait_for_tick(void);          /* blocks until the next 50 ms tick */
extern void board_read_sensors(ExtU_ADCS_Software_v1_T *u);
extern void board_write_actuators(const ExtY_ADCS_Software_v1_T *y);

/* Mission mode command, 0..9 (Constant15 in the model). */
void adcs_set_mission_cmd(uint8_T cmd)
{
    ADCS_Software_v1_P.Constant15_Value = cmd;
}

int main(void)
{
    ADCS_Software_v1_initialize();

    /* Tunable parameters.  The values compiled into ADCS_Software_v1.c are
     * NOT the ones that were tested.  These are the values the Basilisk
     * test campaign flew (simulator/adcs_sim/config/gains.py and
     * actuators.py, 2026-10).  Confirm them against the flight release. */
    I_RW        = 0.0109; /* wheel spin inertia [kg m^2]   (compiled: 1.0)  */
    k_desat     = 0.1;    /* desaturation gain             (compiled: 1.0)  */
    kp_point    = 1.0;    /*                               (compiled: 2.0)  */
    kd_point    = 20.0;   /*                               (compiled: 20.0) */
    kp_spin     = 1.0;    /*                               (compiled: 1.0)  */
    kd_spin     = 20.0;   /*                               (compiled: 0.0)  */
    kd_detumble = 1.0;    /*                               (compiled: 1.0)  */
    ADCS_Software_v1_P.h_max = 4.0; /* wheel momentum capacity [N m s] (compiled: 1.47) */
    adcs_set_mission_cmd(6U);       /* compiled default */

    for (;;) {
        board_wait_for_tick();

        board_read_sensors(&ADCS_Software_v1_U);
        ADCS_Software_v1_step();
        board_write_actuators(&ADCS_Software_v1_Y);

        if (rtmGetErrorStatus(ADCS_Software_v1_M) != (NULL)) {
            break; /* model reported an error -- go to safe mode */
        }
    }

    ADCS_Software_v1_terminate();
    return 0;
}
