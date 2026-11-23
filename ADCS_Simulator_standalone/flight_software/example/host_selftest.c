/*
 * host_selftest.c -- desktop/target smoke test for the standalone package
 *
 * Runs the model for a fixed input script and prints every output with full
 * precision.  Build it once on the PC and once on (or for) the target and
 * diff the two printouts: they must match to the last digit, apart from
 * rounding differences in libm functions (atan2, sqrt, ...), which show up
 * only in the trailing digits.
 *
 * Needs stdio, so it is for bring-up only, not for the flight image.
 */
#include <stdio.h>
#include <string.h>
#include "ADCS_Software_v1.h"

static void set_inputs(int k)
{
    double t = 0.05 * k;
    ExtU_ADCS_Software_v1_T *u = &ADCS_Software_v1_U;

    /* Attitude: small rotation about z growing with time. */
    u->q_BI[0] = 0.9998;
    u->q_BI[1] = 0.0;
    u->q_BI[2] = 0.0;
    u->q_BI[3] = 0.0199 + 1.0e-4 * t;

    u->omega[0] = 0.010;
    u->omega[1] = -0.005;
    u->omega[2] = 0.002;

    u->B[0] = 2.0e-5;
    u->B[1] = -1.0e-5;
    u->B[2] = 3.0e-5;

    u->RW_omega[0] = 100.0;
    u->RW_omega[1] = -50.0;
    u->RW_omega[2] = 25.0;

    u->r_sat_i[0] = 6878137.0;  u->r_sat_i[1] = 0.0;     u->r_sat_i[2] = 0.0;
    u->v_sat_i[0] = 0.0;        u->v_sat_i[1] = 7612.0;  u->v_sat_i[2] = 0.0;
    u->r_p_i[0]   = 6378137.0;  u->r_p_i[1]   = 1.0e5;   u->r_p_i[2]   = 2.0e5;
    u->r_sun_i[0] = 1.496e11;   u->r_sun_i[1] = 0.0;     u->r_sun_i[2] = 0.0;
    u->v_sun_i[0] = 0.0;        u->v_sun_i[1] = 3.0e4;   u->v_sun_i[2] = 0.0;

    /* Image inputs left dark: no blob, no stars. */
    memset(u->Camerafeed, 0, sizeof(u->Camerafeed));
    memset(u->STR1, 0, sizeof(u->STR1));
}

int main(void)
{
    static const uint8_T cmds[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    int c, k, i;

    ADCS_Software_v1_initialize();

    for (c = 0; c < (int)(sizeof(cmds) / sizeof(cmds[0])); c++) {
        ADCS_Software_v1_P.Constant15_Value = cmds[c];
        for (k = 0; k < 20; k++) {
            set_inputs(c * 20 + k);
            ADCS_Software_v1_step();
        }
        printf("cmd %u ephem %u ads %u degraded %u\n", (unsigned)cmds[c],
               (unsigned)ADCS_Software_v1_Y.actual_ephem,
               (unsigned)ADCS_Software_v1_Y.actual_ads,
               (unsigned)ADCS_Software_v1_Y.mode_degraded);
        for (i = 0; i < 3; i++) {
            printf("  RW %.17g THR %.17g MTQ %.17g\n",
                   ADCS_Software_v1_Y.RW_tau[i], ADCS_Software_v1_Y.THR_tau[i],
                   ADCS_Software_v1_Y.MTQ_tau[i]);
        }
        printf("  q_est %.17g %.17g %.17g %.17g\n",
               ADCS_Software_v1_Y.q_BI_est[0], ADCS_Software_v1_Y.q_BI_est[1],
               ADCS_Software_v1_Y.q_BI_est[2], ADCS_Software_v1_Y.q_BI_est[3]);
    }

    ADCS_Software_v1_terminate();
    printf("errorStatus: %s\n", rtmGetErrorStatus(ADCS_Software_v1_M) ?
           rtmGetErrorStatus(ADCS_Software_v1_M) : "none");
    return 0;
}
