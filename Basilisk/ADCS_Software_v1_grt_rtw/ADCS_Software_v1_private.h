/*
 * ADCS_Software_v1_private.h
 *
 * Code generation for model "ADCS_Software_v1".
 *
 * Model version              : 2.178
 * Simulink Coder version : 26.1 (R2026a) 20-Nov-2025
 * C source code generated on : Wed Oct  7 10:46:27 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef ADCS_Software_v1_private_h_
#define ADCS_Software_v1_private_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#include "ADCS_Software_v1.h"
#include "ADCS_Software_v1_types.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmSetTFinal
#define rtmSetTFinal(rtm, val)         ((rtm)->Timing.tFinal = (val))
#endif

extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern void ADCS_Software_v1_M_MIN_DRAG(real_T rty_Out1[9],
  P_M_MIN_DRAG_ADCS_Software_v1_T *localP);

#endif                                 /* ADCS_Software_v1_private_h_ */
