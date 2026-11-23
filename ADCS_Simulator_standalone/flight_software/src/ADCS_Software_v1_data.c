/*
 * ADCS_Software_v1_data.c
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

#include "ADCS_Software_v1.h"

/* Block parameters (default storage) */
P_ADCS_Software_v1_T ADCS_Software_v1_P = {
  /* Variable: h_max
   * Referenced by:
   *   '<S70>/Constant'
   *   '<S73>/Constant'
   */
  1.47,

  /* Mask Parameter: CompareToConstant_const
   * Referenced by: '<S71>/Constant'
   */
  0.010000000000000002,

  /* Mask Parameter: CompareToConstant1_const
   * Referenced by: '<S72>/Constant'
   */
  0.0001,

  /* Expression: [0 0 0]
   * Referenced by: '<S23>/Constant'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: [0 0 0]
   * Referenced by: '<S14>/Constant'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: [0 0 1]
   * Referenced by: '<S21>/Constant'
   */
  { 0.0, 0.0, 1.0 },

  /* Expression: [1 1 1]
   * Referenced by: '<S21>/Constant1'
   */
  { 1.0, 1.0, 1.0 },

  /* Expression: 0.04
   * Referenced by: '<S21>/Gain2'
   */
  0.04,

  /* Expression: 0
   * Referenced by: '<S18>/Constant1'
   */
  0.0,

  /* Expression: -1
   * Referenced by: '<S18>/Gain3'
   */
  -1.0,

  /* Expression: [0 0 0]
   * Referenced by: '<S20>/Constant'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: [0 0 0]
   * Referenced by: '<S17>/Constant'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: -1
   * Referenced by: '<S10>/q_err sign flip'
   */
  -1.0,

  /* Expression: eps(2)
   * Referenced by: '<S41>/Direction Cosine Matrix  to Quaternions'
   */
  4.4408920985006262E-16,

  /* Expression: -1
   * Referenced by: '<S39>/Gain'
   */
  -1.0,

  /* Expression: eps(2)
   * Referenced by: '<S39>/Direction Cosine Matrix  to Quaternions'
   */
  4.4408920985006262E-16,

  /* Expression: [0 0 7.2921159e-5]
   * Referenced by: '<S56>/omega_E'
   */
  { 0.0, 0.0, 7.2921159E-5 },

  /* Expression: eps(2)
   * Referenced by: '<S42>/Direction Cosine Matrix  to Quaternions'
   */
  4.4408920985006262E-16,

  /* Expression: [0 0 1; 0 1 0; -1 0 0]
   * Referenced by: '<S67>/Constant'
   */
  { 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0 },

  /* Expression: [0 0 1; 0 1 0; -1 0 0]
   * Referenced by: '<S69>/Constant1'
   */
  { 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0 },

  /* Expression: eps(2)
   * Referenced by: '<S43>/Direction Cosine Matrix  to Quaternions'
   */
  4.4408920985006262E-16,

  /* Expression: [0; 0; 0]
   * Referenced by: '<S44>/Constant'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: [1 0 0 0]
   * Referenced by: '<S44>/Constant1'
   */
  { 1.0, 0.0, 0.0, 0.0 },

  /* Expression: 0
   * Referenced by: '<S10>/Shortest q error'
   */
  0.0,

  /* Expression: -1
   * Referenced by: '<S12>/Gain'
   */
  -1.0,

  /* Expression: 0
   * Referenced by: '<S9>/Gain'
   */
  0.0,

  /* Expression: nan
   * Referenced by: '<Root>/Delay1'
   */
  -NAN,

  /* Computed Parameter: Constant9_Value
   * Referenced by: '<Root>/Constant9'
   */
  true,

  /* Computed Parameter: Constant3_Value
   * Referenced by: '<Root>/Constant3'
   */
  true,

  /* Computed Parameter: Constant14_Value
   * Referenced by: '<Root>/Constant14'
   */
  false,

  /* Computed Parameter: Constant4_Value
   * Referenced by: '<Root>/Constant4'
   */
  true,

  /* Computed Parameter: Constant13_Value
   * Referenced by: '<Root>/Constant13'
   */
  true,

  /* Computed Parameter: Constant5_Value
   * Referenced by: '<Root>/Constant5'
   */
  true,

  /* Computed Parameter: Constant12_Value
   * Referenced by: '<Root>/Constant12'
   */
  false,

  /* Computed Parameter: Constant6_Value
   * Referenced by: '<Root>/Constant6'
   */
  true,

  /* Computed Parameter: Constant11_Value
   * Referenced by: '<Root>/Constant11'
   */
  true,

  /* Computed Parameter: Constant7_Value
   * Referenced by: '<Root>/Constant7'
   */
  true,

  /* Computed Parameter: Constant10_Value
   * Referenced by: '<Root>/Constant10'
   */
  true,

  /* Computed Parameter: Constant8_Value
   * Referenced by: '<Root>/Constant8'
   */
  true,

  /* Computed Parameter: Constant15_Value
   * Referenced by: '<Root>/Constant15'
   */
  6U,

  /* Start of '<S62>/Default' */
  {
    /* Expression: [1 0 0; 0 1 0; 0 0 1]
     * Referenced by: '<S66>/Constant'
     */
    { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 }
  }
  ,

  /* End of '<S62>/Default' */

  /* Start of '<S62>/M_MIN_DRAG' */
  {
    /* Expression: [1 0 0; 0 1 0; 0 0 1]
     * Referenced by: '<S68>/Constant'
     */
    { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 }
  }
  /* End of '<S62>/M_MIN_DRAG' */
};
