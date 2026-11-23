/*
 * ADCS_Software_v1.h
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

#ifndef ADCS_Software_v1_h_
#define ADCS_Software_v1_h_
#ifndef ADCS_Software_v1_COMMON_INCLUDES_
#define ADCS_Software_v1_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* ADCS_Software_v1_COMMON_INCLUDES_ */

#include "ADCS_Software_v1_types.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include <float.h>
#include <string.h>
#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                (&(rtm)->Timing.taskTime0)
#endif

/* Block signals (default storage) */
typedef struct {
  int32_T stack[1048576];
  boolean_T visited[1048576];
  boolean_T x[262144];
  real_T found[15000];
  real_T predicted[3306];
  uint8_T desired_ephem;               /* '<S32>/Mission_Mode_Manager' */
  uint8_T desired_ads;                 /* '<S32>/Mission_Mode_Manager' */
  uint8_T desired_frame;               /* '<S32>/Mission_Mode_Manager' */
  uint8_T desired_acs;                 /* '<S32>/Mission_Mode_Manager' */
  uint8_T actual_frame;                /* '<S32>/Frame_Manager' */
  uint8_T actual_acs;                  /* '<S32>/ACS_Manager' */
  boolean_T Compare;                   /* '<S71>/Compare' */
  boolean_T Compare_i;                 /* '<S70>/Compare' */
  boolean_T Compare_h;                 /* '<S73>/Compare' */
} B_ADCS_Software_v1_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Delay1_DSTATE[4];             /* '<Root>/Delay1' */
  int8_T SwitchCase_ActiveSubsystem;   /* '<S5>/Switch Case' */
  uint8_T is_active_c9_ADCS_Software_v1;/* '<S4>/Status_Manager' */
  uint8_T is_active_c3_ADCS_Software_v1;/* '<S32>/Mission_Mode_Manager' */
  uint8_T is_c3_ADCS_Software_v1;      /* '<S32>/Mission_Mode_Manager' */
  uint8_T is_active_c4_ADCS_Software_v1;/* '<S32>/Frame_Manager' */
  uint8_T is_c4_ADCS_Software_v1;      /* '<S32>/Frame_Manager' */
  uint8_T is_active_c1_ADCS_Software_v1;/* '<S32>/Ephemeris_Manager' */
  uint8_T is_c1_ADCS_Software_v1;      /* '<S32>/Ephemeris_Manager' */
  uint8_T is_active_c2_ADCS_Software_v1;/* '<S32>/ADS_Manager' */
  uint8_T is_c2_ADCS_Software_v1;      /* '<S32>/ADS_Manager' */
  uint8_T is_active_c5_ADCS_Software_v1;/* '<S32>/ACS_Manager' */
  uint8_T is_c5_ADCS_Software_v1;      /* '<S32>/ACS_Manager' */
} DW_ADCS_Software_v1_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T q_BI[4];                      /* '<Root>/q_BI' */
  real_T omega[3];                     /* '<Root>/omega' */
  real_T B[3];                         /* '<Root>/B' */
  real_T RW_omega[3];                  /* '<Root>/RW_omega' */
  real_T v_sat_i[3];                   /* '<Root>/v_sat_i' */
  real_T r_sat_i[3];                   /* '<Root>/r_sat_i' */
  real_T r_p_i[3];                     /* '<Root>/r_p_i' */
  real_T v_sun_i[3];                   /* '<Root>/v_sun_i' */
  real_T r_sun_i[3];                   /* '<Root>/r_sun_i' */
  uint8_T Camerafeed[262144];          /* '<Root>/Camera feed' */
  uint8_T STR1[1048576];               /* '<Root>/STR 1' */
} ExtU_ADCS_Software_v1_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  uint8_T actual_ephem;                /* '<Root>/actual_ephem' */
  uint8_T actual_ads;                  /* '<Root>/actual_ads' */
  boolean_T mode_degraded;             /* '<Root>/mode_degraded' */
  real_T RW_tau[3];                    /* '<Root>/RW_tau' */
  real_T THR_tau[3];                   /* '<Root>/THR_tau' */
  real_T MTQ_tau[3];                   /* '<Root>/MTQ_tau' */
  real_T q_BI_est[4];                  /* '<Root>/q_BI_est' */
} ExtY_ADCS_Software_v1_T;

/* Parameters for system: '<S62>/M_MIN_DRAG' */
struct P_M_MIN_DRAG_ADCS_Software_v1_T_ {
  real_T Constant_Value[9];            /* Expression: [1 0 0; 0 1 0; 0 0 1]
                                        * Referenced by: '<S68>/Constant'
                                        */
};

/* Parameters (default storage) */
struct P_ADCS_Software_v1_T_ {
  real_T h_max;                        /* Variable: h_max
                                        * Referenced by:
                                        *   '<S70>/Constant'
                                        *   '<S73>/Constant'
                                        */
  real_T CompareToConstant_const;     /* Mask Parameter: CompareToConstant_const
                                       * Referenced by: '<S71>/Constant'
                                       */
  real_T CompareToConstant1_const;   /* Mask Parameter: CompareToConstant1_const
                                      * Referenced by: '<S72>/Constant'
                                      */
  real_T Constant_Value[3];            /* Expression: [0 0 0]
                                        * Referenced by: '<S23>/Constant'
                                        */
  real_T Constant_Value_g[3];          /* Expression: [0 0 0]
                                        * Referenced by: '<S14>/Constant'
                                        */
  real_T Constant_Value_j[3];          /* Expression: [0 0 1]
                                        * Referenced by: '<S21>/Constant'
                                        */
  real_T Constant1_Value[3];           /* Expression: [1 1 1]
                                        * Referenced by: '<S21>/Constant1'
                                        */
  real_T Gain2_Gain;                   /* Expression: 0.04
                                        * Referenced by: '<S21>/Gain2'
                                        */
  real_T Constant1_Value_a;            /* Expression: 0
                                        * Referenced by: '<S18>/Constant1'
                                        */
  real_T Gain3_Gain;                   /* Expression: -1
                                        * Referenced by: '<S18>/Gain3'
                                        */
  real_T Constant_Value_b[3];          /* Expression: [0 0 0]
                                        * Referenced by: '<S20>/Constant'
                                        */
  real_T Constant_Value_f[3];          /* Expression: [0 0 0]
                                        * Referenced by: '<S17>/Constant'
                                        */
  real_T q_errsignflip_Gain;           /* Expression: -1
                                        * Referenced by: '<S10>/q_err sign flip'
                                        */
  real_T DirectionCosineMatrixtoQuaterni;/* Expression: eps(2)
                                          * Referenced by: '<S41>/Direction Cosine Matrix  to Quaternions'
                                          */
  real_T Gain_Gain;                    /* Expression: -1
                                        * Referenced by: '<S39>/Gain'
                                        */
  real_T DirectionCosineMatrixtoQuater_i;/* Expression: eps(2)
                                          * Referenced by: '<S39>/Direction Cosine Matrix  to Quaternions'
                                          */
  real_T omega_E_Value[3];             /* Expression: [0 0 7.2921159e-5]
                                        * Referenced by: '<S56>/omega_E'
                                        */
  real_T DirectionCosineMatrixtoQuater_b;/* Expression: eps(2)
                                          * Referenced by: '<S42>/Direction Cosine Matrix  to Quaternions'
                                          */
  real_T Constant_Value_c[9];          /* Expression: [0 0 1; 0 1 0; -1 0 0]
                                        * Referenced by: '<S67>/Constant'
                                        */
  real_T Constant1_Value_c[9];         /* Expression: [0 0 1; 0 1 0; -1 0 0]
                                        * Referenced by: '<S69>/Constant1'
                                        */
  real_T DirectionCosineMatrixtoQuater_e;/* Expression: eps(2)
                                          * Referenced by: '<S43>/Direction Cosine Matrix  to Quaternions'
                                          */
  real_T Constant_Value_i[3];          /* Expression: [0; 0; 0]
                                        * Referenced by: '<S44>/Constant'
                                        */
  real_T Constant1_Value_o[4];         /* Expression: [1 0 0 0]
                                        * Referenced by: '<S44>/Constant1'
                                        */
  real_T Shortestqerror_Threshold;     /* Expression: 0
                                        * Referenced by: '<S10>/Shortest q error'
                                        */
  real_T Gain_Gain_p;                  /* Expression: -1
                                        * Referenced by: '<S12>/Gain'
                                        */
  real_T Gain_Gain_n;                  /* Expression: 0
                                        * Referenced by: '<S9>/Gain'
                                        */
  real_T Delay1_InitialCondition;      /* Expression: nan
                                        * Referenced by: '<Root>/Delay1'
                                        */
  boolean_T Constant9_Value;           /* Computed Parameter: Constant9_Value
                                        * Referenced by: '<Root>/Constant9'
                                        */
  boolean_T Constant3_Value;           /* Computed Parameter: Constant3_Value
                                        * Referenced by: '<Root>/Constant3'
                                        */
  boolean_T Constant14_Value;          /* Computed Parameter: Constant14_Value
                                        * Referenced by: '<Root>/Constant14'
                                        */
  boolean_T Constant4_Value;           /* Computed Parameter: Constant4_Value
                                        * Referenced by: '<Root>/Constant4'
                                        */
  boolean_T Constant13_Value;          /* Computed Parameter: Constant13_Value
                                        * Referenced by: '<Root>/Constant13'
                                        */
  boolean_T Constant5_Value;           /* Computed Parameter: Constant5_Value
                                        * Referenced by: '<Root>/Constant5'
                                        */
  boolean_T Constant12_Value;          /* Computed Parameter: Constant12_Value
                                        * Referenced by: '<Root>/Constant12'
                                        */
  boolean_T Constant6_Value;           /* Computed Parameter: Constant6_Value
                                        * Referenced by: '<Root>/Constant6'
                                        */
  boolean_T Constant11_Value;          /* Computed Parameter: Constant11_Value
                                        * Referenced by: '<Root>/Constant11'
                                        */
  boolean_T Constant7_Value;           /* Computed Parameter: Constant7_Value
                                        * Referenced by: '<Root>/Constant7'
                                        */
  boolean_T Constant10_Value;          /* Computed Parameter: Constant10_Value
                                        * Referenced by: '<Root>/Constant10'
                                        */
  boolean_T Constant8_Value;           /* Computed Parameter: Constant8_Value
                                        * Referenced by: '<Root>/Constant8'
                                        */
  uint8_T Constant15_Value;            /* Computed Parameter: Constant15_Value
                                        * Referenced by: '<Root>/Constant15'
                                        */
  P_M_MIN_DRAG_ADCS_Software_v1_T Default;/* '<S62>/Default' */
  P_M_MIN_DRAG_ADCS_Software_v1_T M_MIN_DRAG;/* '<S62>/M_MIN_DRAG' */
};

/* Real-time Model Data Structure */
struct tag_RTM_ADCS_Software_v1_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block parameters (default storage) */
extern P_ADCS_Software_v1_T ADCS_Software_v1_P;

/* Block signals (default storage) */
extern B_ADCS_Software_v1_T ADCS_Software_v1_B;

/* Block states (default storage) */
extern DW_ADCS_Software_v1_T ADCS_Software_v1_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_ADCS_Software_v1_T ADCS_Software_v1_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_ADCS_Software_v1_T ADCS_Software_v1_Y;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern real_T I_RW;                    /* Variable: I_RW
                                        * Referenced by:
                                        *   '<S6>/Gain'
                                        *   '<S8>/Gain'
                                        */
extern real_T k_desat;                 /* Variable: k_desat
                                        * Referenced by:
                                        *   '<S11>/Gain'
                                        *   '<S24>/Gain2'
                                        */
extern real_T kd_detumble;             /* Variable: kd_detumble
                                        * Referenced by: '<S25>/Gain'
                                        */
extern real_T kd_point;                /* Variable: kd_point
                                        * Referenced by:
                                        *   '<S16>/Gain1'
                                        *   '<S18>/Gain1'
                                        *   '<S19>/Gain'
                                        */
extern real_T kd_spin;                 /* Variable: kd_spin
                                        * Referenced by: '<S21>/Gain'
                                        */
extern real_T kp_point;                /* Variable: kp_point
                                        * Referenced by:
                                        *   '<S18>/Gain2'
                                        *   '<S19>/Gain1'
                                        */
extern real_T kp_spin;                 /* Variable: kp_spin
                                        * Referenced by: '<S21>/Gain1'
                                        */

/* Model entry point functions */
extern void ADCS_Software_v1_initialize(void);
extern void ADCS_Software_v1_step(void);
extern void ADCS_Software_v1_terminate(void);

/* Real-time Model object */
extern RT_MODEL_ADCS_Software_v1_T *const ADCS_Software_v1_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'ADCS_Software_v1'
 * '<S1>'   : 'ADCS_Software_v1/MATLAB Function'
 * '<S2>'   : 'ADCS_Software_v1/Subsystem3'
 * '<S3>'   : 'ADCS_Software_v1/Subsystem3/Controller'
 * '<S4>'   : 'ADCS_Software_v1/Subsystem3/Mode manager'
 * '<S5>'   : 'ADCS_Software_v1/Subsystem3/Subsystem'
 * '<S6>'   : 'ADCS_Software_v1/Subsystem3/Subsystem1'
 * '<S7>'   : 'ADCS_Software_v1/Subsystem3/Subsystem2'
 * '<S8>'   : 'ADCS_Software_v1/Subsystem3/Subsystem3'
 * '<S9>'   : 'ADCS_Software_v1/Subsystem3/Controller/Control Algorithms'
 * '<S10>'  : 'ADCS_Software_v1/Subsystem3/Controller/error '
 * '<S11>'  : 'ADCS_Software_v1/Subsystem3/Controller/Control Algorithms/DESAT'
 * '<S12>'  : 'ADCS_Software_v1/Subsystem3/Controller/Control Algorithms/RW Torque Commands'
 * '<S13>'  : 'ADCS_Software_v1/Subsystem3/Controller/Control Algorithms/Thruster Torque commands'
 * '<S14>'  : 'ADCS_Software_v1/Subsystem3/Controller/Control Algorithms/default'
 * '<S15>'  : 'ADCS_Software_v1/Subsystem3/Controller/Control Algorithms/DESAT/Cross Product'
 * '<S16>'  : 'ADCS_Software_v1/Subsystem3/Controller/Control Algorithms/RW Torque Commands/DESAT'
 * '<S17>'  : 'ADCS_Software_v1/Subsystem3/Controller/Control Algorithms/RW Torque Commands/IDLE'
 * '<S18>'  : 'ADCS_Software_v1/Subsystem3/Controller/Control Algorithms/RW Torque Commands/OPTICAL'
 * '<S19>'  : 'ADCS_Software_v1/Subsystem3/Controller/Control Algorithms/RW Torque Commands/POINT'
 * '<S20>'  : 'ADCS_Software_v1/Subsystem3/Controller/Control Algorithms/RW Torque Commands/SAFE'
 * '<S21>'  : 'ADCS_Software_v1/Subsystem3/Controller/Control Algorithms/RW Torque Commands/SPIN'
 * '<S22>'  : 'ADCS_Software_v1/Subsystem3/Controller/Control Algorithms/RW Torque Commands/OPTICAL/MATLAB Function2'
 * '<S23>'  : 'ADCS_Software_v1/Subsystem3/Controller/Control Algorithms/Thruster Torque commands/DEFAULT'
 * '<S24>'  : 'ADCS_Software_v1/Subsystem3/Controller/Control Algorithms/Thruster Torque commands/DESAT'
 * '<S25>'  : 'ADCS_Software_v1/Subsystem3/Controller/Control Algorithms/Thruster Torque commands/DETUMBLE'
 * '<S26>'  : 'ADCS_Software_v1/Subsystem3/Controller/error /Quaternion Conjugate1'
 * '<S27>'  : 'ADCS_Software_v1/Subsystem3/Controller/error /Quaternion Multiplication'
 * '<S28>'  : 'ADCS_Software_v1/Subsystem3/Controller/error /Quaternion Multiplication/q0'
 * '<S29>'  : 'ADCS_Software_v1/Subsystem3/Controller/error /Quaternion Multiplication/q1'
 * '<S30>'  : 'ADCS_Software_v1/Subsystem3/Controller/error /Quaternion Multiplication/q2'
 * '<S31>'  : 'ADCS_Software_v1/Subsystem3/Controller/error /Quaternion Multiplication/q3'
 * '<S32>'  : 'ADCS_Software_v1/Subsystem3/Mode manager/Mode Manager'
 * '<S33>'  : 'ADCS_Software_v1/Subsystem3/Mode manager/Status_Manager'
 * '<S34>'  : 'ADCS_Software_v1/Subsystem3/Mode manager/Mode Manager/ACS_Manager'
 * '<S35>'  : 'ADCS_Software_v1/Subsystem3/Mode manager/Mode Manager/ADS_Manager'
 * '<S36>'  : 'ADCS_Software_v1/Subsystem3/Mode manager/Mode Manager/Ephemeris_Manager'
 * '<S37>'  : 'ADCS_Software_v1/Subsystem3/Mode manager/Mode Manager/Frame_Manager'
 * '<S38>'  : 'ADCS_Software_v1/Subsystem3/Mode manager/Mode Manager/Mission_Mode_Manager'
 * '<S39>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/LVLH'
 * '<S40>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/Subsystem'
 * '<S41>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/Sun_point'
 * '<S42>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/Target_Pointing'
 * '<S43>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/VNB'
 * '<S44>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/default'
 * '<S45>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/LVLH/Cross Product'
 * '<S46>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/LVLH/Cross Product1'
 * '<S47>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/LVLH/norm'
 * '<S48>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/LVLH/norm1'
 * '<S49>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/Sun_point/Cross Product'
 * '<S50>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/Sun_point/Cross Product1'
 * '<S51>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/Sun_point/Pointing_r'
 * '<S52>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/Sun_point/norm'
 * '<S53>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/Sun_point/norm1'
 * '<S54>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/Target_Pointing/Cross Product'
 * '<S55>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/Target_Pointing/Cross Product1'
 * '<S56>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/Target_Pointing/Pointing_r'
 * '<S57>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/Target_Pointing/norm'
 * '<S58>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/Target_Pointing/norm1'
 * '<S59>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/Target_Pointing/Pointing_r/v_p'
 * '<S60>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/VNB/Cross Product'
 * '<S61>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/VNB/Cross Product1'
 * '<S62>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/VNB/Subsystem'
 * '<S63>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/VNB/norm'
 * '<S64>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/VNB/norm2'
 * '<S65>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/VNB/norm3'
 * '<S66>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/VNB/Subsystem/Default'
 * '<S67>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/VNB/Subsystem/M_MAX_DRAG'
 * '<S68>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/VNB/Subsystem/M_MIN_DRAG'
 * '<S69>'  : 'ADCS_Software_v1/Subsystem3/Subsystem/VNB/Subsystem/M_RAM'
 * '<S70>'  : 'ADCS_Software_v1/Subsystem3/Subsystem1/Compare To Constant'
 * '<S71>'  : 'ADCS_Software_v1/Subsystem3/Subsystem2/Compare To Constant'
 * '<S72>'  : 'ADCS_Software_v1/Subsystem3/Subsystem2/Compare To Constant1'
 * '<S73>'  : 'ADCS_Software_v1/Subsystem3/Subsystem3/Compare To Constant'
 */
#endif                                 /* ADCS_Software_v1_h_ */
