/*
 * ADCS_Software_v1_types.h
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

#ifndef ADCS_Software_v1_types_h_
#define ADCS_Software_v1_types_h_
#include "rtwtypes.h"
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /* struct_emxArray_real_T */

#ifndef typedef_emxArray_real_T_ADCS_Software_T
#define typedef_emxArray_real_T_ADCS_Software_T

typedef struct emxArray_real_T emxArray_real_T_ADCS_Software_T;

#endif                             /* typedef_emxArray_real_T_ADCS_Software_T */

#ifndef struct_tag_x5VH0nj4JTqxL0rhBdrKDH
#define struct_tag_x5VH0nj4JTqxL0rhBdrKDH

struct tag_x5VH0nj4JTqxL0rhBdrKDH
{
  emxArray_real_T_ADCS_Software_T *f1;
};

#endif                                 /* struct_tag_x5VH0nj4JTqxL0rhBdrKDH */

#ifndef typedef_cell_wrap_0_ADCS_Software_v1_T
#define typedef_cell_wrap_0_ADCS_Software_v1_T

typedef struct tag_x5VH0nj4JTqxL0rhBdrKDH cell_wrap_0_ADCS_Software_v1_T;

#endif                              /* typedef_cell_wrap_0_ADCS_Software_v1_T */

#ifndef struct_emxArray_tag_x5VH0nj4JTqxL0rhBd
#define struct_emxArray_tag_x5VH0nj4JTqxL0rhBd

struct emxArray_tag_x5VH0nj4JTqxL0rhBd
{
  cell_wrap_0_ADCS_Software_v1_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                              /* struct_emxArray_tag_x5VH0nj4JTqxL0rhBd */

#ifndef typedef_emxArray_cell_wrap_0_ADCS_Sof_T
#define typedef_emxArray_cell_wrap_0_ADCS_Sof_T

typedef struct emxArray_tag_x5VH0nj4JTqxL0rhBd emxArray_cell_wrap_0_ADCS_Sof_T;

#endif                             /* typedef_emxArray_cell_wrap_0_ADCS_Sof_T */

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T

struct emxArray_int32_T
{
  int32_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /* struct_emxArray_int32_T */

#ifndef typedef_emxArray_int32_T_ADCS_Softwar_T
#define typedef_emxArray_int32_T_ADCS_Softwar_T

typedef struct emxArray_int32_T emxArray_int32_T_ADCS_Softwar_T;

#endif                             /* typedef_emxArray_int32_T_ADCS_Softwar_T */

#ifndef struct_emxArray_int16_T
#define struct_emxArray_int16_T

struct emxArray_int16_T
{
  int16_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /* struct_emxArray_int16_T */

#ifndef typedef_emxArray_int16_T_ADCS_Softwar_T
#define typedef_emxArray_int16_T_ADCS_Softwar_T

typedef struct emxArray_int16_T emxArray_int16_T_ADCS_Softwar_T;

#endif                             /* typedef_emxArray_int16_T_ADCS_Softwar_T */

/* Custom Type definition for MATLAB Function: '<S18>/MATLAB Function2' */
#ifndef struct_tag_zJU814XmWhn8zfVqQTFZKD
#define struct_tag_zJU814XmWhn8zfVqQTFZKD

struct tag_zJU814XmWhn8zfVqQTFZKD
{
  emxArray_real_T_ADCS_Software_T *f1;
};

#endif                                 /* struct_tag_zJU814XmWhn8zfVqQTFZKD */

#ifndef typedef_cell_wrap_7_ADCS_Software_v1_T
#define typedef_cell_wrap_7_ADCS_Software_v1_T

typedef struct tag_zJU814XmWhn8zfVqQTFZKD cell_wrap_7_ADCS_Software_v1_T;

#endif                              /* typedef_cell_wrap_7_ADCS_Software_v1_T */

/* Parameters for system: '<S62>/M_MIN_DRAG' */
typedef struct P_M_MIN_DRAG_ADCS_Software_v1_T_ P_M_MIN_DRAG_ADCS_Software_v1_T;

/* Parameters (default storage) */
typedef struct P_ADCS_Software_v1_T_ P_ADCS_Software_v1_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_ADCS_Software_v1_T RT_MODEL_ADCS_Software_v1_T;

#endif                                 /* ADCS_Software_v1_types_h_ */
