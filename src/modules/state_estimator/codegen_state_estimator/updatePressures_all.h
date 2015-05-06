/*
 * updatePressures_all.h
 *
 * Code generation for function 'updatePressures_all'
 *
 * C source code generated on: Wed May 06 16:15:52 2015
 *
 */

#ifndef __UPDATEPRESSURES_ALL_H__
#define __UPDATEPRESSURES_ALL_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"

#include "rtwtypes.h"
#include "HyEst_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void updatePressures_all(states_T *x, real32_T P[400], real32_T y_p_stat, real32_T y_p_dyn, real32_T T_h, real32_T ell_offset, const real32_T acc[3], const airplane_T airplane, boolean_T constWind, boolean_T constQFF, boolean_T constK, boolean_T *p_stat_valid, boolean_T *on_ground, boolean_T *aero_valid);
#endif
/* End of code generation (updatePressures_all.h) */
