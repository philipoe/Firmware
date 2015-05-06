/*
 * updatePressures.h
 *
 * Code generation for function 'updatePressures'
 *
 * C source code generated on: Wed May 06 13:59:17 2015
 *
 */

#ifndef __UPDATEPRESSURES_H__
#define __UPDATEPRESSURES_H__
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
extern boolean_T b_updatePressures(states_T *x, real32_T P[400], real32_T y_p_stat, real32_T T_h, real32_T ell_offset, boolean_T constQFF, boolean_T constK);
extern boolean_T updatePressures(states_T *x, real32_T P[400], real32_T y_p_stat, real32_T T_h, real32_T ell_offset, boolean_T constWind, boolean_T constQFF, boolean_T constK);
#endif
/* End of code generation (updatePressures.h) */
