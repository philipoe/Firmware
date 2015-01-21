/*
 * updatePosition.h
 *
 * Code generation for function 'updatePosition'
 *
 * C source code generated on: Fri Jul 11 14:42:13 2014
 *
 */

#ifndef __UPDATEPOSITION_H__
#define __UPDATEPOSITION_H__
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
extern void eml_li_find(const boolean_T x[20], emxArray_int32_T *y);
extern void updatePosition(states_T *x, real32_T P[400], const real_T y_GpsPos_rad[3], const real32_T R_GpsPos_m[9], boolean_T constWind, boolean_T constQFF, boolean_T constK);
#endif
/* End of code generation (updatePosition.h) */
