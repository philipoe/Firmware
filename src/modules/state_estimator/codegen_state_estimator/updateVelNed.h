/*
 * updateVelNed.h
 *
 * Code generation for function 'updateVelNed'
 *
 * C source code generated on: Fri Jul 11 14:42:13 2014
 *
 */

#ifndef __UPDATEVELNED_H__
#define __UPDATEVELNED_H__
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
extern void updateVelNed(states_T *x, real32_T P[400], const real32_T y_VelNed[3], const real32_T R_VelNed[9], boolean_T constWind, boolean_T constQFF, boolean_T constK);
#endif
/* End of code generation (updateVelNed.h) */
