/*
 * updateCompass2.h
 *
 * Code generation for function 'updateCompass2'
 *
 * C source code generated on: Wed May 06 16:15:52 2015
 *
 */

#ifndef __UPDATECOMPASS2_H__
#define __UPDATECOMPASS2_H__
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
extern void updateCompass2(states_T *x, real32_T P[400], const real32_T y_Compass[3], const real32_T b_N[3], const real32_T acc[3], boolean_T constWind, boolean_T constQFF, boolean_T constK);
#endif
/* End of code generation (updateCompass2.h) */
