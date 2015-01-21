/*
 * initStates.h
 *
 * Code generation for function 'initStates'
 *
 * C source code generated on: Fri Jul 11 14:42:13 2014
 *
 */

#ifndef __INITSTATES_H__
#define __INITSTATES_H__
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
extern void initStates(const real_T pos[3], const real32_T acc_S[3], const real32_T gyr_S[3], const real32_T v_S_rel[3], const real32_T b_S[3], const real32_T b_N[3], states_T *x, real32_T P[400]);
#endif
/* End of code generation (initStates.h) */
