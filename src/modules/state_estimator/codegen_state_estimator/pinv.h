/*
 * pinv.h
 *
 * Code generation for function 'pinv'
 *
 * C source code generated on: Fri Jan 23 17:57:26 2015
 *
 */

#ifndef __PINV_H__
#define __PINV_H__
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
extern void b_pinv(const real32_T A[16], real32_T X[16]);
extern void pinv(const real32_T A[9], real32_T X[9]);
#endif
/* End of code generation (pinv.h) */
