/*
 * mldivide.h
 *
 * Code generation for function 'mldivide'
 *
 * C source code generated on: Fri Jan 23 17:57:26 2015
 *
 */

#ifndef __MLDIVIDE_H__
#define __MLDIVIDE_H__
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
extern void b_mldivide(const real32_T A[16], const emxArray_real32_T *B, emxArray_real32_T *Y);
extern void mldivide(const real32_T A[9], const emxArray_real32_T *B, emxArray_real32_T *Y);
#endif
/* End of code generation (mldivide.h) */
