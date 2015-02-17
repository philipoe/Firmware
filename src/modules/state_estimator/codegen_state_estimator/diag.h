/*
 * diag.h
 *
 * Code generation for function 'diag'
 *
 * C source code generated on: Fri Jan 23 17:57:26 2015
 *
 */

#ifndef __DIAG_H__
#define __DIAG_H__
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
extern void b_diag(const real32_T v[3], real32_T d[9]);
extern void c_diag(const real32_T v[7], real32_T d[49]);
extern void d_diag(const real32_T v[6], real32_T d[36]);
extern void diag(const real32_T v[3], real32_T d[9]);
#endif
/* End of code generation (diag.h) */
