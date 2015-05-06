/*
 * propagate.h
 *
 * Code generation for function 'propagate'
 *
 * C source code generated on: Wed May 06 13:59:16 2015
 *
 */

#ifndef __PROPAGATE_H__
#define __PROPAGATE_H__
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
extern void propagate(states_T *x, real32_T P[400], const real32_T y_indirect[6], real32_T dt, boolean_T const_wind, const real32_T wind[2]);
#endif
/* End of code generation (propagate.h) */
