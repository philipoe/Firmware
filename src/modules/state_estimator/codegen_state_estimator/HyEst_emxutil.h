/*
 * HyEst_emxutil.h
 *
 * Code generation for function 'HyEst_emxutil'
 *
 * C source code generated on: Wed May 06 13:59:17 2015
 *
 */

#ifndef __HYEST_EMXUTIL_H__
#define __HYEST_EMXUTIL_H__
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
extern void b_emxInit_real32_T(emxArray_real32_T **pEmxArray, int32_T numDimensions);
extern void emxEnsureCapacity(emxArray__common *emxArray, int32_T oldNumel, int32_T elementSize);
extern void emxFree_int32_T(emxArray_int32_T **pEmxArray);
extern void emxFree_real32_T(emxArray_real32_T **pEmxArray);
extern void emxInit_int32_T(emxArray_int32_T **pEmxArray, int32_T numDimensions);
extern void emxInit_real32_T(emxArray_real32_T **pEmxArray, int32_T numDimensions);
#endif
/* End of code generation (HyEst_emxutil.h) */
