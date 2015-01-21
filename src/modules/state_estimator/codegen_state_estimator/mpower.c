/*
 * mpower.c
 *
 * Code generation for function 'mpower'
 *
 * C source code generated on: Fri Jul 11 14:42:13 2014
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "getAirplane.h"
#include "initStates.h"
#include "magField.h"
#include "propagate.h"
#include "quat2rpy.h"
#include "updateCompass2.h"
#include "updatePosition.h"
#include "updatePressures.h"
#include "updatePressures2.h"
#include "updatePressures_all.h"
#include "updateVelNed.h"
#include "mpower.h"
#include "inv.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void mpower(const real32_T a[16], real32_T c[16])
{
  real32_T b_a[16];
  real32_T b_c[16];
  memcpy(&b_a[0], &a[0], sizeof(real32_T) << 4);
  memcpy(&c[0], &b_a[0], sizeof(real32_T) << 4);
  memcpy(&b_c[0], &c[0], sizeof(real32_T) << 4);
  invNxN(b_c, c);
}

/* End of code generation (mpower.c) */
