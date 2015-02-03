/*
 * dot.c
 *
 * Code generation for function 'dot'
 *
 * C source code generated on: Fri Jan 23 17:57:25 2015
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
#include "dot.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
real32_T dot(const real32_T a[3], const real32_T b[3])
{
  real32_T c;
  int32_T ix;
  int32_T iy;
  int32_T k;
  c = 0.0F;
  ix = 0;
  iy = 0;
  for (k = 0; k < 3; k++) {
    c += a[ix] * b[iy];
    ix++;
    iy++;
  }

  return c;
}

/* End of code generation (dot.c) */
