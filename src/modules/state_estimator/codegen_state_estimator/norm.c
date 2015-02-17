/*
 * norm.c
 *
 * Code generation for function 'norm'
 *
 * C source code generated on: Fri Jan 23 17:57:26 2015
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
#include "norm.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
real32_T norm(const real32_T x[3])
{
  real32_T y;
  real32_T scale;
  int32_T k;
  real32_T absxk;
  real32_T t;
  y = 0.0F;
  scale = 1.17549435E-38F;
  for (k = 0; k < 3; k++) {
    absxk = (real32_T)fabs(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * (real32_T)sqrt(y);
}

/* End of code generation (norm.c) */
