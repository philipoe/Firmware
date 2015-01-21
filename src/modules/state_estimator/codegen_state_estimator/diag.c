/*
 * diag.c
 *
 * Code generation for function 'diag'
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
#include "diag.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void b_diag(const real32_T v[3], real32_T d[9])
{
  int32_T j;
  for (j = 0; j < 9; j++) {
    d[j] = 0.0F;
  }

  for (j = 0; j < 3; j++) {
    d[j + 3 * j] = v[j];
  }
}

void c_diag(const real32_T v[7], real32_T d[49])
{
  int32_T j;
  memset(&d[0], 0, 49U * sizeof(real32_T));
  for (j = 0; j < 7; j++) {
    d[j + 7 * j] = v[j];
  }
}

void d_diag(const real32_T v[6], real32_T d[36])
{
  int32_T j;
  memset(&d[0], 0, 36U * sizeof(real32_T));
  for (j = 0; j < 6; j++) {
    d[j + 6 * j] = v[j];
  }
}

void diag(const real32_T v[3], real32_T d[9])
{
  int32_T j;
  for (j = 0; j < 9; j++) {
    d[j] = 0.0F;
  }

  for (j = 0; j < 3; j++) {
    d[j + 3 * j] = v[j];
  }
}

/* End of code generation (diag.c) */
