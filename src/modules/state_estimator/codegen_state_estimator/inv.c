/*
 * inv.c
 *
 * Code generation for function 'inv'
 *
 * C source code generated on: Fri Jul 11 14:42:14 2014
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
#include "inv.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void invNxN(const real32_T x[16], real32_T y[16])
{
  real32_T A[16];
  int32_T i6;
  int8_T ipiv[4];
  int32_T j;
  int32_T c;
  int32_T pipk;
  int32_T ix;
  real32_T smax;
  int32_T k;
  real32_T s;
  int32_T jy;
  int32_T ijA;
  int8_T p[4];
  for (i6 = 0; i6 < 16; i6++) {
    y[i6] = 0.0F;
    A[i6] = x[i6];
  }

  for (i6 = 0; i6 < 4; i6++) {
    ipiv[i6] = (int8_T)(1 + i6);
  }

  for (j = 0; j < 3; j++) {
    c = j * 5;
    pipk = 0;
    ix = c;
    smax = (real32_T)fabs(A[c]);
    for (k = 2; k <= 4 - j; k++) {
      ix++;
      s = (real32_T)fabs(A[ix]);
      if (s > smax) {
        pipk = k - 1;
        smax = s;
      }
    }

    if (A[c + pipk] != 0.0F) {
      if (pipk != 0) {
        ipiv[j] = (int8_T)((j + pipk) + 1);
        ix = j;
        pipk += j;
        for (k = 0; k < 4; k++) {
          smax = A[ix];
          A[ix] = A[pipk];
          A[pipk] = smax;
          ix += 4;
          pipk += 4;
        }
      }

      i6 = (c - j) + 4;
      for (jy = c + 1; jy + 1 <= i6; jy++) {
        A[jy] /= A[c];
      }
    }

    pipk = c;
    jy = c + 4;
    for (k = 1; k <= 3 - j; k++) {
      smax = A[jy];
      if (A[jy] != 0.0F) {
        ix = c + 1;
        i6 = (pipk - j) + 8;
        for (ijA = 5 + pipk; ijA + 1 <= i6; ijA++) {
          A[ijA] += A[ix] * -smax;
          ix++;
        }
      }

      jy += 4;
      pipk += 4;
    }
  }

  for (i6 = 0; i6 < 4; i6++) {
    p[i6] = (int8_T)(1 + i6);
  }

  for (k = 0; k < 3; k++) {
    if (ipiv[k] > 1 + k) {
      pipk = p[ipiv[k] - 1];
      p[ipiv[k] - 1] = p[k];
      p[k] = (int8_T)pipk;
    }
  }

  for (k = 0; k < 4; k++) {
    y[k + ((p[k] - 1) << 2)] = 1.0F;
    for (j = k; j + 1 < 5; j++) {
      if (y[j + ((p[k] - 1) << 2)] != 0.0F) {
        for (jy = j + 1; jy + 1 < 5; jy++) {
          y[jy + ((p[k] - 1) << 2)] -= y[j + ((p[k] - 1) << 2)] * A[jy + (j << 2)];
        }
      }
    }
  }

  for (j = 0; j < 4; j++) {
    c = j << 2;
    for (k = 3; k > -1; k += -1) {
      pipk = k << 2;
      if (y[k + c] != 0.0F) {
        y[k + c] /= A[k + pipk];
        for (jy = 0; jy + 1 <= k; jy++) {
          y[jy + c] -= y[k + c] * A[jy + pipk];
        }
      }
    }
  }
}

/* End of code generation (inv.c) */
