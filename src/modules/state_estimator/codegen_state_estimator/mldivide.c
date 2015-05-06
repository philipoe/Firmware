/*
 * mldivide.c
 *
 * Code generation for function 'mldivide'
 *
 * C source code generated on: Wed May 06 16:15:52 2015
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
#include "mldivide.h"
#include "HyEst_emxutil.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static void b_eml_lusolve(const real32_T A[16], const emxArray_real32_T *B,
  emxArray_real32_T *X);
static void eml_lusolve(const real32_T A[9], const emxArray_real32_T *B,
  emxArray_real32_T *X);

/* Function Definitions */
static void b_eml_lusolve(const real32_T A[16], const emxArray_real32_T *B,
  emxArray_real32_T *X)
{
  real32_T b_A[16];
  int8_T ipiv[4];
  int32_T i5;
  int32_T j;
  int32_T c;
  int32_T iy;
  int32_T ix;
  real32_T temp;
  int32_T k;
  real32_T s;
  int32_T jy;
  int32_T ijA;
  memcpy(&b_A[0], &A[0], sizeof(real32_T) << 4);
  for (i5 = 0; i5 < 4; i5++) {
    ipiv[i5] = (int8_T)(1 + i5);
  }

  for (j = 0; j < 3; j++) {
    c = j * 5;
    iy = 0;
    ix = c;
    temp = (real32_T)fabs(b_A[c]);
    for (k = 2; k <= 4 - j; k++) {
      ix++;
      s = (real32_T)fabs(b_A[ix]);
      if (s > temp) {
        iy = k - 1;
        temp = s;
      }
    }

    if (b_A[c + iy] != 0.0F) {
      if (iy != 0) {
        ipiv[j] = (int8_T)((j + iy) + 1);
        ix = j;
        iy += j;
        for (k = 0; k < 4; k++) {
          temp = b_A[ix];
          b_A[ix] = b_A[iy];
          b_A[iy] = temp;
          ix += 4;
          iy += 4;
        }
      }

      i5 = (c - j) + 4;
      for (jy = c + 1; jy + 1 <= i5; jy++) {
        b_A[jy] /= b_A[c];
      }
    }

    iy = c;
    jy = c + 4;
    for (k = 1; k <= 3 - j; k++) {
      temp = b_A[jy];
      if (b_A[jy] != 0.0F) {
        ix = c + 1;
        i5 = (iy - j) + 8;
        for (ijA = 5 + iy; ijA + 1 <= i5; ijA++) {
          b_A[ijA] += b_A[ix] * -temp;
          ix++;
        }
      }

      jy += 4;
      iy += 4;
    }
  }

  i5 = X->size[0] * X->size[1];
  X->size[0] = 4;
  X->size[1] = B->size[1];
  emxEnsureCapacity((emxArray__common *)X, i5, (int32_T)sizeof(real32_T));
  iy = B->size[0] * B->size[1];
  for (i5 = 0; i5 < iy; i5++) {
    X->data[i5] = B->data[i5];
  }

  for (jy = 0; jy < 4; jy++) {
    if (ipiv[jy] != jy + 1) {
      for (j = 0; j + 1 <= B->size[1]; j++) {
        temp = X->data[jy + X->size[0] * j];
        X->data[jy + X->size[0] * j] = X->data[(ipiv[jy] + X->size[0] * j) - 1];
        X->data[(ipiv[jy] + X->size[0] * j) - 1] = temp;
      }
    }
  }

  if ((B->size[1] == 0) || (X->size[1] == 0)) {
  } else {
    for (j = 1; j <= B->size[1]; j++) {
      c = (j - 1) << 2;
      for (k = 0; k < 4; k++) {
        iy = k << 2;
        if (X->data[k + c] != 0.0F) {
          for (jy = k + 2; jy < 5; jy++) {
            X->data[(jy + c) - 1] -= X->data[k + c] * b_A[(jy + iy) - 1];
          }
        }
      }
    }
  }

  if ((B->size[1] == 0) || (X->size[1] == 0)) {
  } else {
    for (j = 1; j <= B->size[1]; j++) {
      c = (j - 1) << 2;
      for (k = 3; k > -1; k += -1) {
        iy = k << 2;
        if (X->data[k + c] != 0.0F) {
          X->data[k + c] /= b_A[k + iy];
          for (jy = 0; jy + 1 <= k; jy++) {
            X->data[jy + c] -= X->data[k + c] * b_A[jy + iy];
          }
        }
      }
    }
  }
}

static void eml_lusolve(const real32_T A[9], const emxArray_real32_T *B,
  emxArray_real32_T *X)
{
  real32_T b_A[9];
  int32_T i2;
  int8_T ipiv[3];
  int32_T j;
  int32_T c;
  int32_T iy;
  int32_T ix;
  real32_T temp;
  int32_T k;
  real32_T s;
  int32_T jy;
  int32_T ijA;
  for (i2 = 0; i2 < 9; i2++) {
    b_A[i2] = A[i2];
  }

  for (i2 = 0; i2 < 3; i2++) {
    ipiv[i2] = (int8_T)(1 + i2);
  }

  for (j = 0; j < 2; j++) {
    c = j << 2;
    iy = 0;
    ix = c;
    temp = (real32_T)fabs(b_A[c]);
    for (k = 2; k <= 3 - j; k++) {
      ix++;
      s = (real32_T)fabs(b_A[ix]);
      if (s > temp) {
        iy = k - 1;
        temp = s;
      }
    }

    if (b_A[c + iy] != 0.0F) {
      if (iy != 0) {
        ipiv[j] = (int8_T)((j + iy) + 1);
        ix = j;
        iy += j;
        for (k = 0; k < 3; k++) {
          temp = b_A[ix];
          b_A[ix] = b_A[iy];
          b_A[iy] = temp;
          ix += 3;
          iy += 3;
        }
      }

      i2 = (c - j) + 3;
      for (jy = c + 1; jy + 1 <= i2; jy++) {
        b_A[jy] /= b_A[c];
      }
    }

    iy = c;
    jy = c + 3;
    for (k = 1; k <= 2 - j; k++) {
      temp = b_A[jy];
      if (b_A[jy] != 0.0F) {
        ix = c + 1;
        i2 = (iy - j) + 6;
        for (ijA = 4 + iy; ijA + 1 <= i2; ijA++) {
          b_A[ijA] += b_A[ix] * -temp;
          ix++;
        }
      }

      jy += 3;
      iy += 3;
    }
  }

  i2 = X->size[0] * X->size[1];
  X->size[0] = 3;
  X->size[1] = B->size[1];
  emxEnsureCapacity((emxArray__common *)X, i2, (int32_T)sizeof(real32_T));
  iy = B->size[0] * B->size[1];
  for (i2 = 0; i2 < iy; i2++) {
    X->data[i2] = B->data[i2];
  }

  for (jy = 0; jy < 3; jy++) {
    if (ipiv[jy] != jy + 1) {
      for (j = 0; j + 1 <= B->size[1]; j++) {
        temp = X->data[jy + X->size[0] * j];
        X->data[jy + X->size[0] * j] = X->data[(ipiv[jy] + X->size[0] * j) - 1];
        X->data[(ipiv[jy] + X->size[0] * j) - 1] = temp;
      }
    }
  }

  if ((B->size[1] == 0) || (X->size[1] == 0)) {
  } else {
    for (j = 1; j <= B->size[1]; j++) {
      c = 3 * (j - 1);
      for (k = 0; k < 3; k++) {
        iy = 3 * k;
        if (X->data[k + c] != 0.0F) {
          for (jy = k + 2; jy < 4; jy++) {
            X->data[(jy + c) - 1] -= X->data[k + c] * b_A[(jy + iy) - 1];
          }
        }
      }
    }
  }

  if ((B->size[1] == 0) || (X->size[1] == 0)) {
  } else {
    for (j = 1; j <= B->size[1]; j++) {
      c = 3 * (j - 1);
      for (k = 2; k > -1; k += -1) {
        iy = 3 * k;
        if (X->data[k + c] != 0.0F) {
          X->data[k + c] /= b_A[k + iy];
          for (jy = 0; jy + 1 <= k; jy++) {
            X->data[jy + c] -= X->data[k + c] * b_A[jy + iy];
          }
        }
      }
    }
  }
}

void b_mldivide(const real32_T A[16], const emxArray_real32_T *B,
                emxArray_real32_T *Y)
{
  int32_T i4;
  if (B->size[1] == 0) {
    i4 = Y->size[0] * Y->size[1];
    Y->size[0] = 4;
    Y->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)Y, i4, (int32_T)sizeof(real32_T));
  } else {
    b_eml_lusolve(A, B, Y);
  }
}

void mldivide(const real32_T A[9], const emxArray_real32_T *B, emxArray_real32_T
              *Y)
{
  int32_T i1;
  if (B->size[1] == 0) {
    i1 = Y->size[0] * Y->size[1];
    Y->size[0] = 3;
    Y->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)Y, i1, (int32_T)sizeof(real32_T));
  } else {
    eml_lusolve(A, B, Y);
  }
}

/* End of code generation (mldivide.c) */
