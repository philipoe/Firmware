/*
 * pinv.c
 *
 * Code generation for function 'pinv'
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
#include "pinv.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static real32_T b_eml_div(real_T x, real32_T y);
static void b_eml_xaxpy(int32_T n, real32_T a, const real32_T x[9], int32_T ix0,
  real32_T y[3], int32_T iy0);
static real32_T b_eml_xdotc(int32_T n, const real32_T x[16], int32_T ix0, const
  real32_T y[16], int32_T iy0);
static void b_eml_xgesvd(const real32_T A[16], real32_T U[16], real32_T S[4],
  real32_T V[16]);
static real32_T b_eml_xnrm2(int32_T n, const real32_T x[3], int32_T ix0);
static void b_eml_xrot(real32_T x[16], int32_T ix0, int32_T iy0, real32_T c,
  real32_T s);
static void b_eml_xscal(int32_T n, real32_T a, real32_T x[3], int32_T ix0);
static void b_eml_xswap(real32_T x[16], int32_T ix0, int32_T iy0);
static real32_T c_eml_div(real32_T x, real_T y);
static void c_eml_xaxpy(int32_T n, real32_T a, const real32_T x[3], int32_T ix0,
  real32_T y[9], int32_T iy0);
static real32_T c_eml_xnrm2(int32_T n, const real32_T x[16], int32_T ix0);
static void c_eml_xscal(int32_T n, real32_T a, real32_T x[16], int32_T ix0);
static void d_eml_xaxpy(int32_T n, real32_T a, int32_T ix0, real32_T y[16],
  int32_T iy0);
static real32_T d_eml_xnrm2(int32_T n, const real32_T x[4], int32_T ix0);
static void d_eml_xscal(int32_T n, real32_T a, real32_T x[4], int32_T ix0);
static void e_eml_xaxpy(int32_T n, real32_T a, const real32_T x[16], int32_T ix0,
  real32_T y[4], int32_T iy0);
static real32_T eml_div(real32_T x, real32_T y);
static void eml_xaxpy(int32_T n, real32_T a, int32_T ix0, real32_T y[9], int32_T
                      iy0);
static real32_T eml_xdotc(int32_T n, const real32_T x[9], int32_T ix0, const
  real32_T y[9], int32_T iy0);
static void eml_xgesvd(const real32_T A[9], real32_T U[9], real32_T S[3],
  real32_T V[9]);
static real32_T eml_xnrm2(int32_T n, const real32_T x[9], int32_T ix0);
static void eml_xrot(real32_T x[9], int32_T ix0, int32_T iy0, real32_T c,
                     real32_T s);
static void eml_xrotg(real32_T *a, real32_T *b, real32_T *c, real32_T *s);
static void eml_xscal(int32_T n, real32_T a, real32_T x[9], int32_T ix0);
static void eml_xswap(real32_T x[9], int32_T ix0, int32_T iy0);
static void f_eml_xaxpy(int32_T n, real32_T a, const real32_T x[4], int32_T ix0,
  real32_T y[16], int32_T iy0);

/* Function Definitions */
static real32_T b_eml_div(real_T x, real32_T y)
{
  return (real32_T)x / y;
}

static void b_eml_xaxpy(int32_T n, real32_T a, const real32_T x[9], int32_T ix0,
  real32_T y[3], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

static real32_T b_eml_xdotc(int32_T n, const real32_T x[16], int32_T ix0, const
  real32_T y[16], int32_T iy0)
{
  real32_T d;
  int32_T ix;
  int32_T iy;
  int32_T k;
  d = 0.0F;
  if (n < 1) {
  } else {
    ix = ix0;
    iy = iy0;
    for (k = 1; k <= n; k++) {
      d += x[ix - 1] * y[iy - 1];
      ix++;
      iy++;
    }
  }

  return d;
}

static void b_eml_xgesvd(const real32_T A[16], real32_T U[16], real32_T S[4],
  real32_T V[16])
{
  real32_T b_A[16];
  real32_T s[4];
  real32_T e[4];
  real32_T work[4];
  int32_T i;
  real32_T Vf[16];
  int32_T q;
  int32_T qs;
  real32_T ztest0;
  int32_T ii;
  int32_T m;
  real32_T rt;
  real32_T ztest;
  int32_T iter;
  real32_T tiny;
  real32_T snorm;
  int32_T exitg3;
  boolean_T exitg2;
  real32_T sn;
  real32_T varargin_1[5];
  boolean_T exitg1;
  real32_T sqds;
  real32_T b;
  memcpy(&b_A[0], &A[0], sizeof(real32_T) << 4);
  for (i = 0; i < 4; i++) {
    s[i] = 0.0F;
    e[i] = 0.0F;
    work[i] = 0.0F;
  }

  for (i = 0; i < 16; i++) {
    U[i] = 0.0F;
    Vf[i] = 0.0F;
  }

  for (q = 0; q < 3; q++) {
    qs = q + (q << 2);
    ztest0 = c_eml_xnrm2(4 - q, b_A, qs + 1);
    if (ztest0 > 0.0F) {
      if (b_A[qs] < 0.0F) {
        s[q] = -ztest0;
      } else {
        s[q] = ztest0;
      }

      c_eml_xscal(4 - q, b_eml_div(1.0, s[q]), b_A, qs + 1);
      b_A[qs]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0F;
    }

    for (ii = q + 1; ii + 1 < 5; ii++) {
      i = q + (ii << 2);
      if (s[q] != 0.0F) {
        ztest0 = -eml_div(b_eml_xdotc(4 - q, b_A, qs + 1, b_A, i + 1), b_A[q +
                          (q << 2)]);
        d_eml_xaxpy(4 - q, ztest0, qs + 1, b_A, i + 1);
      }

      e[ii] = b_A[i];
    }

    for (ii = q; ii + 1 < 5; ii++) {
      U[ii + (q << 2)] = b_A[ii + (q << 2)];
    }

    if (q + 1 <= 2) {
      ztest0 = d_eml_xnrm2(3 - q, e, q + 2);
      if (ztest0 == 0.0F) {
        e[q] = 0.0F;
      } else {
        if (e[q + 1] < 0.0F) {
          e[q] = -ztest0;
        } else {
          e[q] = ztest0;
        }

        ztest0 = b_eml_div(1.0, e[q]);
        d_eml_xscal(3 - q, ztest0, e, q + 2);
        e[q + 1]++;
      }

      e[q] = -e[q];
      if (e[q] != 0.0F) {
        for (ii = q + 1; ii + 1 < 5; ii++) {
          work[ii] = 0.0F;
        }

        for (ii = q + 1; ii + 1 < 5; ii++) {
          e_eml_xaxpy(3 - q, e[ii], b_A, (q + (ii << 2)) + 2, work, q + 2);
        }

        for (ii = q + 1; ii + 1 < 5; ii++) {
          f_eml_xaxpy(3 - q, eml_div(-e[ii], e[q + 1]), work, q + 2, b_A, (q +
            (ii << 2)) + 2);
        }
      }

      for (ii = q + 1; ii + 1 < 5; ii++) {
        Vf[ii + (q << 2)] = e[ii];
      }
    }
  }

  m = 2;
  s[3] = b_A[15];
  e[2] = b_A[14];
  e[3] = 0.0F;
  for (ii = 0; ii < 4; ii++) {
    U[12 + ii] = 0.0F;
  }

  U[15] = 1.0F;
  for (q = 2; q > -1; q += -1) {
    qs = q + (q << 2);
    if (s[q] != 0.0F) {
      for (ii = q + 1; ii + 1 < 5; ii++) {
        i = (q + (ii << 2)) + 1;
        ztest0 = -eml_div(b_eml_xdotc(4 - q, U, qs + 1, U, i), U[qs]);
        d_eml_xaxpy(4 - q, ztest0, qs + 1, U, i);
      }

      for (ii = q; ii + 1 < 5; ii++) {
        U[ii + (q << 2)] = -U[ii + (q << 2)];
      }

      U[qs]++;
      for (ii = 1; ii <= q; ii++) {
        U[(ii + (q << 2)) - 1] = 0.0F;
      }
    } else {
      for (ii = 0; ii < 4; ii++) {
        U[ii + (q << 2)] = 0.0F;
      }

      U[qs] = 1.0F;
    }
  }

  for (q = 3; q > -1; q += -1) {
    if ((q + 1 <= 2) && (e[q] != 0.0F)) {
      i = (q + (q << 2)) + 2;
      for (ii = q + 1; ii + 1 < 5; ii++) {
        qs = (q + (ii << 2)) + 2;
        ztest0 = -eml_div(b_eml_xdotc(3 - q, Vf, i, Vf, qs), Vf[i - 1]);
        d_eml_xaxpy(3 - q, ztest0, i, Vf, qs);
      }
    }

    for (ii = 0; ii < 4; ii++) {
      Vf[ii + (q << 2)] = 0.0F;
    }

    Vf[q + (q << 2)] = 1.0F;
  }

  for (q = 0; q < 4; q++) {
    ztest0 = e[q];
    if (s[q] != 0.0F) {
      rt = (real32_T)fabs(s[q]);
      ztest = eml_div(s[q], rt);
      s[q] = rt;
      if (q + 1 < 4) {
        ztest0 = eml_div(e[q], ztest);
      }

      c_eml_xscal(4, ztest, U, (q << 2) + 1);
    }

    if ((q + 1 < 4) && (ztest0 != 0.0F)) {
      rt = (real32_T)fabs(ztest0);
      ztest = eml_div(rt, ztest0);
      ztest0 = rt;
      s[q + 1] *= ztest;
      c_eml_xscal(4, ztest, Vf, ((q + 1) << 2) + 1);
    }

    e[q] = ztest0;
  }

  iter = 0;
  tiny = eml_div(1.17549435E-38F, 1.1920929E-7F);
  snorm = 0.0F;
  for (ii = 0; ii < 4; ii++) {
    ztest0 = (real32_T)fabs(s[ii]);
    ztest = (real32_T)fabs(e[ii]);
    if ((ztest0 >= ztest) || rtIsNaNF(ztest)) {
    } else {
      ztest0 = ztest;
    }

    if ((snorm >= ztest0) || rtIsNaNF(ztest0)) {
    } else {
      snorm = ztest0;
    }
  }

  while ((m + 2 > 0) && (!(iter >= 75))) {
    ii = m;
    do {
      exitg3 = 0;
      q = ii + 1;
      if (ii + 1 == 0) {
        exitg3 = 1;
      } else {
        ztest0 = (real32_T)fabs(e[ii]);
        if ((ztest0 <= 1.1920929E-7F * ((real32_T)fabs(s[ii]) + (real32_T)fabs
              (s[ii + 1]))) || (ztest0 <= tiny) || ((iter > 20) && (ztest0 <=
              1.1920929E-7F * snorm))) {
          e[ii] = 0.0F;
          exitg3 = 1;
        } else {
          ii--;
        }
      }
    } while (exitg3 == 0);

    if (ii + 1 == m + 1) {
      i = 4;
    } else {
      qs = m + 2;
      i = m + 2;
      exitg2 = FALSE;
      while ((exitg2 == FALSE) && (i >= ii + 1)) {
        qs = i;
        if (i == ii + 1) {
          exitg2 = TRUE;
        } else {
          ztest0 = 0.0F;
          if (i < m + 2) {
            ztest0 = (real32_T)fabs(e[i - 1]);
          }

          if (i > ii + 2) {
            ztest0 += (real32_T)fabs(e[i - 2]);
          }

          ztest = (real32_T)fabs(s[i - 1]);
          if ((ztest <= 1.1920929E-7F * ztest0) || (ztest <= tiny)) {
            s[i - 1] = 0.0F;
            exitg2 = TRUE;
          } else {
            i--;
          }
        }
      }

      if (qs == ii + 1) {
        i = 3;
      } else if (qs == m + 2) {
        i = 1;
      } else {
        i = 2;
        q = qs;
      }
    }

    switch (i) {
     case 1:
      ztest = e[m];
      e[m] = 0.0F;
      for (qs = m; qs + 1 >= q + 1; qs--) {
        ztest0 = s[qs];
        eml_xrotg(&ztest0, &ztest, &rt, &sn);
        s[qs] = ztest0;
        if (qs + 1 > q + 1) {
          ztest = -sn * e[qs - 1];
          e[qs - 1] *= rt;
        }

        b_eml_xrot(Vf, (qs << 2) + 1, ((m + 1) << 2) + 1, rt, sn);
      }
      break;

     case 2:
      ztest = e[q - 1];
      e[q - 1] = 0.0F;
      for (qs = q; qs + 1 <= m + 2; qs++) {
        eml_xrotg(&s[qs], &ztest, &rt, &sn);
        ztest = -sn * e[qs];
        e[qs] *= rt;
        b_eml_xrot(U, (qs << 2) + 1, ((q - 1) << 2) + 1, rt, sn);
      }
      break;

     case 3:
      varargin_1[0] = (real32_T)fabs(s[m + 1]);
      varargin_1[1] = (real32_T)fabs(s[m]);
      varargin_1[2] = (real32_T)fabs(e[m]);
      varargin_1[3] = (real32_T)fabs(s[q]);
      varargin_1[4] = (real32_T)fabs(e[q]);
      i = 1;
      sn = varargin_1[0];
      if (rtIsNaNF(varargin_1[0])) {
        qs = 2;
        exitg1 = FALSE;
        while ((exitg1 == FALSE) && (qs < 6)) {
          i = qs;
          if (!rtIsNaNF(varargin_1[qs - 1])) {
            sn = varargin_1[qs - 1];
            exitg1 = TRUE;
          } else {
            qs++;
          }
        }
      }

      if (i < 5) {
        while (i + 1 < 6) {
          if (varargin_1[i] > sn) {
            sn = varargin_1[i];
          }

          i++;
        }
      }

      rt = eml_div(s[m + 1], sn);
      ztest0 = eml_div(s[m], sn);
      ztest = eml_div(e[m], sn);
      sqds = eml_div(s[q], sn);
      b = c_eml_div((ztest0 + rt) * (ztest0 - rt) + ztest * ztest, 2.0);
      ztest0 = rt * ztest;
      ztest0 *= ztest0;
      ztest = 0.0F;
      if ((b != 0.0F) || (ztest0 != 0.0F)) {
        ztest = (real32_T)sqrt(b * b + ztest0);
        if (b < 0.0F) {
          ztest = -ztest;
        }

        ztest = eml_div(ztest0, b + ztest);
      }

      ztest += (sqds + rt) * (sqds - rt);
      ztest0 = sqds * eml_div(e[q], sn);
      for (qs = q + 1; qs <= m + 1; qs++) {
        eml_xrotg(&ztest, &ztest0, &rt, &sn);
        if (qs > q + 1) {
          e[qs - 2] = ztest;
        }

        ztest0 = rt * s[qs - 1];
        ztest = sn * e[qs - 1];
        e[qs - 1] = rt * e[qs - 1] - sn * s[qs - 1];
        b = s[qs];
        s[qs] *= rt;
        b_eml_xrot(Vf, ((qs - 1) << 2) + 1, (qs << 2) + 1, rt, sn);
        s[qs - 1] = ztest0 + ztest;
        ztest0 = sn * b;
        eml_xrotg(&s[qs - 1], &ztest0, &rt, &sn);
        ztest = rt * e[qs - 1] + sn * s[qs];
        s[qs] = -sn * e[qs - 1] + rt * s[qs];
        ztest0 = sn * e[qs];
        e[qs] *= rt;
        b_eml_xrot(U, ((qs - 1) << 2) + 1, (qs << 2) + 1, rt, sn);
      }

      e[m] = ztest;
      iter++;
      break;

     default:
      if (s[q] < 0.0F) {
        s[q] = -s[q];
        c_eml_xscal(4, -1.0F, Vf, (q << 2) + 1);
      }

      i = q + 1;
      while ((q + 1 < 4) && (s[q] < s[i])) {
        rt = s[q];
        s[q] = s[i];
        s[i] = rt;
        b_eml_xswap(Vf, (q << 2) + 1, ((q + 1) << 2) + 1);
        b_eml_xswap(U, (q << 2) + 1, ((q + 1) << 2) + 1);
        q = i;
        i++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  for (qs = 0; qs < 4; qs++) {
    S[qs] = s[qs];
    for (i = 0; i < 4; i++) {
      V[i + (qs << 2)] = Vf[i + (qs << 2)];
    }
  }
}

static real32_T b_eml_xnrm2(int32_T n, const real32_T x[3], int32_T ix0)
{
  real32_T y;
  real32_T scale;
  int32_T kend;
  int32_T k;
  real32_T absxk;
  real32_T t;
  y = 0.0F;
  if (n < 1) {
  } else if (n == 1) {
    y = (real32_T)fabs(x[ix0 - 1]);
  } else {
    scale = 1.17549435E-38F;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = (real32_T)fabs(x[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0F + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * (real32_T)sqrt(y);
  }

  return y;
}

static void b_eml_xrot(real32_T x[16], int32_T ix0, int32_T iy0, real32_T c,
  real32_T s)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  real32_T y;
  real32_T b_y;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 4; k++) {
    y = c * x[ix];
    b_y = s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = y + b_y;
    iy++;
    ix++;
  }
}

static void b_eml_xscal(int32_T n, real32_T a, real32_T x[3], int32_T ix0)
{
  int32_T i12;
  int32_T k;
  i12 = (ix0 + n) - 1;
  for (k = ix0; k <= i12; k++) {
    x[k - 1] *= a;
  }
}

static void b_eml_xswap(real32_T x[16], int32_T ix0, int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  real32_T temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 4; k++) {
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
  }
}

static real32_T c_eml_div(real32_T x, real_T y)
{
  return x / (real32_T)y;
}

static void c_eml_xaxpy(int32_T n, real32_T a, const real32_T x[3], int32_T ix0,
  real32_T y[9], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

static real32_T c_eml_xnrm2(int32_T n, const real32_T x[16], int32_T ix0)
{
  real32_T y;
  real32_T scale;
  int32_T kend;
  int32_T k;
  real32_T absxk;
  real32_T t;
  y = 0.0F;
  if (n < 1) {
  } else if (n == 1) {
    y = (real32_T)fabs(x[ix0 - 1]);
  } else {
    scale = 1.17549435E-38F;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = (real32_T)fabs(x[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0F + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * (real32_T)sqrt(y);
  }

  return y;
}

static void c_eml_xscal(int32_T n, real32_T a, real32_T x[16], int32_T ix0)
{
  int32_T i21;
  int32_T k;
  i21 = (ix0 + n) - 1;
  for (k = ix0; k <= i21; k++) {
    x[k - 1] *= a;
  }
}

static void d_eml_xaxpy(int32_T n, real32_T a, int32_T ix0, real32_T y[16],
  int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

static real32_T d_eml_xnrm2(int32_T n, const real32_T x[4], int32_T ix0)
{
  real32_T y;
  real32_T scale;
  int32_T kend;
  int32_T k;
  real32_T absxk;
  real32_T t;
  y = 0.0F;
  if (n < 1) {
  } else if (n == 1) {
    y = (real32_T)fabs(x[ix0 - 1]);
  } else {
    scale = 1.17549435E-38F;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = (real32_T)fabs(x[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0F + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * (real32_T)sqrt(y);
  }

  return y;
}

static void d_eml_xscal(int32_T n, real32_T a, real32_T x[4], int32_T ix0)
{
  int32_T i22;
  int32_T k;
  i22 = (ix0 + n) - 1;
  for (k = ix0; k <= i22; k++) {
    x[k - 1] *= a;
  }
}

static void e_eml_xaxpy(int32_T n, real32_T a, const real32_T x[16], int32_T ix0,
  real32_T y[4], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

static real32_T eml_div(real32_T x, real32_T y)
{
  return x / y;
}

static void eml_xaxpy(int32_T n, real32_T a, int32_T ix0, real32_T y[9], int32_T
                      iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

static real32_T eml_xdotc(int32_T n, const real32_T x[9], int32_T ix0, const
  real32_T y[9], int32_T iy0)
{
  real32_T d;
  int32_T ix;
  int32_T iy;
  int32_T k;
  d = 0.0F;
  if (n < 1) {
  } else {
    ix = ix0;
    iy = iy0;
    for (k = 1; k <= n; k++) {
      d += x[ix - 1] * y[iy - 1];
      ix++;
      iy++;
    }
  }

  return d;
}

static void eml_xgesvd(const real32_T A[9], real32_T U[9], real32_T S[3],
  real32_T V[9])
{
  real32_T b_A[9];
  int32_T i;
  real32_T s[3];
  real32_T e[3];
  real32_T work[3];
  real32_T Vf[9];
  int32_T q;
  int32_T qs;
  real32_T ztest0;
  int32_T ii;
  real32_T ztest;
  int32_T m;
  real32_T rt;
  int32_T iter;
  real32_T tiny;
  real32_T snorm;
  int32_T exitg3;
  boolean_T exitg2;
  real32_T sn;
  real32_T varargin_1[5];
  boolean_T exitg1;
  real32_T sqds;
  real32_T b;
  for (i = 0; i < 9; i++) {
    b_A[i] = A[i];
  }

  for (i = 0; i < 3; i++) {
    s[i] = 0.0F;
    e[i] = 0.0F;
    work[i] = 0.0F;
  }

  for (i = 0; i < 9; i++) {
    U[i] = 0.0F;
    Vf[i] = 0.0F;
  }

  for (q = 0; q < 2; q++) {
    qs = q + 3 * q;
    ztest0 = eml_xnrm2(3 - q, b_A, qs + 1);
    if (ztest0 > 0.0F) {
      if (b_A[qs] < 0.0F) {
        s[q] = -ztest0;
      } else {
        s[q] = ztest0;
      }

      eml_xscal(3 - q, b_eml_div(1.0, s[q]), b_A, qs + 1);
      b_A[qs]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0F;
    }

    for (ii = q + 1; ii + 1 < 4; ii++) {
      i = q + 3 * ii;
      if (s[q] != 0.0F) {
        ztest0 = -eml_div(eml_xdotc(3 - q, b_A, qs + 1, b_A, i + 1), b_A[q + 3 *
                          q]);
        eml_xaxpy(3 - q, ztest0, qs + 1, b_A, i + 1);
      }

      e[ii] = b_A[i];
    }

    for (ii = q; ii + 1 < 4; ii++) {
      U[ii + 3 * q] = b_A[ii + 3 * q];
    }

    if (q + 1 <= 1) {
      ztest0 = b_eml_xnrm2(2, e, 2);
      if (ztest0 == 0.0F) {
        e[0] = 0.0F;
      } else {
        if (e[1] < 0.0F) {
          ztest = -ztest0;
        } else {
          ztest = ztest0;
        }

        if (e[1] < 0.0F) {
          e[0] = -ztest0;
        } else {
          e[0] = ztest0;
        }

        b_eml_xscal(2, b_eml_div(1.0, ztest), e, 2);
        e[1]++;
      }

      e[0] = -e[0];
      if (e[0] != 0.0F) {
        for (ii = 2; ii < 4; ii++) {
          work[ii - 1] = 0.0F;
        }

        for (ii = 1; ii + 1 < 4; ii++) {
          b_eml_xaxpy(2, e[ii], b_A, 2 + 3 * ii, work, 2);
        }

        for (ii = 1; ii + 1 < 4; ii++) {
          c_eml_xaxpy(2, eml_div(-e[ii], e[1]), work, 2, b_A, 2 + 3 * ii);
        }
      }

      for (ii = 1; ii + 1 < 4; ii++) {
        Vf[ii] = e[ii];
      }
    }
  }

  m = 1;
  s[2] = b_A[8];
  e[1] = b_A[7];
  e[2] = 0.0F;
  for (ii = 0; ii < 3; ii++) {
    U[6 + ii] = 0.0F;
  }

  U[8] = 1.0F;
  for (q = 1; q > -1; q += -1) {
    qs = q + 3 * q;
    if (s[q] != 0.0F) {
      for (ii = q + 1; ii + 1 < 4; ii++) {
        i = (q + 3 * ii) + 1;
        ztest0 = -eml_div(eml_xdotc(3 - q, U, qs + 1, U, i), U[qs]);
        eml_xaxpy(3 - q, ztest0, qs + 1, U, i);
      }

      for (ii = q; ii + 1 < 4; ii++) {
        U[ii + 3 * q] = -U[ii + 3 * q];
      }

      U[qs]++;
      ii = 1;
      while (ii <= q) {
        U[3] = 0.0F;
        ii = 2;
      }
    } else {
      for (ii = 0; ii < 3; ii++) {
        U[ii + 3 * q] = 0.0F;
      }

      U[qs] = 1.0F;
    }
  }

  for (q = 2; q > -1; q += -1) {
    if ((q + 1 <= 1) && (e[0] != 0.0F)) {
      for (ii = 2; ii < 4; ii++) {
        i = 2 + 3 * (ii - 1);
        ztest0 = -eml_div(eml_xdotc(2, Vf, 2, Vf, i), Vf[1]);
        eml_xaxpy(2, ztest0, 2, Vf, i);
      }
    }

    for (ii = 0; ii < 3; ii++) {
      Vf[ii + 3 * q] = 0.0F;
    }

    Vf[q + 3 * q] = 1.0F;
  }

  for (q = 0; q < 3; q++) {
    ztest0 = e[q];
    if (s[q] != 0.0F) {
      rt = (real32_T)fabs(s[q]);
      ztest = eml_div(s[q], rt);
      s[q] = rt;
      if (q + 1 < 3) {
        ztest0 = eml_div(e[q], ztest);
      }

      eml_xscal(3, ztest, U, 3 * q + 1);
    }

    if ((q + 1 < 3) && (ztest0 != 0.0F)) {
      rt = (real32_T)fabs(ztest0);
      ztest = eml_div(rt, ztest0);
      ztest0 = rt;
      s[q + 1] *= ztest;
      eml_xscal(3, ztest, Vf, 3 * (q + 1) + 1);
    }

    e[q] = ztest0;
  }

  iter = 0;
  tiny = eml_div(1.17549435E-38F, 1.1920929E-7F);
  snorm = 0.0F;
  for (ii = 0; ii < 3; ii++) {
    ztest0 = (real32_T)fabs(s[ii]);
    ztest = (real32_T)fabs(e[ii]);
    if ((ztest0 >= ztest) || rtIsNaNF(ztest)) {
      ztest = ztest0;
    }

    if ((snorm >= ztest) || rtIsNaNF(ztest)) {
    } else {
      snorm = ztest;
    }
  }

  while ((m + 2 > 0) && (!(iter >= 75))) {
    ii = m;
    do {
      exitg3 = 0;
      q = ii + 1;
      if (ii + 1 == 0) {
        exitg3 = 1;
      } else {
        ztest0 = (real32_T)fabs(e[ii]);
        if ((ztest0 <= 1.1920929E-7F * ((real32_T)fabs(s[ii]) + (real32_T)fabs
              (s[ii + 1]))) || (ztest0 <= tiny) || ((iter > 20) && (ztest0 <=
              1.1920929E-7F * snorm))) {
          e[ii] = 0.0F;
          exitg3 = 1;
        } else {
          ii--;
        }
      }
    } while (exitg3 == 0);

    if (ii + 1 == m + 1) {
      i = 4;
    } else {
      qs = m + 2;
      i = m + 2;
      exitg2 = FALSE;
      while ((exitg2 == FALSE) && (i >= ii + 1)) {
        qs = i;
        if (i == ii + 1) {
          exitg2 = TRUE;
        } else {
          ztest0 = 0.0F;
          if (i < m + 2) {
            ztest0 = (real32_T)fabs(e[i - 1]);
          }

          if (i > ii + 2) {
            ztest0 += (real32_T)fabs(e[i - 2]);
          }

          ztest = (real32_T)fabs(s[i - 1]);
          if ((ztest <= 1.1920929E-7F * ztest0) || (ztest <= tiny)) {
            s[i - 1] = 0.0F;
            exitg2 = TRUE;
          } else {
            i--;
          }
        }
      }

      if (qs == ii + 1) {
        i = 3;
      } else if (qs == m + 2) {
        i = 1;
      } else {
        i = 2;
        q = qs;
      }
    }

    switch (i) {
     case 1:
      ztest = e[m];
      e[m] = 0.0F;
      for (ii = m; ii + 1 >= q + 1; ii--) {
        ztest0 = s[ii];
        eml_xrotg(&ztest0, &ztest, &rt, &sn);
        s[ii] = ztest0;
        if (ii + 1 > q + 1) {
          ztest = -sn * e[0];
          e[0] *= rt;
        }

        eml_xrot(Vf, 3 * ii + 1, 3 * (m + 1) + 1, rt, sn);
      }
      break;

     case 2:
      ztest = e[q - 1];
      e[q - 1] = 0.0F;
      for (ii = q; ii + 1 <= m + 2; ii++) {
        eml_xrotg(&s[ii], &ztest, &rt, &sn);
        ztest = -sn * e[ii];
        e[ii] *= rt;
        eml_xrot(U, 3 * ii + 1, 3 * (q - 1) + 1, rt, sn);
      }
      break;

     case 3:
      varargin_1[0] = (real32_T)fabs(s[m + 1]);
      varargin_1[1] = (real32_T)fabs(s[m]);
      varargin_1[2] = (real32_T)fabs(e[m]);
      varargin_1[3] = (real32_T)fabs(s[q]);
      varargin_1[4] = (real32_T)fabs(e[q]);
      i = 1;
      sn = varargin_1[0];
      if (rtIsNaNF(varargin_1[0])) {
        ii = 2;
        exitg1 = FALSE;
        while ((exitg1 == FALSE) && (ii < 6)) {
          i = ii;
          if (!rtIsNaNF(varargin_1[ii - 1])) {
            sn = varargin_1[ii - 1];
            exitg1 = TRUE;
          } else {
            ii++;
          }
        }
      }

      if (i < 5) {
        while (i + 1 < 6) {
          if (varargin_1[i] > sn) {
            sn = varargin_1[i];
          }

          i++;
        }
      }

      rt = eml_div(s[m + 1], sn);
      ztest0 = eml_div(s[m], sn);
      ztest = eml_div(e[m], sn);
      sqds = eml_div(s[q], sn);
      b = c_eml_div((ztest0 + rt) * (ztest0 - rt) + ztest * ztest, 2.0);
      ztest0 = rt * ztest;
      ztest0 *= ztest0;
      ztest = 0.0F;
      if ((b != 0.0F) || (ztest0 != 0.0F)) {
        ztest = (real32_T)sqrt(b * b + ztest0);
        if (b < 0.0F) {
          ztest = -ztest;
        }

        ztest = eml_div(ztest0, b + ztest);
      }

      ztest += (sqds + rt) * (sqds - rt);
      ztest0 = sqds * eml_div(e[q], sn);
      for (ii = q + 1; ii <= m + 1; ii++) {
        eml_xrotg(&ztest, &ztest0, &rt, &sn);
        if (ii > q + 1) {
          e[0] = ztest;
        }

        ztest = rt * s[ii - 1];
        ztest0 = sn * e[ii - 1];
        e[ii - 1] = rt * e[ii - 1] - sn * s[ii - 1];
        b = s[ii];
        s[ii] *= rt;
        eml_xrot(Vf, 3 * (ii - 1) + 1, 3 * ii + 1, rt, sn);
        s[ii - 1] = ztest + ztest0;
        ztest0 = sn * b;
        eml_xrotg(&s[ii - 1], &ztest0, &rt, &sn);
        ztest = rt * e[ii - 1] + sn * s[ii];
        s[ii] = -sn * e[ii - 1] + rt * s[ii];
        ztest0 = sn * e[ii];
        e[ii] *= rt;
        eml_xrot(U, 3 * (ii - 1) + 1, 3 * ii + 1, rt, sn);
      }

      e[m] = ztest;
      iter++;
      break;

     default:
      if (s[q] < 0.0F) {
        s[q] = -s[q];
        eml_xscal(3, -1.0F, Vf, 3 * q + 1);
      }

      i = q + 1;
      while ((q + 1 < 3) && (s[q] < s[i])) {
        rt = s[q];
        s[q] = s[i];
        s[i] = rt;
        eml_xswap(Vf, 3 * q + 1, 3 * (q + 1) + 1);
        eml_xswap(U, 3 * q + 1, 3 * (q + 1) + 1);
        q = i;
        i++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  for (ii = 0; ii < 3; ii++) {
    S[ii] = s[ii];
    for (i = 0; i < 3; i++) {
      V[i + 3 * ii] = Vf[i + 3 * ii];
    }
  }
}

static real32_T eml_xnrm2(int32_T n, const real32_T x[9], int32_T ix0)
{
  real32_T y;
  real32_T scale;
  int32_T kend;
  int32_T k;
  real32_T absxk;
  real32_T t;
  y = 0.0F;
  if (n < 1) {
  } else if (n == 1) {
    y = (real32_T)fabs(x[ix0 - 1]);
  } else {
    scale = 1.17549435E-38F;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = (real32_T)fabs(x[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0F + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * (real32_T)sqrt(y);
  }

  return y;
}

static void eml_xrot(real32_T x[9], int32_T ix0, int32_T iy0, real32_T c,
                     real32_T s)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  real32_T y;
  real32_T b_y;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 3; k++) {
    y = c * x[ix];
    b_y = s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = y + b_y;
    iy++;
    ix++;
  }
}

static void eml_xrotg(real32_T *a, real32_T *b, real32_T *c, real32_T *s)
{
  real32_T roe;
  real32_T absa;
  real32_T absb;
  real32_T scale;
  real32_T ads;
  real32_T bds;
  roe = *b;
  absa = (real32_T)fabs(*a);
  absb = (real32_T)fabs(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0F) {
    *s = 0.0F;
    *c = 1.0F;
    ads = 0.0F;
    scale = 0.0F;
  } else {
    ads = absa / scale;
    bds = absb / scale;
    ads = scale * (real32_T)sqrt(ads * ads + bds * bds);
    if (roe < 0.0F) {
      ads = -ads;
    }

    *c = *a / ads;
    *s = *b / ads;
    if (absa > absb) {
      scale = *s;
    } else if (*c != 0.0F) {
      scale = 1.0F / *c;
    } else {
      scale = 1.0F;
    }
  }

  *a = ads;
  *b = scale;
}

static void eml_xscal(int32_T n, real32_T a, real32_T x[9], int32_T ix0)
{
  int32_T i11;
  int32_T k;
  i11 = (ix0 + n) - 1;
  for (k = ix0; k <= i11; k++) {
    x[k - 1] *= a;
  }
}

static void eml_xswap(real32_T x[9], int32_T ix0, int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  real32_T temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 3; k++) {
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
  }
}

static void f_eml_xaxpy(int32_T n, real32_T a, const real32_T x[4], int32_T ix0,
  real32_T y[16], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

void b_pinv(const real32_T A[16], real32_T X[16])
{
  real32_T V[16];
  real32_T s[4];
  real32_T U[16];
  real32_T S[16];
  int32_T k;
  int32_T r;
  int32_T vcol;
  int32_T ar;
  real32_T z;
  int32_T ic;
  int32_T i3;
  int32_T ib;
  int32_T ia;
  memset(&X[0], 0, sizeof(real32_T) << 4);
  b_eml_xgesvd(A, U, s, V);
  memset(&S[0], 0, sizeof(real32_T) << 4);
  for (k = 0; k < 4; k++) {
    S[k + (k << 2)] = s[k];
  }

  r = 0;
  k = 0;
  while ((k + 1 < 5) && (S[k + (k << 2)] > 4.0F * S[0] * 1.1920929E-7F)) {
    r++;
    k++;
  }

  if (r > 0) {
    vcol = 0;
    for (ar = 0; ar + 1 <= r; ar++) {
      z = 1.0F / S[ar + (ar << 2)];
      for (k = vcol; k + 1 <= vcol + 4; k++) {
        V[k] *= z;
      }

      vcol += 4;
    }

    for (k = 0; k < 14; k += 4) {
      for (ic = k; ic + 1 <= k + 4; ic++) {
        X[ic] = 0.0F;
      }
    }

    vcol = -1;
    for (k = 0; k < 14; k += 4) {
      ar = 0;
      vcol++;
      i3 = (vcol + ((r - 1) << 2)) + 1;
      for (ib = vcol; ib + 1 <= i3; ib += 4) {
        if (U[ib] != 0.0F) {
          ia = ar;
          for (ic = k; ic + 1 <= k + 4; ic++) {
            ia++;
            X[ic] += U[ib] * V[ia - 1];
          }
        }

        ar += 4;
      }
    }
  }
}

void pinv(const real32_T A[9], real32_T X[9])
{
  int32_T k;
  real32_T V[9];
  real32_T s[3];
  real32_T U[9];
  real32_T S[9];
  int32_T r;
  int32_T vcol;
  int32_T br;
  real32_T z;
  int32_T ic;
  int32_T ar;
  int32_T ib;
  int32_T ia;
  for (k = 0; k < 9; k++) {
    X[k] = 0.0F;
  }

  eml_xgesvd(A, U, s, V);
  for (k = 0; k < 9; k++) {
    S[k] = 0.0F;
  }

  for (k = 0; k < 3; k++) {
    S[k + 3 * k] = s[k];
  }

  r = 0;
  k = 0;
  while ((k + 1 < 4) && (S[k + 3 * k] > 3.0F * S[0] * 1.1920929E-7F)) {
    r++;
    k++;
  }

  if (r > 0) {
    vcol = 0;
    for (br = 0; br + 1 <= r; br++) {
      z = 1.0F / S[br + 3 * br];
      for (k = vcol; k + 1 <= vcol + 3; k++) {
        V[k] *= z;
      }

      vcol += 3;
    }

    for (vcol = 0; vcol < 8; vcol += 3) {
      for (ic = vcol; ic + 1 <= vcol + 3; ic++) {
        X[ic] = 0.0F;
      }
    }

    br = -1;
    for (vcol = 0; vcol < 8; vcol += 3) {
      ar = 0;
      br++;
      k = (br + 3 * (r - 1)) + 1;
      for (ib = br; ib + 1 <= k; ib += 3) {
        if (U[ib] != 0.0F) {
          ia = ar;
          for (ic = vcol; ic + 1 <= vcol + 3; ic++) {
            ia++;
            X[ic] += U[ib] * V[ia - 1];
          }
        }

        ar += 3;
      }
    }
  }
}

/* End of code generation (pinv.c) */
