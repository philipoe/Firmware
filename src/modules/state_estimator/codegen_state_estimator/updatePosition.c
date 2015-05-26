/*
 * updatePosition.c
 *
 * Code generation for function 'updatePosition'
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
#include "dot.h"
#include "HyEst_emxutil.h"
#include "mldivide.h"
#include "diag.h"
#include "HyEst_rtwutil.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void eml_li_find(const boolean_T x[20], emxArray_int32_T *y)
{
  int32_T k;
  int32_T i;
  int32_T j;
  k = 0;
  for (i = 0; i < 20; i++) {
    if (x[i]) {
      k++;
    }
  }

  j = y->size[0];
  y->size[0] = k;
  emxEnsureCapacity((emxArray__common *)y, j, (int32_T)sizeof(int32_T));
  j = 0;
  for (i = 0; i < 20; i++) {
    if (x[i]) {
      y->data[j] = i + 1;
      j++;
    }
  }
}

void updatePosition(states_T *x, real32_T P[400], const real_T y_GpsPos_rad[3],
                    const real32_T R_GpsPos_m[9], boolean_T constWind, boolean_T
                    constQFF, boolean_T constK)
{
  real32_T denom;
  real32_T scaling[3];
  real32_T S[9];
  real32_T b[9];
  real32_T a[9];
  int32_T i7;
  int32_T i8;
  int32_T ar;
  real32_T K[60];
  boolean_T selector[20];
  int32_T i;
  emxArray_int32_T *r6;
  boolean_T b_selector[20];
  emxArray_int32_T *r7;
  emxArray_real32_T *b_a;
  emxArray_int32_T *r8;
  emxArray_int32_T *r9;
  int32_T iy;
  emxArray_int32_T *r10;
  emxArray_int32_T *r11;
  emxArray_int32_T *r12;
  emxArray_int32_T *r13;
  int8_T H[60];
  static const int8_T iv1[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  emxArray_real32_T *c_a;
  emxArray_real32_T *b_b;
  emxArray_real32_T *y;
  uint32_T unnamed_idx_1;
  int32_T ic;
  int32_T br;
  int32_T ib;
  int32_T ia;
  emxArray_real32_T *d_a;
  emxArray_real32_T *KH;
  uint32_T unnamed_idx_0;
  emxArray_real32_T *r14;
  emxArray_real32_T *C;
  emxArray_real32_T *b_C;
  emxArray_real32_T *b_y;
  emxArray_int32_T *r15;
  emxArray_int32_T *r16;
  static const int32_T iv2[3] = { 10000000, 10000000, 1 };

  real32_T delta_x[20];
  real32_T dtheta_half_norm;
  real32_T dq[4];
  real32_T Q[16];

  /*  */
  /*  [x,P] = UPDATEPOSITION(x,P,y_GpsPos_rad, R_GpsPos_m, ... */
  /*      constWind, constQFF,constK) */
  /*  */
  /*  Kalman filter update with GPS position. */
  /*  */
  /*  Author: lestefan */
  /*  date:  02/2013 */
  /*  */
  /*  Inputs: */
  /*    x:            states [p(lat*1e7,lon*1e7,h),q_NS,v_N,b_g,b_a,QFF,w,K] */
  /*    P:            covariance */
  /*    y_GpsPos_rad: GPS position [lat,lon,h] [rad,rad,m] */
  /*    R_GpsPos_m:   measurement covariance 3x3 [m^2] */
  /*    constWind:    keep wind constant [true/false] (use true on GPS out) */
  /*    constQFF:     keep QFF constant [true/false] (use true on GPS out) */
  /*    constK:       K (sideslip parameter) const. [true/false] (use true) */
  /*  */
  /*  Outputs: */
  /*    x:            updated states (see inputs) */
  /*    P:            updated covariance (see inputs) */
  /*  */
  /*  Needed globals: */
  /*    global configuration; */
  /*    configuration.tau:         accelerometer autocorrelation time [s] */
  /*    configuration.sigma_a_c:   accelerometer noise [m/s^2/sqrt(Hz)] */
  /*    configuration.sigma_aw_c:  accelerometer bias noise [m/s^3/sqrt(Hz)] */
  /*    configuration.sigma_g_c:   gyro noise [rad/s/sqrt(Hz)] */
  /*    configuration.sigma_gw_c:  gyro bias noise [rad/s^2/sqrt(Hz)] */
  /*    configuration.sigma_pw_c:  static pressure bias noise [kPa/s/sqrt(Hz)] */
  /*    configuration.sigma_w_c:   wind drift noise [m/s^2/sqrt(Hz)] */
  /*    configuration.sigma_psmd:  static pressure measurement variance [kPa] */
  /*    configuration.sigma_pdmd:  dynamic pressure measurement variance [kPa] */
  /*    configuration.sigma_Td=1:  temperature measurement variance [°K]% */
  /*  */
  /*  See also UPDATEVELNED, UPDATECOMPASS, UPDATEPRESSURES, UPDATEPRESSURES2, */
  /*  UPDATEPRESSURES3, PROPATATE, GETINITIALQ, MAGFIELD */
  /*  */
  /*  TODO: update in the past, re-propagate. This might not be fast enough... */
  /*  todo: use t to propagate to current time */
  /*  convert into radians */
  denom = (real32_T)sqrt(1.0F - 0.00669438F * rt_powf_snf((real32_T)sin
    ((real32_T)x->p[0] / 1.0E+7F), 2.0F));
  scaling[0] = 1.0E+7F / (6.3354395E+6F / rt_powf_snf(denom, 3.0F) + (real32_T)
    x->p[2]);
  scaling[1] = 1.0E+7F / ((6.378137E+6F / denom + (real32_T)x->p[2]) * (real32_T)
    cos((real32_T)x->p[0] / 1.0E+7F));
  scaling[2] = -1.0F;
  diag(scaling, S);
  diag(scaling, b);

  /*  measurement Jacobian */
  /* H=double([eye(3),zeros(3,13)]); */
  /* S=H*P*H'+R_GpsPos_m; */
  for (i7 = 0; i7 < 3; i7++) {
    for (i8 = 0; i8 < 3; i8++) {
      a[i7 + 3 * i8] = 0.0F;
      for (ar = 0; ar < 3; ar++) {
        a[i7 + 3 * i8] += S[i7 + 3 * ar] * R_GpsPos_m[ar + 3 * i8];
      }
    }
  }

  for (i7 = 0; i7 < 3; i7++) {
    for (i8 = 0; i8 < 3; i8++) {
      denom = 0.0F;
      for (ar = 0; ar < 3; ar++) {
        denom += a[i7 + 3 * ar] * b[ar + 3 * i8];
      }

      S[i7 + 3 * i8] = P[i7 + 20 * i8] + denom;
    }
  }

  memset(&K[0], 0, 60U * sizeof(real32_T));

  /*  set float type */
  for (i = 0; i < 20; i++) {
    selector[i] = TRUE;
  }

  if (constK) {
    selector[19] = FALSE;
  }

  if (constQFF) {
    selector[15] = FALSE;
  }

  emxInit_int32_T(&r6, 1);

  /*  remove correlations of fixed states with active states */
  eml_li_find(selector, r6);
  i7 = r6->size[0];
  r6->size[0] = r6->size[0];
  emxEnsureCapacity((emxArray__common *)r6, i7, (int32_T)sizeof(int32_T));
  i = r6->size[0];
  for (i7 = 0; i7 < i; i7++) {
    r6->data[i7]--;
  }

  for (i = 0; i < 20; i++) {
    b_selector[i] = !selector[i];
  }

  emxInit_int32_T(&r7, 1);
  eml_li_find(b_selector, r7);
  i7 = r7->size[0];
  r7->size[0] = r7->size[0];
  emxEnsureCapacity((emxArray__common *)r7, i7, (int32_T)sizeof(int32_T));
  i = r7->size[0];
  for (i7 = 0; i7 < i; i7++) {
    r7->data[i7]--;
  }

  for (i = 0; i < 20; i++) {
    b_selector[i] = !selector[i];
  }

  emxInit_real32_T(&b_a, 2);
  emxInit_int32_T(&r8, 1);
  emxInit_int32_T(&r9, 1);
  eml_li_find(b_selector, r8);
  eml_li_find(selector, r9);
  i7 = b_a->size[0] * b_a->size[1];
  b_a->size[0] = r9->size[0];
  b_a->size[1] = r8->size[0];
  emxEnsureCapacity((emxArray__common *)b_a, i7, (int32_T)sizeof(real32_T));
  i = r8->size[0];
  for (i7 = 0; i7 < i; i7++) {
    iy = r9->size[0];
    for (i8 = 0; i8 < iy; i8++) {
      b_a->data[i8 + b_a->size[0] * i7] = P[(r9->data[i8] + 20 * (r8->data[i7] -
        1)) - 1];
    }
  }

  emxInit_int32_T(&r10, 1);
  i7 = r10->size[0];
  r10->size[0] = r7->size[0];
  emxEnsureCapacity((emxArray__common *)r10, i7, (int32_T)sizeof(int32_T));
  i = r7->size[0];
  for (i7 = 0; i7 < i; i7++) {
    r10->data[i7] = r7->data[i7];
  }

  emxInit_int32_T(&r11, 1);
  i7 = r11->size[0];
  r11->size[0] = r6->size[0];
  emxEnsureCapacity((emxArray__common *)r11, i7, (int32_T)sizeof(int32_T));
  i = r6->size[0];
  for (i7 = 0; i7 < i; i7++) {
    r11->data[i7] = r6->data[i7];
  }

  i = b_a->size[1];
  for (i7 = 0; i7 < i; i7++) {
    iy = b_a->size[0];
    for (i8 = 0; i8 < iy; i8++) {
      P[r11->data[i8] + 20 * r10->data[i7]] = b_a->data[i8 + b_a->size[0] * i7] *
        0.0F;
    }
  }

  emxFree_int32_T(&r11);
  emxFree_int32_T(&r10);
  for (i = 0; i < 20; i++) {
    b_selector[i] = !selector[i];
  }

  eml_li_find(b_selector, r6);
  i7 = r6->size[0];
  r6->size[0] = r6->size[0];
  emxEnsureCapacity((emxArray__common *)r6, i7, (int32_T)sizeof(int32_T));
  i = r6->size[0];
  for (i7 = 0; i7 < i; i7++) {
    r6->data[i7]--;
  }

  eml_li_find(selector, r7);
  i7 = r7->size[0];
  r7->size[0] = r7->size[0];
  emxEnsureCapacity((emxArray__common *)r7, i7, (int32_T)sizeof(int32_T));
  i = r7->size[0];
  for (i7 = 0; i7 < i; i7++) {
    r7->data[i7]--;
  }

  eml_li_find(selector, r8);
  for (i = 0; i < 20; i++) {
    b_selector[i] = !selector[i];
  }

  eml_li_find(b_selector, r9);
  i7 = b_a->size[0] * b_a->size[1];
  b_a->size[0] = r9->size[0];
  b_a->size[1] = r8->size[0];
  emxEnsureCapacity((emxArray__common *)b_a, i7, (int32_T)sizeof(real32_T));
  i = r8->size[0];
  for (i7 = 0; i7 < i; i7++) {
    iy = r9->size[0];
    for (i8 = 0; i8 < iy; i8++) {
      b_a->data[i8 + b_a->size[0] * i7] = P[(r9->data[i8] + 20 * (r8->data[i7] -
        1)) - 1];
    }
  }

  emxFree_int32_T(&r9);
  emxInit_int32_T(&r12, 1);
  i7 = r12->size[0];
  r12->size[0] = r7->size[0];
  emxEnsureCapacity((emxArray__common *)r12, i7, (int32_T)sizeof(int32_T));
  i = r7->size[0];
  for (i7 = 0; i7 < i; i7++) {
    r12->data[i7] = r7->data[i7];
  }

  emxInit_int32_T(&r13, 1);
  i7 = r13->size[0];
  r13->size[0] = r6->size[0];
  emxEnsureCapacity((emxArray__common *)r13, i7, (int32_T)sizeof(int32_T));
  i = r6->size[0];
  for (i7 = 0; i7 < i; i7++) {
    r13->data[i7] = r6->data[i7];
  }

  i = b_a->size[1];
  for (i7 = 0; i7 < i; i7++) {
    iy = b_a->size[0];
    for (i8 = 0; i8 < iy; i8++) {
      P[r13->data[i8] + 20 * r12->data[i7]] = b_a->data[i8 + b_a->size[0] * i7] *
        0.0F;
    }
  }

  emxFree_int32_T(&r13);
  emxFree_int32_T(&r12);

  /* P(~selector,~selector)=P(~selector,~selector)*0; */
  for (i7 = 0; i7 < 60; i7++) {
    H[i7] = 0;
  }

  /*  set float type */
  for (i7 = 0; i7 < 3; i7++) {
    for (i8 = 0; i8 < 3; i8++) {
      H[i8 + 3 * i7] = iv1[i8 + 3 * i7];
    }
  }

  /*  set float type */
  eml_li_find(selector, r6);
  i7 = r6->size[0];
  r6->size[0] = r6->size[0];
  emxEnsureCapacity((emxArray__common *)r6, i7, (int32_T)sizeof(int32_T));
  i = r6->size[0];
  for (i7 = 0; i7 < i; i7++) {
    r6->data[i7]--;
  }

  emxInit_real32_T(&c_a, 2);
  eml_li_find(selector, r7);
  i7 = c_a->size[0] * c_a->size[1];
  c_a->size[0] = 3;
  c_a->size[1] = r7->size[0];
  emxEnsureCapacity((emxArray__common *)c_a, i7, (int32_T)sizeof(real32_T));
  i = r7->size[0];
  for (i7 = 0; i7 < i; i7++) {
    for (i8 = 0; i8 < 3; i8++) {
      c_a->data[i8 + c_a->size[0] * i7] = (real32_T)H[i8 + 3 * (r7->data[i7] - 1)];
    }
  }

  emxInit_real32_T(&b_b, 2);
  eml_li_find(selector, r7);
  eml_li_find(selector, r8);
  i7 = b_b->size[0] * b_b->size[1];
  b_b->size[0] = r8->size[0];
  b_b->size[1] = r7->size[0];
  emxEnsureCapacity((emxArray__common *)b_b, i7, (int32_T)sizeof(real32_T));
  i = r7->size[0];
  for (i7 = 0; i7 < i; i7++) {
    iy = r8->size[0];
    for (i8 = 0; i8 < iy; i8++) {
      b_b->data[i8 + b_b->size[0] * i7] = P[(r8->data[i8] + 20 * (r7->data[i7] -
        1)) - 1];
    }
  }

  emxFree_int32_T(&r8);
  emxInit_real32_T(&y, 2);
  if ((c_a->size[1] == 1) || (b_b->size[0] == 1)) {
    i7 = y->size[0] * y->size[1];
    y->size[0] = 3;
    y->size[1] = b_b->size[1];
    emxEnsureCapacity((emxArray__common *)y, i7, (int32_T)sizeof(real32_T));
    for (i7 = 0; i7 < 3; i7++) {
      i = b_b->size[1];
      for (i8 = 0; i8 < i; i8++) {
        y->data[i7 + y->size[0] * i8] = 0.0F;
        iy = c_a->size[1];
        for (ar = 0; ar < iy; ar++) {
          y->data[i7 + y->size[0] * i8] += c_a->data[i7 + c_a->size[0] * ar] *
            b_b->data[ar + b_b->size[0] * i8];
        }
      }
    }
  } else {
    unnamed_idx_1 = (uint32_T)b_b->size[1];
    i7 = y->size[0] * y->size[1];
    y->size[0] = 3;
    emxEnsureCapacity((emxArray__common *)y, i7, (int32_T)sizeof(real32_T));
    i7 = y->size[0] * y->size[1];
    y->size[1] = (int32_T)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)y, i7, (int32_T)sizeof(real32_T));
    i = 3 * (int32_T)unnamed_idx_1;
    for (i7 = 0; i7 < i; i7++) {
      y->data[i7] = 0.0F;
    }

    if (b_b->size[1] == 0) {
    } else {
      i = 3 * (b_b->size[1] - 1);
      for (iy = 0; iy <= i; iy += 3) {
        for (ic = iy; ic + 1 <= iy + 3; ic++) {
          y->data[ic] = 0.0F;
        }
      }

      br = 0;
      for (iy = 0; iy <= i; iy += 3) {
        ar = 0;
        i7 = br + c_a->size[1];
        for (ib = br; ib + 1 <= i7; ib++) {
          if (b_b->data[ib] != 0.0F) {
            ia = ar;
            for (ic = iy; ic + 1 <= iy + 3; ic++) {
              ia++;
              y->data[ic] += b_b->data[ib] * c_a->data[ia - 1];
            }
          }

          ar += 3;
        }

        br += c_a->size[1];
      }
    }
  }

  emxInit_real32_T(&d_a, 2);
  mldivide(S, y, c_a);
  i7 = d_a->size[0] * d_a->size[1];
  d_a->size[0] = c_a->size[1];
  d_a->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)d_a, i7, (int32_T)sizeof(real32_T));
  emxFree_real32_T(&y);
  for (i7 = 0; i7 < 3; i7++) {
    i = c_a->size[1];
    for (i8 = 0; i8 < i; i8++) {
      d_a->data[i8 + d_a->size[0] * i7] = c_a->data[i7 + c_a->size[0] * i8];
    }
  }

  for (i7 = 0; i7 < 3; i7++) {
    i = d_a->size[0];
    for (i8 = 0; i8 < i; i8++) {
      K[r6->data[i8] + 20 * i7] = d_a->data[i8 + d_a->size[0] * i7];
    }
  }

  /*  */
  /*  % Outline for the position inovation (canceled) */
  /*  e =y_GpsPos_rad.*[1e7;1e7;1]-x.p; */
  /*  chi2=(e.*[1e-7;1e-7;1])'*pinv(S)*(e.*[1e-7;1e-7;1]); */
  /*  %disp(e); */
  /*  if (chi2>50)       %~0.7*0.7/(P+0.01)~25 */
  /*      %disp('updateVelocity chi2-test fail'); */
  /*      return */
  /*  end */
  /*  delta_x=(K*e);  */
  /*  */
  /*  update: */
  eml_li_find(selector, r6);
  i7 = d_a->size[0] * d_a->size[1];
  d_a->size[0] = r6->size[0];
  d_a->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)d_a, i7, (int32_T)sizeof(real32_T));
  for (i7 = 0; i7 < 3; i7++) {
    i = r6->size[0];
    for (i8 = 0; i8 < i; i8++) {
      d_a->data[i8 + d_a->size[0] * i7] = K[(r6->data[i8] + 20 * i7) - 1];
    }
  }

  eml_li_find(selector, r6);
  i7 = c_a->size[0] * c_a->size[1];
  c_a->size[0] = 3;
  c_a->size[1] = r6->size[0];
  emxEnsureCapacity((emxArray__common *)c_a, i7, (int32_T)sizeof(real32_T));
  i = r6->size[0];
  for (i7 = 0; i7 < i; i7++) {
    for (i8 = 0; i8 < 3; i8++) {
      c_a->data[i8 + c_a->size[0] * i7] = (real32_T)H[i8 + 3 * (r6->data[i7] - 1)];
    }
  }

  emxInit_real32_T(&KH, 2);
  unnamed_idx_0 = (uint32_T)d_a->size[0];
  unnamed_idx_1 = (uint32_T)c_a->size[1];
  i7 = KH->size[0] * KH->size[1];
  KH->size[0] = (int32_T)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)KH, i7, (int32_T)sizeof(real32_T));
  i7 = KH->size[0] * KH->size[1];
  KH->size[1] = (int32_T)unnamed_idx_1;
  emxEnsureCapacity((emxArray__common *)KH, i7, (int32_T)sizeof(real32_T));
  i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
  for (i7 = 0; i7 < i; i7++) {
    KH->data[i7] = 0.0F;
  }

  if ((d_a->size[0] == 0) || (c_a->size[1] == 0)) {
  } else {
    i = d_a->size[0] * (c_a->size[1] - 1);
    for (iy = 0; iy <= i; iy += d_a->size[0]) {
      i7 = iy + d_a->size[0];
      for (ic = iy; ic + 1 <= i7; ic++) {
        KH->data[ic] = 0.0F;
      }
    }

    br = 0;
    for (iy = 0; iy <= i; iy += d_a->size[0]) {
      ar = 0;
      for (ib = br; ib + 1 <= br + 3; ib++) {
        if (c_a->data[ib] != 0.0F) {
          ia = ar;
          i7 = iy + d_a->size[0];
          for (ic = iy; ic + 1 <= i7; ic++) {
            ia++;
            KH->data[ic] += c_a->data[ib] * d_a->data[ia - 1];
          }
        }

        ar += d_a->size[0];
      }

      br += 3;
    }
  }

  emxInit_real32_T(&r14, 2);
  eml_li_find(selector, r6);
  eml_li_find(selector, r7);
  i7 = r14->size[0] * r14->size[1];
  r14->size[0] = r7->size[0];
  r14->size[1] = r6->size[0];
  emxEnsureCapacity((emxArray__common *)r14, i7, (int32_T)sizeof(real32_T));
  i = r6->size[0];
  for (i7 = 0; i7 < i; i7++) {
    iy = r7->size[0];
    for (i8 = 0; i8 < iy; i8++) {
      r14->data[i8 + r14->size[0] * i7] = P[(r7->data[i8] + 20 * (r6->data[i7] -
        1)) - 1];
    }
  }

  eml_li_find(selector, r6);
  eml_li_find(selector, r7);
  i7 = b_b->size[0] * b_b->size[1];
  b_b->size[0] = r7->size[0];
  b_b->size[1] = r6->size[0];
  emxEnsureCapacity((emxArray__common *)b_b, i7, (int32_T)sizeof(real32_T));
  i = r6->size[0];
  for (i7 = 0; i7 < i; i7++) {
    iy = r7->size[0];
    for (i8 = 0; i8 < iy; i8++) {
      b_b->data[i8 + b_b->size[0] * i7] = P[(r7->data[i8] + 20 * (r6->data[i7] -
        1)) - 1];
    }
  }

  emxInit_real32_T(&C, 2);
  if ((KH->size[1] == 1) || (b_b->size[0] == 1)) {
    i7 = C->size[0] * C->size[1];
    C->size[0] = KH->size[0];
    C->size[1] = b_b->size[1];
    emxEnsureCapacity((emxArray__common *)C, i7, (int32_T)sizeof(real32_T));
    i = KH->size[0];
    for (i7 = 0; i7 < i; i7++) {
      iy = b_b->size[1];
      for (i8 = 0; i8 < iy; i8++) {
        C->data[i7 + C->size[0] * i8] = 0.0F;
        br = KH->size[1];
        for (ar = 0; ar < br; ar++) {
          C->data[i7 + C->size[0] * i8] += KH->data[i7 + KH->size[0] * ar] *
            b_b->data[ar + b_b->size[0] * i8];
        }
      }
    }
  } else {
    unnamed_idx_0 = (uint32_T)KH->size[0];
    unnamed_idx_1 = (uint32_T)b_b->size[1];
    i7 = C->size[0] * C->size[1];
    C->size[0] = (int32_T)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)C, i7, (int32_T)sizeof(real32_T));
    i7 = C->size[0] * C->size[1];
    C->size[1] = (int32_T)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)C, i7, (int32_T)sizeof(real32_T));
    i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
    for (i7 = 0; i7 < i; i7++) {
      C->data[i7] = 0.0F;
    }

    if ((KH->size[0] == 0) || (b_b->size[1] == 0)) {
    } else {
      i = KH->size[0] * (b_b->size[1] - 1);
      for (iy = 0; iy <= i; iy += KH->size[0]) {
        i7 = iy + KH->size[0];
        for (ic = iy; ic + 1 <= i7; ic++) {
          C->data[ic] = 0.0F;
        }
      }

      br = 0;
      for (iy = 0; iy <= i; iy += KH->size[0]) {
        ar = 0;
        i7 = br + KH->size[1];
        for (ib = br; ib + 1 <= i7; ib++) {
          if (b_b->data[ib] != 0.0F) {
            ia = ar;
            i8 = iy + KH->size[0];
            for (ic = iy; ic + 1 <= i8; ic++) {
              ia++;
              C->data[ic] += b_b->data[ib] * KH->data[ia - 1];
            }
          }

          ar += KH->size[0];
        }

        br += KH->size[1];
      }
    }
  }

  eml_li_find(selector, r6);
  eml_li_find(selector, r7);
  i7 = b_a->size[0] * b_a->size[1];
  b_a->size[0] = r7->size[0];
  b_a->size[1] = r6->size[0];
  emxEnsureCapacity((emxArray__common *)b_a, i7, (int32_T)sizeof(real32_T));
  i = r6->size[0];
  for (i7 = 0; i7 < i; i7++) {
    iy = r7->size[0];
    for (i8 = 0; i8 < iy; i8++) {
      b_a->data[i8 + b_a->size[0] * i7] = P[(r7->data[i8] + 20 * (r6->data[i7] -
        1)) - 1];
    }
  }

  i7 = b_b->size[0] * b_b->size[1];
  b_b->size[0] = KH->size[1];
  b_b->size[1] = KH->size[0];
  emxEnsureCapacity((emxArray__common *)b_b, i7, (int32_T)sizeof(real32_T));
  i = KH->size[0];
  for (i7 = 0; i7 < i; i7++) {
    iy = KH->size[1];
    for (i8 = 0; i8 < iy; i8++) {
      b_b->data[i8 + b_b->size[0] * i7] = KH->data[i7 + KH->size[0] * i8];
    }
  }

  emxInit_real32_T(&b_C, 2);
  if ((b_a->size[1] == 1) || (b_b->size[0] == 1)) {
    i7 = b_C->size[0] * b_C->size[1];
    b_C->size[0] = b_a->size[0];
    b_C->size[1] = b_b->size[1];
    emxEnsureCapacity((emxArray__common *)b_C, i7, (int32_T)sizeof(real32_T));
    i = b_a->size[0];
    for (i7 = 0; i7 < i; i7++) {
      iy = b_b->size[1];
      for (i8 = 0; i8 < iy; i8++) {
        b_C->data[i7 + b_C->size[0] * i8] = 0.0F;
        br = b_a->size[1];
        for (ar = 0; ar < br; ar++) {
          b_C->data[i7 + b_C->size[0] * i8] += b_a->data[i7 + b_a->size[0] * ar]
            * b_b->data[ar + b_b->size[0] * i8];
        }
      }
    }
  } else {
    unnamed_idx_0 = (uint32_T)b_a->size[0];
    unnamed_idx_1 = (uint32_T)b_b->size[1];
    i7 = b_C->size[0] * b_C->size[1];
    b_C->size[0] = (int32_T)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)b_C, i7, (int32_T)sizeof(real32_T));
    i7 = b_C->size[0] * b_C->size[1];
    b_C->size[1] = (int32_T)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)b_C, i7, (int32_T)sizeof(real32_T));
    i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
    for (i7 = 0; i7 < i; i7++) {
      b_C->data[i7] = 0.0F;
    }

    if ((b_a->size[0] == 0) || (b_b->size[1] == 0)) {
    } else {
      i = b_a->size[0] * (b_b->size[1] - 1);
      for (iy = 0; iy <= i; iy += b_a->size[0]) {
        i7 = iy + b_a->size[0];
        for (ic = iy; ic + 1 <= i7; ic++) {
          b_C->data[ic] = 0.0F;
        }
      }

      br = 0;
      for (iy = 0; iy <= i; iy += b_a->size[0]) {
        ar = 0;
        i7 = br + b_a->size[1];
        for (ib = br; ib + 1 <= i7; ib++) {
          if (b_b->data[ib] != 0.0F) {
            ia = ar;
            i8 = iy + b_a->size[0];
            for (ic = iy; ic + 1 <= i8; ic++) {
              ia++;
              b_C->data[ic] += b_b->data[ib] * b_a->data[ia - 1];
            }
          }

          ar += b_a->size[0];
        }

        br += b_a->size[1];
      }
    }
  }

  emxFree_real32_T(&b_b);
  emxFree_real32_T(&b_a);
  eml_li_find(selector, r6);
  i7 = d_a->size[0] * d_a->size[1];
  d_a->size[0] = r6->size[0];
  d_a->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)d_a, i7, (int32_T)sizeof(real32_T));
  for (i7 = 0; i7 < 3; i7++) {
    i = r6->size[0];
    for (i8 = 0; i8 < i; i8++) {
      d_a->data[i8 + d_a->size[0] * i7] = K[(r6->data[i8] + 20 * i7) - 1];
    }
  }

  emxInit_real32_T(&b_y, 2);
  unnamed_idx_0 = (uint32_T)d_a->size[0];
  i7 = b_y->size[0] * b_y->size[1];
  b_y->size[0] = (int32_T)unnamed_idx_0;
  b_y->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)b_y, i7, (int32_T)sizeof(real32_T));
  i = (int32_T)unnamed_idx_0 * 3;
  for (i7 = 0; i7 < i; i7++) {
    b_y->data[i7] = 0.0F;
  }

  if (d_a->size[0] == 0) {
  } else {
    i = d_a->size[0] << 1;
    for (iy = 0; iy <= i; iy += d_a->size[0]) {
      i7 = iy + d_a->size[0];
      for (ic = iy; ic + 1 <= i7; ic++) {
        b_y->data[ic] = 0.0F;
      }
    }

    br = 0;
    for (iy = 0; iy <= i; iy += d_a->size[0]) {
      ar = 0;
      for (ib = br; ib + 1 <= br + 3; ib++) {
        if (S[ib] != 0.0F) {
          ia = ar;
          i7 = iy + d_a->size[0];
          for (ic = iy; ic + 1 <= i7; ic++) {
            ia++;
            b_y->data[ic] += S[ib] * d_a->data[ia - 1];
          }
        }

        ar += d_a->size[0];
      }

      br += 3;
    }
  }

  emxFree_real32_T(&d_a);
  eml_li_find(selector, r6);
  i7 = c_a->size[0] * c_a->size[1];
  c_a->size[0] = 3;
  c_a->size[1] = r6->size[0];
  emxEnsureCapacity((emxArray__common *)c_a, i7, (int32_T)sizeof(real32_T));
  i = r6->size[0];
  for (i7 = 0; i7 < i; i7++) {
    for (i8 = 0; i8 < 3; i8++) {
      c_a->data[i8 + c_a->size[0] * i7] = K[(r6->data[i7] + 20 * i8) - 1];
    }
  }

  unnamed_idx_0 = (uint32_T)b_y->size[0];
  unnamed_idx_1 = (uint32_T)c_a->size[1];
  i7 = KH->size[0] * KH->size[1];
  KH->size[0] = (int32_T)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)KH, i7, (int32_T)sizeof(real32_T));
  i7 = KH->size[0] * KH->size[1];
  KH->size[1] = (int32_T)unnamed_idx_1;
  emxEnsureCapacity((emxArray__common *)KH, i7, (int32_T)sizeof(real32_T));
  i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
  for (i7 = 0; i7 < i; i7++) {
    KH->data[i7] = 0.0F;
  }

  if ((b_y->size[0] == 0) || (c_a->size[1] == 0)) {
  } else {
    i = b_y->size[0] * (c_a->size[1] - 1);
    for (iy = 0; iy <= i; iy += b_y->size[0]) {
      i7 = iy + b_y->size[0];
      for (ic = iy; ic + 1 <= i7; ic++) {
        KH->data[ic] = 0.0F;
      }
    }

    br = 0;
    for (iy = 0; iy <= i; iy += b_y->size[0]) {
      ar = 0;
      for (ib = br; ib + 1 <= br + 3; ib++) {
        if (c_a->data[ib] != 0.0F) {
          ia = ar;
          i7 = iy + b_y->size[0];
          for (ic = iy; ic + 1 <= i7; ic++) {
            ia++;
            KH->data[ic] += c_a->data[ib] * b_y->data[ia - 1];
          }
        }

        ar += b_y->size[0];
      }

      br += 3;
    }
  }

  emxFree_real32_T(&b_y);
  emxFree_real32_T(&c_a);
  eml_li_find(selector, r6);
  i7 = r6->size[0];
  r6->size[0] = r6->size[0];
  emxEnsureCapacity((emxArray__common *)r6, i7, (int32_T)sizeof(int32_T));
  i = r6->size[0];
  for (i7 = 0; i7 < i; i7++) {
    r6->data[i7]--;
  }

  eml_li_find(selector, r7);
  i7 = r7->size[0];
  r7->size[0] = r7->size[0];
  emxEnsureCapacity((emxArray__common *)r7, i7, (int32_T)sizeof(int32_T));
  i = r7->size[0];
  for (i7 = 0; i7 < i; i7++) {
    r7->data[i7]--;
  }

  emxInit_int32_T(&r15, 1);
  i7 = r15->size[0];
  r15->size[0] = r7->size[0];
  emxEnsureCapacity((emxArray__common *)r15, i7, (int32_T)sizeof(int32_T));
  i = r7->size[0];
  for (i7 = 0; i7 < i; i7++) {
    r15->data[i7] = r7->data[i7];
  }

  emxFree_int32_T(&r7);
  emxInit_int32_T(&r16, 1);
  i7 = r16->size[0];
  r16->size[0] = r6->size[0];
  emxEnsureCapacity((emxArray__common *)r16, i7, (int32_T)sizeof(int32_T));
  i = r6->size[0];
  for (i7 = 0; i7 < i; i7++) {
    r16->data[i7] = r6->data[i7];
  }

  emxFree_int32_T(&r6);
  i = r14->size[1];
  for (i7 = 0; i7 < i; i7++) {
    iy = r14->size[0];
    for (i8 = 0; i8 < iy; i8++) {
      P[r16->data[i8] + 20 * r15->data[i7]] = ((r14->data[i8 + r14->size[0] * i7]
        - C->data[i8 + C->size[0] * i7]) - b_C->data[i8 + b_C->size[0] * i7]) +
        KH->data[i8 + KH->size[0] * i7];
    }
  }

  emxFree_int32_T(&r16);
  emxFree_int32_T(&r15);
  emxFree_real32_T(&b_C);
  emxFree_real32_T(&C);
  emxFree_real32_T(&r14);
  emxFree_real32_T(&KH);
  for (i7 = 0; i7 < 3; i7++) {
    scaling[i7] = (real32_T)(y_GpsPos_rad[i7] * (real_T)iv2[i7] - x->p[i7]);
  }

  for (i7 = 0; i7 < 20; i7++) {
    delta_x[i7] = 0.0F;
    for (i8 = 0; i8 < 3; i8++) {
      delta_x[i7] += K[i7 + 20 * i8] * scaling[i8];
    }
  }

  /*  effective state update: */
  for (i7 = 0; i7 < 3; i7++) {
    x->p[i7] += (real_T)delta_x[i7];
  }

  /* disp(x.p(3)-y_GpsPos_rad(3)) */
  for (i = 0; i < 3; i++) {
    scaling[i] = 0.5F * delta_x[i + 3];
  }

  dtheta_half_norm = (real32_T)sqrt(dot(scaling, scaling));
  if ((real32_T)fabs(dtheta_half_norm) < 0.01F) {
    denom = dtheta_half_norm * dtheta_half_norm;
    denom = ((1.0F - 0.166666672F * denom) + 0.00833333377F * (denom * denom)) -
      0.000198412701F * (denom * denom * denom);
  } else {
    denom = (real32_T)sin(dtheta_half_norm) / dtheta_half_norm;
  }

  for (i = 0; i < 3; i++) {
    dq[i] = denom * scaling[i];
  }

  dq[3] = (real32_T)cos(dtheta_half_norm);

  /*  optimization (rectification of sinc) */
  /*  [ q3, q2, -q1, q0] */
  /*  [ -q2, q3, q0, q1] */
  /*  [ q1, -q0, q3, q2] */
  /*  [ -q0, -q1, -q2, q3] */
  /*  set float type */
  Q[0] = dq[3];
  Q[4] = dq[2];
  Q[8] = -dq[1];
  Q[12] = dq[0];
  Q[1] = -dq[2];
  Q[5] = dq[3];
  Q[9] = dq[0];
  Q[13] = dq[1];
  Q[2] = dq[1];
  Q[6] = -dq[0];
  Q[10] = dq[3];
  Q[14] = dq[2];
  Q[3] = -dq[0];
  Q[7] = -dq[1];
  Q[11] = -dq[2];
  Q[15] = dq[3];
  for (i7 = 0; i7 < 4; i7++) {
    dq[i7] = 0.0F;
    for (i8 = 0; i8 < 4; i8++) {
      denom = dq[i7] + Q[i7 + (i8 << 2)] * x->q_NS[i8];
      dq[i7] = denom;
    }
  }

  denom = 0.0F;
  i = 0;
  iy = 0;
  for (br = 0; br < 4; br++) {
    denom += dq[i] * dq[iy];
    i++;
    iy++;
  }

  denom = (real32_T)sqrt(denom);
  for (i7 = 0; i7 < 4; i7++) {
    dq[i7] /= denom;
  }

  for (i = 0; i < 4; i++) {
    x->q_NS[i] = dq[i];
  }

  for (i7 = 0; i7 < 3; i7++) {
    x->v_N[i7] += delta_x[6 + i7];
  }

  for (i7 = 0; i7 < 3; i7++) {
    x->b_g[i7] += delta_x[9 + i7];
  }

  for (i7 = 0; i7 < 3; i7++) {
    x->b_a[i7] += delta_x[12 + i7];
  }

  x->QFF += delta_x[15];
  for (i7 = 0; i7 < 3; i7++) {
    x->w[i7] += delta_x[16 + i7];
  }

  x->K += delta_x[19];
}

/* End of code generation (updatePosition.c) */
