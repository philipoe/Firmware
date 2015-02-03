/*
 * updateVelNed.c
 *
 * Code generation for function 'updateVelNed'
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
#include "HyEst_emxutil.h"
#include "dot.h"
#include "pinv.h"
#include "mldivide.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void updateVelNed(states_T *x, real32_T P[400], const real32_T y_VelNed[3],
                  const real32_T R_VelNed[9], boolean_T constWind, boolean_T
                  constQFF, boolean_T constK)
{
  real32_T S[9];
  int32_T i9;
  int32_T i10;
  real32_T K[60];
  boolean_T selector[20];
  int32_T i;
  emxArray_int32_T *r17;
  boolean_T b_selector[20];
  emxArray_int32_T *r18;
  emxArray_real32_T *a;
  emxArray_int32_T *r19;
  emxArray_int32_T *r20;
  int32_T iy;
  emxArray_int32_T *r21;
  emxArray_int32_T *r22;
  emxArray_int32_T *r23;
  emxArray_int32_T *r24;
  int8_T H[60];
  static const int8_T iv3[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  emxArray_real32_T *b_a;
  emxArray_real32_T *b;
  emxArray_real32_T *y;
  int32_T ar;
  uint32_T unnamed_idx_1;
  int32_T ic;
  int32_T br;
  int32_T ib;
  int32_T ia;
  emxArray_real32_T *c_a;
  real32_T e[3];
  real32_T b_b[9];
  real32_T chi2;
  real32_T b_y[3];
  real32_T delta_x[20];
  emxArray_real32_T *KH;
  uint32_T unnamed_idx_0;
  emxArray_real32_T *r25;
  emxArray_real32_T *C;
  emxArray_real32_T *b_C;
  emxArray_real32_T *c_y;
  emxArray_int32_T *r26;
  emxArray_int32_T *r27;
  real32_T dtheta_half_norm;
  real32_T dq[4];
  real32_T Q[16];

  /*  */
  /*  [x,P] = UPDATEVELNED(x,P,y_VelNed, R_VelNed, ... */
  /*      constWind, constQFF,constK) */
  /*  */
  /*  Kalman filter update with GPS speed. */
  /*  */
  /*  Author: lestefan */
  /*  date:  02/2013 */
  /*   */
  /*  Update: Amir Melzer (8/2013): Add outline for the velocity innovation */
  /*  Inputs: */
  /*    x:            states [p(lat*1e7,lon*1e7,h),q_NS,v_N,b_g,b_a,QFF,w,K] */
  /*    P:            covariance */
  /*    y_VelNed:     GPS speed (nav frame N) [V_N_N,V_N_E,V_N_D] [m/s,m/s,m/s] */
  /*    R_VelNed:     measurement covariance 3x3 [m^2/s^2] */
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
  /*    configuration.sigma_Td=1:  temperature measurement variance [°K] */
  /*  */
  /*  See also UPDATEPOSITION, UPDATECOMPASS, UPDATEPRESSURES,  */
  /*  UPDATEPRESSURES2, UPDATEPRESSURES3, PROPATATE, GETINITIALQ, MAGFIELD */
  /*  */
  /*  TODO: update in the past, re-propagate. This might not be fast enough... */
  /*  TODO: test this... */
  /*  measurement Jacobian */
  /* H=double([zeros(3,6),eye(3),zeros(3,7)]); */
  /* S=H*P*H'+R_GpsPos_m; */
  for (i9 = 0; i9 < 3; i9++) {
    for (i10 = 0; i10 < 3; i10++) {
      S[i10 + 3 * i9] = P[(i10 + 20 * (6 + i9)) + 6] + R_VelNed[i10 + 3 * i9];
    }
  }

  /* K=(P*H'*(S)^(-1)); % gain */
  /* K=(S\P(7:9,:))'; % gain */
  /* K=zeros(size(P,1),3);                                  % commented                              */
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

  emxInit_int32_T(&r17, 1);

  /*  remove correlations of fixed states with active states */
  eml_li_find(selector, r17);
  i9 = r17->size[0];
  r17->size[0] = r17->size[0];
  emxEnsureCapacity((emxArray__common *)r17, i9, (int32_T)sizeof(int32_T));
  i = r17->size[0];
  for (i9 = 0; i9 < i; i9++) {
    r17->data[i9]--;
  }

  for (i = 0; i < 20; i++) {
    b_selector[i] = !selector[i];
  }

  emxInit_int32_T(&r18, 1);
  eml_li_find(b_selector, r18);
  i9 = r18->size[0];
  r18->size[0] = r18->size[0];
  emxEnsureCapacity((emxArray__common *)r18, i9, (int32_T)sizeof(int32_T));
  i = r18->size[0];
  for (i9 = 0; i9 < i; i9++) {
    r18->data[i9]--;
  }

  for (i = 0; i < 20; i++) {
    b_selector[i] = !selector[i];
  }

  emxInit_real32_T(&a, 2);
  emxInit_int32_T(&r19, 1);
  emxInit_int32_T(&r20, 1);
  eml_li_find(b_selector, r19);
  eml_li_find(selector, r20);
  i9 = a->size[0] * a->size[1];
  a->size[0] = r20->size[0];
  a->size[1] = r19->size[0];
  emxEnsureCapacity((emxArray__common *)a, i9, (int32_T)sizeof(real32_T));
  i = r19->size[0];
  for (i9 = 0; i9 < i; i9++) {
    iy = r20->size[0];
    for (i10 = 0; i10 < iy; i10++) {
      a->data[i10 + a->size[0] * i9] = P[(r20->data[i10] + 20 * (r19->data[i9] -
        1)) - 1];
    }
  }

  emxInit_int32_T(&r21, 1);
  i9 = r21->size[0];
  r21->size[0] = r18->size[0];
  emxEnsureCapacity((emxArray__common *)r21, i9, (int32_T)sizeof(int32_T));
  i = r18->size[0];
  for (i9 = 0; i9 < i; i9++) {
    r21->data[i9] = r18->data[i9];
  }

  emxInit_int32_T(&r22, 1);
  i9 = r22->size[0];
  r22->size[0] = r17->size[0];
  emxEnsureCapacity((emxArray__common *)r22, i9, (int32_T)sizeof(int32_T));
  i = r17->size[0];
  for (i9 = 0; i9 < i; i9++) {
    r22->data[i9] = r17->data[i9];
  }

  i = a->size[1];
  for (i9 = 0; i9 < i; i9++) {
    iy = a->size[0];
    for (i10 = 0; i10 < iy; i10++) {
      P[r22->data[i10] + 20 * r21->data[i9]] = a->data[i10 + a->size[0] * i9] *
        0.0F;
    }
  }

  emxFree_int32_T(&r22);
  emxFree_int32_T(&r21);
  for (i = 0; i < 20; i++) {
    b_selector[i] = !selector[i];
  }

  eml_li_find(b_selector, r17);
  i9 = r17->size[0];
  r17->size[0] = r17->size[0];
  emxEnsureCapacity((emxArray__common *)r17, i9, (int32_T)sizeof(int32_T));
  i = r17->size[0];
  for (i9 = 0; i9 < i; i9++) {
    r17->data[i9]--;
  }

  eml_li_find(selector, r18);
  i9 = r18->size[0];
  r18->size[0] = r18->size[0];
  emxEnsureCapacity((emxArray__common *)r18, i9, (int32_T)sizeof(int32_T));
  i = r18->size[0];
  for (i9 = 0; i9 < i; i9++) {
    r18->data[i9]--;
  }

  eml_li_find(selector, r19);
  for (i = 0; i < 20; i++) {
    b_selector[i] = !selector[i];
  }

  eml_li_find(b_selector, r20);
  i9 = a->size[0] * a->size[1];
  a->size[0] = r20->size[0];
  a->size[1] = r19->size[0];
  emxEnsureCapacity((emxArray__common *)a, i9, (int32_T)sizeof(real32_T));
  i = r19->size[0];
  for (i9 = 0; i9 < i; i9++) {
    iy = r20->size[0];
    for (i10 = 0; i10 < iy; i10++) {
      a->data[i10 + a->size[0] * i9] = P[(r20->data[i10] + 20 * (r19->data[i9] -
        1)) - 1];
    }
  }

  emxFree_int32_T(&r20);
  emxInit_int32_T(&r23, 1);
  i9 = r23->size[0];
  r23->size[0] = r18->size[0];
  emxEnsureCapacity((emxArray__common *)r23, i9, (int32_T)sizeof(int32_T));
  i = r18->size[0];
  for (i9 = 0; i9 < i; i9++) {
    r23->data[i9] = r18->data[i9];
  }

  emxInit_int32_T(&r24, 1);
  i9 = r24->size[0];
  r24->size[0] = r17->size[0];
  emxEnsureCapacity((emxArray__common *)r24, i9, (int32_T)sizeof(int32_T));
  i = r17->size[0];
  for (i9 = 0; i9 < i; i9++) {
    r24->data[i9] = r17->data[i9];
  }

  i = a->size[1];
  for (i9 = 0; i9 < i; i9++) {
    iy = a->size[0];
    for (i10 = 0; i10 < iy; i10++) {
      P[r24->data[i10] + 20 * r23->data[i9]] = a->data[i10 + a->size[0] * i9] *
        0.0F;
    }
  }

  emxFree_int32_T(&r24);
  emxFree_int32_T(&r23);

  /* P(~selector,~selector)=P(~selector,~selector)*0; */
  /* H=zeros(3,size(P,1));                                 % commented   */
  /* H(1:3,7:9)=eye(3);                                    % commented   */
  for (i9 = 0; i9 < 60; i9++) {
    H[i9] = 0;
  }

  /*  set float type */
  for (i9 = 0; i9 < 3; i9++) {
    for (i10 = 0; i10 < 3; i10++) {
      H[i10 + 3 * (6 + i9)] = iv3[i10 + 3 * i9];
    }
  }

  /*  set float type */
  eml_li_find(selector, r17);
  i9 = r17->size[0];
  r17->size[0] = r17->size[0];
  emxEnsureCapacity((emxArray__common *)r17, i9, (int32_T)sizeof(int32_T));
  i = r17->size[0];
  for (i9 = 0; i9 < i; i9++) {
    r17->data[i9]--;
  }

  emxInit_real32_T(&b_a, 2);
  eml_li_find(selector, r18);
  i9 = b_a->size[0] * b_a->size[1];
  b_a->size[0] = 3;
  b_a->size[1] = r18->size[0];
  emxEnsureCapacity((emxArray__common *)b_a, i9, (int32_T)sizeof(real32_T));
  i = r18->size[0];
  for (i9 = 0; i9 < i; i9++) {
    for (i10 = 0; i10 < 3; i10++) {
      b_a->data[i10 + b_a->size[0] * i9] = (real32_T)H[i10 + 3 * (r18->data[i9]
        - 1)];
    }
  }

  emxInit_real32_T(&b, 2);
  eml_li_find(selector, r18);
  eml_li_find(selector, r19);
  i9 = b->size[0] * b->size[1];
  b->size[0] = r19->size[0];
  b->size[1] = r18->size[0];
  emxEnsureCapacity((emxArray__common *)b, i9, (int32_T)sizeof(real32_T));
  i = r18->size[0];
  for (i9 = 0; i9 < i; i9++) {
    iy = r19->size[0];
    for (i10 = 0; i10 < iy; i10++) {
      b->data[i10 + b->size[0] * i9] = P[(r19->data[i10] + 20 * (r18->data[i9] -
        1)) - 1];
    }
  }

  emxFree_int32_T(&r19);
  emxInit_real32_T(&y, 2);
  if ((b_a->size[1] == 1) || (b->size[0] == 1)) {
    i9 = y->size[0] * y->size[1];
    y->size[0] = 3;
    y->size[1] = b->size[1];
    emxEnsureCapacity((emxArray__common *)y, i9, (int32_T)sizeof(real32_T));
    for (i9 = 0; i9 < 3; i9++) {
      i = b->size[1];
      for (i10 = 0; i10 < i; i10++) {
        y->data[i9 + y->size[0] * i10] = 0.0F;
        iy = b_a->size[1];
        for (ar = 0; ar < iy; ar++) {
          y->data[i9 + y->size[0] * i10] += b_a->data[i9 + b_a->size[0] * ar] *
            b->data[ar + b->size[0] * i10];
        }
      }
    }
  } else {
    unnamed_idx_1 = (uint32_T)b->size[1];
    i9 = y->size[0] * y->size[1];
    y->size[0] = 3;
    emxEnsureCapacity((emxArray__common *)y, i9, (int32_T)sizeof(real32_T));
    i9 = y->size[0] * y->size[1];
    y->size[1] = (int32_T)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)y, i9, (int32_T)sizeof(real32_T));
    i = 3 * (int32_T)unnamed_idx_1;
    for (i9 = 0; i9 < i; i9++) {
      y->data[i9] = 0.0F;
    }

    if (b->size[1] == 0) {
    } else {
      i = 3 * (b->size[1] - 1);
      for (iy = 0; iy <= i; iy += 3) {
        for (ic = iy; ic + 1 <= iy + 3; ic++) {
          y->data[ic] = 0.0F;
        }
      }

      br = 0;
      for (iy = 0; iy <= i; iy += 3) {
        ar = 0;
        i9 = br + b_a->size[1];
        for (ib = br; ib + 1 <= i9; ib++) {
          if (b->data[ib] != 0.0F) {
            ia = ar;
            for (ic = iy; ic + 1 <= iy + 3; ic++) {
              ia++;
              y->data[ic] += b->data[ib] * b_a->data[ia - 1];
            }
          }

          ar += 3;
        }

        br += b_a->size[1];
      }
    }
  }

  emxInit_real32_T(&c_a, 2);
  mldivide(S, y, b_a);
  i9 = c_a->size[0] * c_a->size[1];
  c_a->size[0] = b_a->size[1];
  c_a->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)c_a, i9, (int32_T)sizeof(real32_T));
  emxFree_real32_T(&y);
  for (i9 = 0; i9 < 3; i9++) {
    i = b_a->size[1];
    for (i10 = 0; i10 < i; i10++) {
      c_a->data[i10 + c_a->size[0] * i9] = b_a->data[i9 + b_a->size[0] * i10];
    }
  }

  for (i9 = 0; i9 < 3; i9++) {
    i = c_a->size[0];
    for (i10 = 0; i10 < i; i10++) {
      K[r17->data[i10] + 20 * i9] = c_a->data[i10 + c_a->size[0] * i9];
    }
  }

  /*  Outline for the velocity inovation: */
  for (i = 0; i < 3; i++) {
    e[i] = y_VelNed[i] - x->v_N[i];
  }

  pinv(S, b_b);
  chi2 = 0.0F;
  for (i9 = 0; i9 < 3; i9++) {
    b_y[i9] = 0.0F;
    for (i10 = 0; i10 < 3; i10++) {
      b_y[i9] += e[i10] * b_b[i10 + 3 * i9];
    }

    chi2 += b_y[i9] * e[i9];
  }

  /* disp(e); */
  if (chi2 > 25.0F) {
    /* ~0.7*0.7/(P+0.01)~25 */
    /* disp('updateVelocity chi2-test fail'); */
  } else {
    for (i9 = 0; i9 < 20; i9++) {
      delta_x[i9] = 0.0F;
      for (i10 = 0; i10 < 3; i10++) {
        delta_x[i9] += K[i9 + 20 * i10] * e[i10];
      }
    }

    /*  update: */
    eml_li_find(selector, r17);
    i9 = c_a->size[0] * c_a->size[1];
    c_a->size[0] = r17->size[0];
    c_a->size[1] = 3;
    emxEnsureCapacity((emxArray__common *)c_a, i9, (int32_T)sizeof(real32_T));
    for (i9 = 0; i9 < 3; i9++) {
      i = r17->size[0];
      for (i10 = 0; i10 < i; i10++) {
        c_a->data[i10 + c_a->size[0] * i9] = K[(r17->data[i10] + 20 * i9) - 1];
      }
    }

    eml_li_find(selector, r17);
    i9 = b_a->size[0] * b_a->size[1];
    b_a->size[0] = 3;
    b_a->size[1] = r17->size[0];
    emxEnsureCapacity((emxArray__common *)b_a, i9, (int32_T)sizeof(real32_T));
    i = r17->size[0];
    for (i9 = 0; i9 < i; i9++) {
      for (i10 = 0; i10 < 3; i10++) {
        b_a->data[i10 + b_a->size[0] * i9] = (real32_T)H[i10 + 3 * (r17->data[i9]
          - 1)];
      }
    }

    emxInit_real32_T(&KH, 2);
    unnamed_idx_0 = (uint32_T)c_a->size[0];
    unnamed_idx_1 = (uint32_T)b_a->size[1];
    i9 = KH->size[0] * KH->size[1];
    KH->size[0] = (int32_T)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)KH, i9, (int32_T)sizeof(real32_T));
    i9 = KH->size[0] * KH->size[1];
    KH->size[1] = (int32_T)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)KH, i9, (int32_T)sizeof(real32_T));
    i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
    for (i9 = 0; i9 < i; i9++) {
      KH->data[i9] = 0.0F;
    }

    if ((c_a->size[0] == 0) || (b_a->size[1] == 0)) {
    } else {
      i = c_a->size[0] * (b_a->size[1] - 1);
      for (iy = 0; iy <= i; iy += c_a->size[0]) {
        i9 = iy + c_a->size[0];
        for (ic = iy; ic + 1 <= i9; ic++) {
          KH->data[ic] = 0.0F;
        }
      }

      br = 0;
      for (iy = 0; iy <= i; iy += c_a->size[0]) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 3; ib++) {
          if (b_a->data[ib] != 0.0F) {
            ia = ar;
            i9 = iy + c_a->size[0];
            for (ic = iy; ic + 1 <= i9; ic++) {
              ia++;
              KH->data[ic] += b_a->data[ib] * c_a->data[ia - 1];
            }
          }

          ar += c_a->size[0];
        }

        br += 3;
      }
    }

    emxInit_real32_T(&r25, 2);
    eml_li_find(selector, r17);
    eml_li_find(selector, r18);
    i9 = r25->size[0] * r25->size[1];
    r25->size[0] = r18->size[0];
    r25->size[1] = r17->size[0];
    emxEnsureCapacity((emxArray__common *)r25, i9, (int32_T)sizeof(real32_T));
    i = r17->size[0];
    for (i9 = 0; i9 < i; i9++) {
      iy = r18->size[0];
      for (i10 = 0; i10 < iy; i10++) {
        r25->data[i10 + r25->size[0] * i9] = P[(r18->data[i10] + 20 * (r17->
          data[i9] - 1)) - 1];
      }
    }

    eml_li_find(selector, r17);
    eml_li_find(selector, r18);
    i9 = b->size[0] * b->size[1];
    b->size[0] = r18->size[0];
    b->size[1] = r17->size[0];
    emxEnsureCapacity((emxArray__common *)b, i9, (int32_T)sizeof(real32_T));
    i = r17->size[0];
    for (i9 = 0; i9 < i; i9++) {
      iy = r18->size[0];
      for (i10 = 0; i10 < iy; i10++) {
        b->data[i10 + b->size[0] * i9] = P[(r18->data[i10] + 20 * (r17->data[i9]
          - 1)) - 1];
      }
    }

    emxInit_real32_T(&C, 2);
    if ((KH->size[1] == 1) || (b->size[0] == 1)) {
      i9 = C->size[0] * C->size[1];
      C->size[0] = KH->size[0];
      C->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)C, i9, (int32_T)sizeof(real32_T));
      i = KH->size[0];
      for (i9 = 0; i9 < i; i9++) {
        iy = b->size[1];
        for (i10 = 0; i10 < iy; i10++) {
          C->data[i9 + C->size[0] * i10] = 0.0F;
          br = KH->size[1];
          for (ar = 0; ar < br; ar++) {
            C->data[i9 + C->size[0] * i10] += KH->data[i9 + KH->size[0] * ar] *
              b->data[ar + b->size[0] * i10];
          }
        }
      }
    } else {
      unnamed_idx_0 = (uint32_T)KH->size[0];
      unnamed_idx_1 = (uint32_T)b->size[1];
      i9 = C->size[0] * C->size[1];
      C->size[0] = (int32_T)unnamed_idx_0;
      emxEnsureCapacity((emxArray__common *)C, i9, (int32_T)sizeof(real32_T));
      i9 = C->size[0] * C->size[1];
      C->size[1] = (int32_T)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)C, i9, (int32_T)sizeof(real32_T));
      i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
      for (i9 = 0; i9 < i; i9++) {
        C->data[i9] = 0.0F;
      }

      if ((KH->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        i = KH->size[0] * (b->size[1] - 1);
        for (iy = 0; iy <= i; iy += KH->size[0]) {
          i9 = iy + KH->size[0];
          for (ic = iy; ic + 1 <= i9; ic++) {
            C->data[ic] = 0.0F;
          }
        }

        br = 0;
        for (iy = 0; iy <= i; iy += KH->size[0]) {
          ar = 0;
          i9 = br + KH->size[1];
          for (ib = br; ib + 1 <= i9; ib++) {
            if (b->data[ib] != 0.0F) {
              ia = ar;
              i10 = iy + KH->size[0];
              for (ic = iy; ic + 1 <= i10; ic++) {
                ia++;
                C->data[ic] += b->data[ib] * KH->data[ia - 1];
              }
            }

            ar += KH->size[0];
          }

          br += KH->size[1];
        }
      }
    }

    eml_li_find(selector, r17);
    eml_li_find(selector, r18);
    i9 = a->size[0] * a->size[1];
    a->size[0] = r18->size[0];
    a->size[1] = r17->size[0];
    emxEnsureCapacity((emxArray__common *)a, i9, (int32_T)sizeof(real32_T));
    i = r17->size[0];
    for (i9 = 0; i9 < i; i9++) {
      iy = r18->size[0];
      for (i10 = 0; i10 < iy; i10++) {
        a->data[i10 + a->size[0] * i9] = P[(r18->data[i10] + 20 * (r17->data[i9]
          - 1)) - 1];
      }
    }

    i9 = b->size[0] * b->size[1];
    b->size[0] = KH->size[1];
    b->size[1] = KH->size[0];
    emxEnsureCapacity((emxArray__common *)b, i9, (int32_T)sizeof(real32_T));
    i = KH->size[0];
    for (i9 = 0; i9 < i; i9++) {
      iy = KH->size[1];
      for (i10 = 0; i10 < iy; i10++) {
        b->data[i10 + b->size[0] * i9] = KH->data[i9 + KH->size[0] * i10];
      }
    }

    emxInit_real32_T(&b_C, 2);
    if ((a->size[1] == 1) || (b->size[0] == 1)) {
      i9 = b_C->size[0] * b_C->size[1];
      b_C->size[0] = a->size[0];
      b_C->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)b_C, i9, (int32_T)sizeof(real32_T));
      i = a->size[0];
      for (i9 = 0; i9 < i; i9++) {
        iy = b->size[1];
        for (i10 = 0; i10 < iy; i10++) {
          b_C->data[i9 + b_C->size[0] * i10] = 0.0F;
          br = a->size[1];
          for (ar = 0; ar < br; ar++) {
            b_C->data[i9 + b_C->size[0] * i10] += a->data[i9 + a->size[0] * ar] *
              b->data[ar + b->size[0] * i10];
          }
        }
      }
    } else {
      unnamed_idx_0 = (uint32_T)a->size[0];
      unnamed_idx_1 = (uint32_T)b->size[1];
      i9 = b_C->size[0] * b_C->size[1];
      b_C->size[0] = (int32_T)unnamed_idx_0;
      emxEnsureCapacity((emxArray__common *)b_C, i9, (int32_T)sizeof(real32_T));
      i9 = b_C->size[0] * b_C->size[1];
      b_C->size[1] = (int32_T)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)b_C, i9, (int32_T)sizeof(real32_T));
      i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
      for (i9 = 0; i9 < i; i9++) {
        b_C->data[i9] = 0.0F;
      }

      if ((a->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        i = a->size[0] * (b->size[1] - 1);
        for (iy = 0; iy <= i; iy += a->size[0]) {
          i9 = iy + a->size[0];
          for (ic = iy; ic + 1 <= i9; ic++) {
            b_C->data[ic] = 0.0F;
          }
        }

        br = 0;
        for (iy = 0; iy <= i; iy += a->size[0]) {
          ar = 0;
          i9 = br + a->size[1];
          for (ib = br; ib + 1 <= i9; ib++) {
            if (b->data[ib] != 0.0F) {
              ia = ar;
              i10 = iy + a->size[0];
              for (ic = iy; ic + 1 <= i10; ic++) {
                ia++;
                b_C->data[ic] += b->data[ib] * a->data[ia - 1];
              }
            }

            ar += a->size[0];
          }

          br += a->size[1];
        }
      }
    }

    eml_li_find(selector, r17);
    i9 = c_a->size[0] * c_a->size[1];
    c_a->size[0] = r17->size[0];
    c_a->size[1] = 3;
    emxEnsureCapacity((emxArray__common *)c_a, i9, (int32_T)sizeof(real32_T));
    for (i9 = 0; i9 < 3; i9++) {
      i = r17->size[0];
      for (i10 = 0; i10 < i; i10++) {
        c_a->data[i10 + c_a->size[0] * i9] = K[(r17->data[i10] + 20 * i9) - 1];
      }
    }

    emxInit_real32_T(&c_y, 2);
    unnamed_idx_0 = (uint32_T)c_a->size[0];
    i9 = c_y->size[0] * c_y->size[1];
    c_y->size[0] = (int32_T)unnamed_idx_0;
    c_y->size[1] = 3;
    emxEnsureCapacity((emxArray__common *)c_y, i9, (int32_T)sizeof(real32_T));
    i = (int32_T)unnamed_idx_0 * 3;
    for (i9 = 0; i9 < i; i9++) {
      c_y->data[i9] = 0.0F;
    }

    if (c_a->size[0] == 0) {
    } else {
      i = c_a->size[0] << 1;
      for (iy = 0; iy <= i; iy += c_a->size[0]) {
        i9 = iy + c_a->size[0];
        for (ic = iy; ic + 1 <= i9; ic++) {
          c_y->data[ic] = 0.0F;
        }
      }

      br = 0;
      for (iy = 0; iy <= i; iy += c_a->size[0]) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 3; ib++) {
          if (S[ib] != 0.0F) {
            ia = ar;
            i9 = iy + c_a->size[0];
            for (ic = iy; ic + 1 <= i9; ic++) {
              ia++;
              c_y->data[ic] += S[ib] * c_a->data[ia - 1];
            }
          }

          ar += c_a->size[0];
        }

        br += 3;
      }
    }

    eml_li_find(selector, r17);
    i9 = b_a->size[0] * b_a->size[1];
    b_a->size[0] = 3;
    b_a->size[1] = r17->size[0];
    emxEnsureCapacity((emxArray__common *)b_a, i9, (int32_T)sizeof(real32_T));
    i = r17->size[0];
    for (i9 = 0; i9 < i; i9++) {
      for (i10 = 0; i10 < 3; i10++) {
        b_a->data[i10 + b_a->size[0] * i9] = K[(r17->data[i9] + 20 * i10) - 1];
      }
    }

    unnamed_idx_0 = (uint32_T)c_y->size[0];
    unnamed_idx_1 = (uint32_T)b_a->size[1];
    i9 = KH->size[0] * KH->size[1];
    KH->size[0] = (int32_T)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)KH, i9, (int32_T)sizeof(real32_T));
    i9 = KH->size[0] * KH->size[1];
    KH->size[1] = (int32_T)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)KH, i9, (int32_T)sizeof(real32_T));
    i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
    for (i9 = 0; i9 < i; i9++) {
      KH->data[i9] = 0.0F;
    }

    if ((c_y->size[0] == 0) || (b_a->size[1] == 0)) {
    } else {
      i = c_y->size[0] * (b_a->size[1] - 1);
      for (iy = 0; iy <= i; iy += c_y->size[0]) {
        i9 = iy + c_y->size[0];
        for (ic = iy; ic + 1 <= i9; ic++) {
          KH->data[ic] = 0.0F;
        }
      }

      br = 0;
      for (iy = 0; iy <= i; iy += c_y->size[0]) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 3; ib++) {
          if (b_a->data[ib] != 0.0F) {
            ia = ar;
            i9 = iy + c_y->size[0];
            for (ic = iy; ic + 1 <= i9; ic++) {
              ia++;
              KH->data[ic] += b_a->data[ib] * c_y->data[ia - 1];
            }
          }

          ar += c_y->size[0];
        }

        br += 3;
      }
    }

    emxFree_real32_T(&c_y);
    eml_li_find(selector, r17);
    i9 = r17->size[0];
    r17->size[0] = r17->size[0];
    emxEnsureCapacity((emxArray__common *)r17, i9, (int32_T)sizeof(int32_T));
    i = r17->size[0];
    for (i9 = 0; i9 < i; i9++) {
      r17->data[i9]--;
    }

    eml_li_find(selector, r18);
    i9 = r18->size[0];
    r18->size[0] = r18->size[0];
    emxEnsureCapacity((emxArray__common *)r18, i9, (int32_T)sizeof(int32_T));
    i = r18->size[0];
    for (i9 = 0; i9 < i; i9++) {
      r18->data[i9]--;
    }

    emxInit_int32_T(&r26, 1);
    i9 = r26->size[0];
    r26->size[0] = r18->size[0];
    emxEnsureCapacity((emxArray__common *)r26, i9, (int32_T)sizeof(int32_T));
    i = r18->size[0];
    for (i9 = 0; i9 < i; i9++) {
      r26->data[i9] = r18->data[i9];
    }

    emxInit_int32_T(&r27, 1);
    i9 = r27->size[0];
    r27->size[0] = r17->size[0];
    emxEnsureCapacity((emxArray__common *)r27, i9, (int32_T)sizeof(int32_T));
    i = r17->size[0];
    for (i9 = 0; i9 < i; i9++) {
      r27->data[i9] = r17->data[i9];
    }

    i = r25->size[1];
    for (i9 = 0; i9 < i; i9++) {
      iy = r25->size[0];
      for (i10 = 0; i10 < iy; i10++) {
        P[r27->data[i10] + 20 * r26->data[i9]] = ((r25->data[i10 + r25->size[0] *
          i9] - C->data[i10 + C->size[0] * i9]) - b_C->data[i10 + b_C->size[0] *
          i9]) + KH->data[i10 + KH->size[0] * i9];
      }
    }

    emxFree_int32_T(&r27);
    emxFree_int32_T(&r26);
    emxFree_real32_T(&b_C);
    emxFree_real32_T(&C);
    emxFree_real32_T(&r25);
    emxFree_real32_T(&KH);

    /* delta_x=(K*(y_VelNed-x.v_N)); */
    /*  effective state update: */
    for (i9 = 0; i9 < 3; i9++) {
      x->p[i9] += (real_T)delta_x[i9];
    }

    for (i = 0; i < 3; i++) {
      e[i] = 0.5F * delta_x[i + 3];
    }

    dtheta_half_norm = (real32_T)sqrt(dot(e, e));
    if ((real32_T)fabs(dtheta_half_norm) < 0.01F) {
      chi2 = dtheta_half_norm * dtheta_half_norm;
      chi2 = ((1.0F - 0.166666672F * chi2) + 0.00833333377F * (chi2 * chi2)) -
        0.000198412701F * (chi2 * chi2 * chi2);
    } else {
      chi2 = (real32_T)sin(dtheta_half_norm) / dtheta_half_norm;
    }

    for (i = 0; i < 3; i++) {
      dq[i] = chi2 * e[i];
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
    for (i9 = 0; i9 < 4; i9++) {
      dq[i9] = 0.0F;
      for (i10 = 0; i10 < 4; i10++) {
        chi2 = dq[i9] + Q[i9 + (i10 << 2)] * x->q_NS[i10];
        dq[i9] = chi2;
      }
    }

    chi2 = 0.0F;
    i = 0;
    iy = 0;
    for (br = 0; br < 4; br++) {
      chi2 += dq[i] * dq[iy];
      i++;
      iy++;
    }

    chi2 = (real32_T)sqrt(chi2);
    for (i9 = 0; i9 < 4; i9++) {
      dq[i9] /= chi2;
    }

    for (i = 0; i < 4; i++) {
      x->q_NS[i] = dq[i];
    }

    for (i9 = 0; i9 < 3; i9++) {
      x->v_N[i9] += delta_x[6 + i9];
    }

    for (i9 = 0; i9 < 3; i9++) {
      x->b_g[i9] += delta_x[9 + i9];
    }

    for (i9 = 0; i9 < 3; i9++) {
      x->b_a[i9] += delta_x[12 + i9];
    }

    x->QFF += delta_x[15];
    for (i9 = 0; i9 < 3; i9++) {
      x->w[i9] += delta_x[16 + i9];
    }

    x->K += delta_x[19];
  }

  emxFree_int32_T(&r17);
  emxFree_int32_T(&r18);
  emxFree_real32_T(&c_a);
  emxFree_real32_T(&b);
  emxFree_real32_T(&b_a);
  emxFree_real32_T(&a);
}

/* End of code generation (updateVelNed.c) */
