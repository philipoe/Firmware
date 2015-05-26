/*
 * updatePressures.c
 *
 * Code generation for function 'updatePressures'
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
#include "HyEst_emxutil.h"
#include "dot.h"
#include "HyEst_rtwutil.h"
#include "HyEst_data.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
boolean_T b_updatePressures(states_T *x, real32_T P[400], real32_T y_p_stat,
  real32_T T_h, real32_T ell_offset, boolean_T constQFF, boolean_T constK)
{
  boolean_T p_stat_valid;
  real32_T dPdP0;
  int32_T i19;
  real32_T H[20];
  real32_T b_H[2];
  real32_T b_P[4];
  real32_T c_H[2];
  real32_T y;
  real32_T b_y[2];
  int32_T i20;
  real32_T b[2];
  real32_T S;
  real32_T K[20];
  boolean_T selector[20];
  int32_T i;
  emxArray_int32_T *r61;
  boolean_T b_selector[20];
  emxArray_int32_T *r62;
  emxArray_real32_T *a;
  emxArray_int32_T *r63;
  emxArray_int32_T *r64;
  int32_T br;
  emxArray_int32_T *r65;
  emxArray_int32_T *r66;
  emxArray_int32_T *r67;
  emxArray_int32_T *r68;
  emxArray_real32_T *b_a;
  emxArray_real32_T *b_b;
  emxArray_real32_T *c_y;
  uint32_T unnamed_idx_1;
  int32_T ic;
  int32_T ar;
  int32_T ib;
  int32_T ia;
  emxArray_real32_T *c_a;
  emxArray_real32_T *KH;
  emxArray_real32_T *r69;
  emxArray_real32_T *C;
  uint32_T unnamed_idx_0;
  int32_T c;
  emxArray_int32_T *r70;
  emxArray_int32_T *r71;
  real32_T dtheta_half[3];
  real32_T dq[4];
  real32_T Q[16];

  /*  */
  /*  [x,P] = UPDATEPRESSURES(x,P,y_p_stat,T_h,ellipsoid_offset, ... */
  /*      constWind, constQFF,constK) */
  /*  */
  /*  Kalman filter update with static pressure. */
  /*  */
  /*  Author: lestefan */
  /*  date:  02/2013 */
  /*  */
  /*  Inputs: */
  /*    x:            states [p(lat*1e7,lon*1e7,h),q_NS,v_N,b_g,b_a,QFF,w,K] */
  /*    P:            covariance */
  /*    y_p_stat:     static pressure [kPa] */
  /*    T_h:          ambient air temperature [°K] */
  /*    ell_offset:   offset between WGS-84 and MSL [m] */
  /*    constWind:    keep wind constant [true/false] (use true on GPS out) */
  /*    constQFF:     keep QFF constant [true/false] (use true on GPS out) */
  /*    constK:       K (sideslip parameter) const. [true/false] (use true) */
  /*  */
  /*  Outputs: */
  /*    x:            updated states (see inputs) */
  /*    P:            updated covariance (see inputs) */
  /*    p_stat_valid: valid pressure [true/false] */
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
  /*  See also UPDATEPOSITION, UPDATEVELNED, UPDATECOMPASS,  */
  /*  UPDATEPRESSURES2, UPDATEPRESSURES3, PROPATATE, GETINITIALQ, MAGFIELD */
  /*  */
  /*  TODO: update in the past, re-propagate. This might not be fast enough... */
  /* eml.inline('always'); */
  p_stat_valid = FALSE;

  /*  [K/m] standard temperature gradient */
  if (y_p_stat < 0.0F) {
  } else {
    /*  some derivatives of the barometric formula */
    dPdP0 = rt_powf_snf((T_h - -0.00649F * ((real32_T)x->p[2] - ell_offset)) /
                        T_h, -5.26388407F);

    /* make sure the pressure data is valid: */
    for (i19 = 0; i19 < 2; i19++) {
      H[i19] = 0.0F;
    }

    H[2] = -x->QFF * -0.00649F * -5.26388407F / T_h * rt_powf_snf((T_h -
      -0.00649F * ((real32_T)x->p[2] - ell_offset)) / T_h, -6.26388407F);
    for (i19 = 0; i19 < 12; i19++) {
      H[i19 + 3] = 0.0F;
    }

    H[15] = dPdP0;
    for (i19 = 0; i19 < 4; i19++) {
      H[i19 + 16] = 0.0F;
    }

    b_H[0] = H[2];
    b_H[1] = H[15];
    b_P[0] = P[42];
    b_P[2] = P[302];
    b_P[1] = P[55];
    b_P[3] = P[315];
    c_H[0] = H[2];
    c_H[1] = H[15];
    y = 0.0F;
    for (i19 = 0; i19 < 2; i19++) {
      b_y[i19] = 0.0F;
      for (i20 = 0; i20 < 2; i20++) {
        b_y[i19] += b_H[i20] * b_P[i20 + (i19 << 1)];
      }

      b[i19] = c_H[i19];
      y += b_y[i19] * b[i19];
    }

    S = y + rt_powf_snf(configuration.sigma_psmd, 2.0F);
    for (i = 0; i < 20; i++) {
      K[i] = 0.0F;
      selector[i] = TRUE;
    }

    if (constK) {
      selector[19] = FALSE;
    }

    /* selector(17:18,1)=false; */
    if (constQFF) {
      selector[15] = FALSE;
    }

    emxInit_int32_T(&r61, 1);

    /*  remove correlations of fixed states with active states */
    eml_li_find(selector, r61);
    i19 = r61->size[0];
    r61->size[0] = r61->size[0];
    emxEnsureCapacity((emxArray__common *)r61, i19, (int32_T)sizeof(int32_T));
    i = r61->size[0];
    for (i19 = 0; i19 < i; i19++) {
      r61->data[i19]--;
    }

    for (i = 0; i < 20; i++) {
      b_selector[i] = !selector[i];
    }

    emxInit_int32_T(&r62, 1);
    eml_li_find(b_selector, r62);
    i19 = r62->size[0];
    r62->size[0] = r62->size[0];
    emxEnsureCapacity((emxArray__common *)r62, i19, (int32_T)sizeof(int32_T));
    i = r62->size[0];
    for (i19 = 0; i19 < i; i19++) {
      r62->data[i19]--;
    }

    for (i = 0; i < 20; i++) {
      b_selector[i] = !selector[i];
    }

    emxInit_real32_T(&a, 2);
    emxInit_int32_T(&r63, 1);
    emxInit_int32_T(&r64, 1);
    eml_li_find(b_selector, r63);
    eml_li_find(selector, r64);
    i19 = a->size[0] * a->size[1];
    a->size[0] = r64->size[0];
    a->size[1] = r63->size[0];
    emxEnsureCapacity((emxArray__common *)a, i19, (int32_T)sizeof(real32_T));
    i = r63->size[0];
    for (i19 = 0; i19 < i; i19++) {
      br = r64->size[0];
      for (i20 = 0; i20 < br; i20++) {
        a->data[i20 + a->size[0] * i19] = P[(r64->data[i20] + 20 * (r63->
          data[i19] - 1)) - 1];
      }
    }

    emxInit_int32_T(&r65, 1);
    i19 = r65->size[0];
    r65->size[0] = r62->size[0];
    emxEnsureCapacity((emxArray__common *)r65, i19, (int32_T)sizeof(int32_T));
    i = r62->size[0];
    for (i19 = 0; i19 < i; i19++) {
      r65->data[i19] = r62->data[i19];
    }

    emxInit_int32_T(&r66, 1);
    i19 = r66->size[0];
    r66->size[0] = r61->size[0];
    emxEnsureCapacity((emxArray__common *)r66, i19, (int32_T)sizeof(int32_T));
    i = r61->size[0];
    for (i19 = 0; i19 < i; i19++) {
      r66->data[i19] = r61->data[i19];
    }

    i = a->size[1];
    for (i19 = 0; i19 < i; i19++) {
      br = a->size[0];
      for (i20 = 0; i20 < br; i20++) {
        P[r66->data[i20] + 20 * r65->data[i19]] = a->data[i20 + a->size[0] * i19]
          * 0.0F;
      }
    }

    emxFree_int32_T(&r66);
    emxFree_int32_T(&r65);

    /*  update only the relevant states    */
    for (i = 0; i < 20; i++) {
      b_selector[i] = !selector[i];
    }

    eml_li_find(b_selector, r61);
    i19 = r61->size[0];
    r61->size[0] = r61->size[0];
    emxEnsureCapacity((emxArray__common *)r61, i19, (int32_T)sizeof(int32_T));
    i = r61->size[0];
    for (i19 = 0; i19 < i; i19++) {
      r61->data[i19]--;
    }

    eml_li_find(selector, r62);
    i19 = r62->size[0];
    r62->size[0] = r62->size[0];
    emxEnsureCapacity((emxArray__common *)r62, i19, (int32_T)sizeof(int32_T));
    i = r62->size[0];
    for (i19 = 0; i19 < i; i19++) {
      r62->data[i19]--;
    }

    eml_li_find(selector, r63);
    for (i = 0; i < 20; i++) {
      b_selector[i] = !selector[i];
    }

    eml_li_find(b_selector, r64);
    i19 = a->size[0] * a->size[1];
    a->size[0] = r64->size[0];
    a->size[1] = r63->size[0];
    emxEnsureCapacity((emxArray__common *)a, i19, (int32_T)sizeof(real32_T));
    i = r63->size[0];
    for (i19 = 0; i19 < i; i19++) {
      br = r64->size[0];
      for (i20 = 0; i20 < br; i20++) {
        a->data[i20 + a->size[0] * i19] = P[(r64->data[i20] + 20 * (r63->
          data[i19] - 1)) - 1];
      }
    }

    emxFree_int32_T(&r64);
    emxInit_int32_T(&r67, 1);
    i19 = r67->size[0];
    r67->size[0] = r62->size[0];
    emxEnsureCapacity((emxArray__common *)r67, i19, (int32_T)sizeof(int32_T));
    i = r62->size[0];
    for (i19 = 0; i19 < i; i19++) {
      r67->data[i19] = r62->data[i19];
    }

    emxInit_int32_T(&r68, 1);
    i19 = r68->size[0];
    r68->size[0] = r61->size[0];
    emxEnsureCapacity((emxArray__common *)r68, i19, (int32_T)sizeof(int32_T));
    i = r61->size[0];
    for (i19 = 0; i19 < i; i19++) {
      r68->data[i19] = r61->data[i19];
    }

    i = a->size[1];
    for (i19 = 0; i19 < i; i19++) {
      br = a->size[0];
      for (i20 = 0; i20 < br; i20++) {
        P[r68->data[i20] + 20 * r67->data[i19]] = a->data[i20 + a->size[0] * i19]
          * 0.0F;
      }
    }

    emxFree_int32_T(&r68);
    emxFree_int32_T(&r67);

    /*  update only the relevant states      */
    /* P(~selector,~selector)=P(~selector,~selector)*0; */
    eml_li_find(selector, r61);
    i19 = r61->size[0];
    r61->size[0] = r61->size[0];
    emxEnsureCapacity((emxArray__common *)r61, i19, (int32_T)sizeof(int32_T));
    i = r61->size[0];
    for (i19 = 0; i19 < i; i19++) {
      r61->data[i19]--;
    }

    emxInit_real32_T(&b_a, 2);
    eml_li_find(selector, r62);
    i19 = b_a->size[0] * b_a->size[1];
    b_a->size[0] = 1;
    b_a->size[1] = r62->size[0];
    emxEnsureCapacity((emxArray__common *)b_a, i19, (int32_T)sizeof(real32_T));
    i = r62->size[0];
    for (i19 = 0; i19 < i; i19++) {
      b_a->data[b_a->size[0] * i19] = H[r62->data[i19] - 1];
    }

    emxInit_real32_T(&b_b, 2);
    eml_li_find(selector, r62);
    eml_li_find(selector, r63);
    i19 = b_b->size[0] * b_b->size[1];
    b_b->size[0] = r63->size[0];
    b_b->size[1] = r62->size[0];
    emxEnsureCapacity((emxArray__common *)b_b, i19, (int32_T)sizeof(real32_T));
    i = r62->size[0];
    for (i19 = 0; i19 < i; i19++) {
      br = r63->size[0];
      for (i20 = 0; i20 < br; i20++) {
        b_b->data[i20 + b_b->size[0] * i19] = P[(r63->data[i20] + 20 *
          (r62->data[i19] - 1)) - 1];
      }
    }

    emxFree_int32_T(&r63);
    emxInit_real32_T(&c_y, 2);
    if ((b_a->size[1] == 1) || (b_b->size[0] == 1)) {
      i19 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = 1;
      c_y->size[1] = b_b->size[1];
      emxEnsureCapacity((emxArray__common *)c_y, i19, (int32_T)sizeof(real32_T));
      i = b_b->size[1];
      for (i19 = 0; i19 < i; i19++) {
        c_y->data[c_y->size[0] * i19] = 0.0F;
        br = b_a->size[1];
        for (i20 = 0; i20 < br; i20++) {
          c_y->data[c_y->size[0] * i19] += b_a->data[b_a->size[0] * i20] *
            b_b->data[i20 + b_b->size[0] * i19];
        }
      }
    } else {
      unnamed_idx_1 = (uint32_T)b_b->size[1];
      i19 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = 1;
      emxEnsureCapacity((emxArray__common *)c_y, i19, (int32_T)sizeof(real32_T));
      i19 = c_y->size[0] * c_y->size[1];
      c_y->size[1] = (int32_T)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)c_y, i19, (int32_T)sizeof(real32_T));
      i = (int32_T)unnamed_idx_1;
      for (i19 = 0; i19 < i; i19++) {
        c_y->data[i19] = 0.0F;
      }

      if (b_b->size[1] == 0) {
      } else {
        for (i = 0; i < b_b->size[1]; i++) {
          for (ic = i; ic + 1 <= i + 1; ic++) {
            c_y->data[ic] = 0.0F;
          }
        }

        br = 0;
        for (i = 0; i < b_b->size[1]; i++) {
          ar = 0;
          i19 = br + b_a->size[1];
          for (ib = br; ib + 1 <= i19; ib++) {
            if (b_b->data[ib] != 0.0F) {
              ia = ar;
              for (ic = i; ic + 1 <= i + 1; ic++) {
                ia++;
                c_y->data[ic] += b_b->data[ib] * b_a->data[ia - 1];
              }
            }

            ar++;
          }

          br += b_a->size[1];
        }
      }
    }

    i = c_y->size[1];
    for (i19 = 0; i19 < i; i19++) {
      K[r61->data[i19]] = c_y->data[i19] / S;
    }

    emxFree_real32_T(&c_y);

    /*                */
    /* K=(S\([H(1,3),H(1,16)]*[P(3,:);P(16,:)]))'; % gain */
    dPdP0 = y_p_stat - x->QFF * dPdP0;

    /* disp(e); */
    if (dPdP0 / S * dPdP0 > 25.0F) {
      /* disp(['updatePressures chi2-test fail e=' num2str(e)]) */
    } else {
      b_emxInit_real32_T(&c_a, 1);
      p_stat_valid = TRUE;

      /*  update: */
      eml_li_find(selector, r61);
      i19 = c_a->size[0];
      c_a->size[0] = r61->size[0];
      emxEnsureCapacity((emxArray__common *)c_a, i19, (int32_T)sizeof(real32_T));
      i = r61->size[0];
      for (i19 = 0; i19 < i; i19++) {
        c_a->data[i19] = K[r61->data[i19] - 1];
      }

      eml_li_find(selector, r61);
      i19 = b_a->size[0] * b_a->size[1];
      b_a->size[0] = 1;
      b_a->size[1] = r61->size[0];
      emxEnsureCapacity((emxArray__common *)b_a, i19, (int32_T)sizeof(real32_T));
      i = r61->size[0];
      for (i19 = 0; i19 < i; i19++) {
        b_a->data[b_a->size[0] * i19] = H[r61->data[i19] - 1];
      }

      emxInit_real32_T(&KH, 2);
      i19 = KH->size[0] * KH->size[1];
      KH->size[0] = c_a->size[0];
      KH->size[1] = b_a->size[1];
      emxEnsureCapacity((emxArray__common *)KH, i19, (int32_T)sizeof(real32_T));
      i = c_a->size[0];
      for (i19 = 0; i19 < i; i19++) {
        br = b_a->size[1];
        for (i20 = 0; i20 < br; i20++) {
          KH->data[i19 + KH->size[0] * i20] = c_a->data[i19] * b_a->data
            [b_a->size[0] * i20];
        }
      }

      emxInit_real32_T(&r69, 2);
      eml_li_find(selector, r61);
      eml_li_find(selector, r62);
      i19 = r69->size[0] * r69->size[1];
      r69->size[0] = r62->size[0];
      r69->size[1] = r61->size[0];
      emxEnsureCapacity((emxArray__common *)r69, i19, (int32_T)sizeof(real32_T));
      i = r61->size[0];
      for (i19 = 0; i19 < i; i19++) {
        br = r62->size[0];
        for (i20 = 0; i20 < br; i20++) {
          r69->data[i20 + r69->size[0] * i19] = P[(r62->data[i20] + 20 *
            (r61->data[i19] - 1)) - 1];
        }
      }

      eml_li_find(selector, r61);
      eml_li_find(selector, r62);
      i19 = b_b->size[0] * b_b->size[1];
      b_b->size[0] = r62->size[0];
      b_b->size[1] = r61->size[0];
      emxEnsureCapacity((emxArray__common *)b_b, i19, (int32_T)sizeof(real32_T));
      i = r61->size[0];
      for (i19 = 0; i19 < i; i19++) {
        br = r62->size[0];
        for (i20 = 0; i20 < br; i20++) {
          b_b->data[i20 + b_b->size[0] * i19] = P[(r62->data[i20] + 20 *
            (r61->data[i19] - 1)) - 1];
        }
      }

      emxInit_real32_T(&C, 2);
      if ((KH->size[1] == 1) || (b_b->size[0] == 1)) {
        i19 = C->size[0] * C->size[1];
        C->size[0] = KH->size[0];
        C->size[1] = b_b->size[1];
        emxEnsureCapacity((emxArray__common *)C, i19, (int32_T)sizeof(real32_T));
        i = KH->size[0];
        for (i19 = 0; i19 < i; i19++) {
          br = b_b->size[1];
          for (i20 = 0; i20 < br; i20++) {
            C->data[i19 + C->size[0] * i20] = 0.0F;
            ar = KH->size[1];
            for (ib = 0; ib < ar; ib++) {
              C->data[i19 + C->size[0] * i20] += KH->data[i19 + KH->size[0] * ib]
                * b_b->data[ib + b_b->size[0] * i20];
            }
          }
        }
      } else {
        unnamed_idx_0 = (uint32_T)KH->size[0];
        unnamed_idx_1 = (uint32_T)b_b->size[1];
        i19 = C->size[0] * C->size[1];
        C->size[0] = (int32_T)unnamed_idx_0;
        emxEnsureCapacity((emxArray__common *)C, i19, (int32_T)sizeof(real32_T));
        i19 = C->size[0] * C->size[1];
        C->size[1] = (int32_T)unnamed_idx_1;
        emxEnsureCapacity((emxArray__common *)C, i19, (int32_T)sizeof(real32_T));
        i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
        for (i19 = 0; i19 < i; i19++) {
          C->data[i19] = 0.0F;
        }

        if ((KH->size[0] == 0) || (b_b->size[1] == 0)) {
        } else {
          c = KH->size[0] * (b_b->size[1] - 1);
          for (i = 0; i <= c; i += KH->size[0]) {
            i19 = i + KH->size[0];
            for (ic = i; ic + 1 <= i19; ic++) {
              C->data[ic] = 0.0F;
            }
          }

          br = 0;
          for (i = 0; i <= c; i += KH->size[0]) {
            ar = 0;
            i19 = br + KH->size[1];
            for (ib = br; ib + 1 <= i19; ib++) {
              if (b_b->data[ib] != 0.0F) {
                ia = ar;
                i20 = i + KH->size[0];
                for (ic = i; ic + 1 <= i20; ic++) {
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

      eml_li_find(selector, r61);
      eml_li_find(selector, r62);
      i19 = a->size[0] * a->size[1];
      a->size[0] = r62->size[0];
      a->size[1] = r61->size[0];
      emxEnsureCapacity((emxArray__common *)a, i19, (int32_T)sizeof(real32_T));
      i = r61->size[0];
      for (i19 = 0; i19 < i; i19++) {
        br = r62->size[0];
        for (i20 = 0; i20 < br; i20++) {
          a->data[i20 + a->size[0] * i19] = P[(r62->data[i20] + 20 * (r61->
            data[i19] - 1)) - 1];
        }
      }

      i19 = b_b->size[0] * b_b->size[1];
      b_b->size[0] = KH->size[1];
      b_b->size[1] = KH->size[0];
      emxEnsureCapacity((emxArray__common *)b_b, i19, (int32_T)sizeof(real32_T));
      i = KH->size[0];
      for (i19 = 0; i19 < i; i19++) {
        br = KH->size[1];
        for (i20 = 0; i20 < br; i20++) {
          b_b->data[i20 + b_b->size[0] * i19] = KH->data[i19 + KH->size[0] * i20];
        }
      }

      if ((a->size[1] == 1) || (b_b->size[0] == 1)) {
        i19 = KH->size[0] * KH->size[1];
        KH->size[0] = a->size[0];
        KH->size[1] = b_b->size[1];
        emxEnsureCapacity((emxArray__common *)KH, i19, (int32_T)sizeof(real32_T));
        i = a->size[0];
        for (i19 = 0; i19 < i; i19++) {
          br = b_b->size[1];
          for (i20 = 0; i20 < br; i20++) {
            KH->data[i19 + KH->size[0] * i20] = 0.0F;
            ar = a->size[1];
            for (ib = 0; ib < ar; ib++) {
              KH->data[i19 + KH->size[0] * i20] += a->data[i19 + a->size[0] * ib]
                * b_b->data[ib + b_b->size[0] * i20];
            }
          }
        }
      } else {
        unnamed_idx_0 = (uint32_T)a->size[0];
        unnamed_idx_1 = (uint32_T)b_b->size[1];
        i19 = KH->size[0] * KH->size[1];
        KH->size[0] = (int32_T)unnamed_idx_0;
        emxEnsureCapacity((emxArray__common *)KH, i19, (int32_T)sizeof(real32_T));
        i19 = KH->size[0] * KH->size[1];
        KH->size[1] = (int32_T)unnamed_idx_1;
        emxEnsureCapacity((emxArray__common *)KH, i19, (int32_T)sizeof(real32_T));
        i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
        for (i19 = 0; i19 < i; i19++) {
          KH->data[i19] = 0.0F;
        }

        if ((a->size[0] == 0) || (b_b->size[1] == 0)) {
        } else {
          c = a->size[0] * (b_b->size[1] - 1);
          for (i = 0; i <= c; i += a->size[0]) {
            i19 = i + a->size[0];
            for (ic = i; ic + 1 <= i19; ic++) {
              KH->data[ic] = 0.0F;
            }
          }

          br = 0;
          for (i = 0; i <= c; i += a->size[0]) {
            ar = 0;
            i19 = br + a->size[1];
            for (ib = br; ib + 1 <= i19; ib++) {
              if (b_b->data[ib] != 0.0F) {
                ia = ar;
                i20 = i + a->size[0];
                for (ic = i; ic + 1 <= i20; ic++) {
                  ia++;
                  KH->data[ic] += b_b->data[ib] * a->data[ia - 1];
                }
              }

              ar += a->size[0];
            }

            br += a->size[1];
          }
        }
      }

      eml_li_find(selector, r61);
      i19 = c_a->size[0];
      c_a->size[0] = r61->size[0];
      emxEnsureCapacity((emxArray__common *)c_a, i19, (int32_T)sizeof(real32_T));
      i = r61->size[0];
      for (i19 = 0; i19 < i; i19++) {
        c_a->data[i19] = K[r61->data[i19] - 1];
      }

      eml_li_find(selector, r61);
      i19 = b_a->size[0] * b_a->size[1];
      b_a->size[0] = 1;
      b_a->size[1] = r61->size[0];
      emxEnsureCapacity((emxArray__common *)b_a, i19, (int32_T)sizeof(real32_T));
      i = r61->size[0];
      for (i19 = 0; i19 < i; i19++) {
        b_a->data[b_a->size[0] * i19] = K[r61->data[i19] - 1];
      }

      eml_li_find(selector, r61);
      i19 = r61->size[0];
      r61->size[0] = r61->size[0];
      emxEnsureCapacity((emxArray__common *)r61, i19, (int32_T)sizeof(int32_T));
      i = r61->size[0];
      for (i19 = 0; i19 < i; i19++) {
        r61->data[i19]--;
      }

      eml_li_find(selector, r62);
      i19 = r62->size[0];
      r62->size[0] = r62->size[0];
      emxEnsureCapacity((emxArray__common *)r62, i19, (int32_T)sizeof(int32_T));
      i = r62->size[0];
      for (i19 = 0; i19 < i; i19++) {
        r62->data[i19]--;
      }

      emxInit_int32_T(&r70, 1);
      i19 = r70->size[0];
      r70->size[0] = r62->size[0];
      emxEnsureCapacity((emxArray__common *)r70, i19, (int32_T)sizeof(int32_T));
      i = r62->size[0];
      for (i19 = 0; i19 < i; i19++) {
        r70->data[i19] = r62->data[i19];
      }

      emxInit_int32_T(&r71, 1);
      i19 = r71->size[0];
      r71->size[0] = r61->size[0];
      emxEnsureCapacity((emxArray__common *)r71, i19, (int32_T)sizeof(int32_T));
      i = r61->size[0];
      for (i19 = 0; i19 < i; i19++) {
        r71->data[i19] = r61->data[i19];
      }

      i = c_a->size[0];
      for (i19 = 0; i19 < i; i19++) {
        br = b_a->size[1];
        for (i20 = 0; i20 < br; i20++) {
          y = c_a->data[i19] * S * b_a->data[b_a->size[0] * i20];
          P[r71->data[i19] + 20 * r70->data[i20]] = ((r69->data[i19 + r69->size
            [0] * i20] - C->data[i19 + C->size[0] * i20]) - KH->data[i19 +
            KH->size[0] * i20]) + y;
        }
      }

      emxFree_int32_T(&r71);
      emxFree_int32_T(&r70);
      emxFree_real32_T(&c_a);
      emxFree_real32_T(&C);
      emxFree_real32_T(&r69);
      emxFree_real32_T(&KH);
      for (i19 = 0; i19 < 20; i19++) {
        K[i19] *= dPdP0;
      }

      /*  effective state update: */
      for (i19 = 0; i19 < 3; i19++) {
        x->p[i19] += (real_T)K[i19];
      }

      for (i = 0; i < 3; i++) {
        dtheta_half[i] = 0.5F * K[i + 3];
      }

      S = (real32_T)sqrt(dot(dtheta_half, dtheta_half));
      if ((real32_T)fabs(S) < 0.01F) {
        dPdP0 = S * S;
        y = ((1.0F - 0.166666672F * dPdP0) + 0.00833333377F * (dPdP0 * dPdP0)) -
          0.000198412701F * (dPdP0 * dPdP0 * dPdP0);
      } else {
        y = (real32_T)sin(S) / S;
      }

      for (i = 0; i < 3; i++) {
        dq[i] = y * dtheta_half[i];
      }

      dq[3] = (real32_T)cos(S);

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
      for (i19 = 0; i19 < 4; i19++) {
        dq[i19] = 0.0F;
        for (i20 = 0; i20 < 4; i20++) {
          dPdP0 = dq[i19] + Q[i19 + (i20 << 2)] * x->q_NS[i20];
          dq[i19] = dPdP0;
        }
      }

      dPdP0 = 0.0F;
      i = 0;
      br = 0;
      for (ar = 0; ar < 4; ar++) {
        dPdP0 += dq[i] * dq[br];
        i++;
        br++;
      }

      dPdP0 = (real32_T)sqrt(dPdP0);
      for (i19 = 0; i19 < 4; i19++) {
        dq[i19] /= dPdP0;
      }

      for (i = 0; i < 4; i++) {
        x->q_NS[i] = dq[i];
      }

      for (i19 = 0; i19 < 3; i19++) {
        x->v_N[i19] += K[6 + i19];
      }

      for (i19 = 0; i19 < 3; i19++) {
        x->b_g[i19] += K[9 + i19];
      }

      for (i19 = 0; i19 < 3; i19++) {
        x->b_a[i19] += K[12 + i19];
      }

      x->QFF += K[15];
      for (i19 = 0; i19 < 3; i19++) {
        x->w[i19] += K[16 + i19];
      }

      x->K += K[19];
    }

    emxFree_int32_T(&r61);
    emxFree_int32_T(&r62);
    emxFree_real32_T(&b_b);
    emxFree_real32_T(&b_a);
    emxFree_real32_T(&a);
  }

  return p_stat_valid;
}

boolean_T updatePressures(states_T *x, real32_T P[400], real32_T y_p_stat,
  real32_T T_h, real32_T ell_offset, boolean_T constWind, boolean_T constQFF,
  boolean_T constK)
{
  boolean_T p_stat_valid;
  real32_T dPdP0;
  int32_T i15;
  real32_T H[20];
  real32_T b_H[2];
  real32_T b_P[4];
  real32_T c_H[2];
  real32_T y;
  real32_T b_y[2];
  int32_T i16;
  real32_T b[2];
  real32_T S;
  real32_T K[20];
  boolean_T selector[20];
  int32_T i;
  emxArray_int32_T *r39;
  boolean_T b_selector[20];
  emxArray_int32_T *r40;
  emxArray_real32_T *a;
  emxArray_int32_T *r41;
  emxArray_int32_T *r42;
  int32_T br;
  emxArray_int32_T *r43;
  emxArray_int32_T *r44;
  emxArray_int32_T *r45;
  emxArray_int32_T *r46;
  emxArray_real32_T *b_a;
  emxArray_real32_T *b_b;
  emxArray_real32_T *c_y;
  uint32_T unnamed_idx_1;
  int32_T ic;
  int32_T ar;
  int32_T ib;
  int32_T ia;
  emxArray_real32_T *c_a;
  emxArray_real32_T *KH;
  emxArray_real32_T *r47;
  emxArray_real32_T *C;
  uint32_T unnamed_idx_0;
  int32_T c;
  emxArray_int32_T *r48;
  emxArray_int32_T *r49;
  real32_T dtheta_half[3];
  real32_T dq[4];
  real32_T Q[16];

  /*  */
  /*  [x,P] = UPDATEPRESSURES(x,P,y_p_stat,T_h,ellipsoid_offset, ... */
  /*      constWind, constQFF,constK) */
  /*  */
  /*  Kalman filter update with static pressure. */
  /*  */
  /*  Author: lestefan */
  /*  date:  02/2013 */
  /*  */
  /*  Inputs: */
  /*    x:            states [p(lat*1e7,lon*1e7,h),q_NS,v_N,b_g,b_a,QFF,w,K] */
  /*    P:            covariance */
  /*    y_p_stat:     static pressure [kPa] */
  /*    T_h:          ambient air temperature [°K] */
  /*    ell_offset:   offset between WGS-84 and MSL [m] */
  /*    constWind:    keep wind constant [true/false] (use true on GPS out) */
  /*    constQFF:     keep QFF constant [true/false] (use true on GPS out) */
  /*    constK:       K (sideslip parameter) const. [true/false] (use true) */
  /*  */
  /*  Outputs: */
  /*    x:            updated states (see inputs) */
  /*    P:            updated covariance (see inputs) */
  /*    p_stat_valid: valid pressure [true/false] */
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
  /*  See also UPDATEPOSITION, UPDATEVELNED, UPDATECOMPASS,  */
  /*  UPDATEPRESSURES2, UPDATEPRESSURES3, PROPATATE, GETINITIALQ, MAGFIELD */
  /*  */
  /*  TODO: update in the past, re-propagate. This might not be fast enough... */
  /* eml.inline('always'); */
  p_stat_valid = FALSE;

  /*  [K/m] standard temperature gradient */
  if (y_p_stat < 0.0F) {
  } else {
    /*  some derivatives of the barometric formula */
    dPdP0 = rt_powf_snf((T_h - -0.00649F * ((real32_T)x->p[2] - ell_offset)) /
                        T_h, -5.26388407F);

    /* make sure the pressure data is valid: */
    for (i15 = 0; i15 < 2; i15++) {
      H[i15] = 0.0F;
    }

    H[2] = -x->QFF * -0.00649F * -5.26388407F / T_h * rt_powf_snf((T_h -
      -0.00649F * ((real32_T)x->p[2] - ell_offset)) / T_h, -6.26388407F);
    for (i15 = 0; i15 < 12; i15++) {
      H[i15 + 3] = 0.0F;
    }

    H[15] = dPdP0;
    for (i15 = 0; i15 < 4; i15++) {
      H[i15 + 16] = 0.0F;
    }

    b_H[0] = H[2];
    b_H[1] = H[15];
    b_P[0] = P[42];
    b_P[2] = P[302];
    b_P[1] = P[55];
    b_P[3] = P[315];
    c_H[0] = H[2];
    c_H[1] = H[15];
    y = 0.0F;
    for (i15 = 0; i15 < 2; i15++) {
      b_y[i15] = 0.0F;
      for (i16 = 0; i16 < 2; i16++) {
        b_y[i15] += b_H[i16] * b_P[i16 + (i15 << 1)];
      }

      b[i15] = c_H[i15];
      y += b_y[i15] * b[i15];
    }

    S = y + rt_powf_snf(configuration.sigma_psmd, 2.0F);
    for (i = 0; i < 20; i++) {
      K[i] = 0.0F;
      selector[i] = TRUE;
    }

    if (constK) {
      selector[19] = FALSE;
    }

    if (constQFF) {
      selector[15] = FALSE;
    }

    emxInit_int32_T(&r39, 1);

    /*  remove correlations of fixed states with active states */
    eml_li_find(selector, r39);
    i15 = r39->size[0];
    r39->size[0] = r39->size[0];
    emxEnsureCapacity((emxArray__common *)r39, i15, (int32_T)sizeof(int32_T));
    i = r39->size[0];
    for (i15 = 0; i15 < i; i15++) {
      r39->data[i15]--;
    }

    for (i = 0; i < 20; i++) {
      b_selector[i] = !selector[i];
    }

    emxInit_int32_T(&r40, 1);
    eml_li_find(b_selector, r40);
    i15 = r40->size[0];
    r40->size[0] = r40->size[0];
    emxEnsureCapacity((emxArray__common *)r40, i15, (int32_T)sizeof(int32_T));
    i = r40->size[0];
    for (i15 = 0; i15 < i; i15++) {
      r40->data[i15]--;
    }

    for (i = 0; i < 20; i++) {
      b_selector[i] = !selector[i];
    }

    emxInit_real32_T(&a, 2);
    emxInit_int32_T(&r41, 1);
    emxInit_int32_T(&r42, 1);
    eml_li_find(b_selector, r41);
    eml_li_find(selector, r42);
    i15 = a->size[0] * a->size[1];
    a->size[0] = r42->size[0];
    a->size[1] = r41->size[0];
    emxEnsureCapacity((emxArray__common *)a, i15, (int32_T)sizeof(real32_T));
    i = r41->size[0];
    for (i15 = 0; i15 < i; i15++) {
      br = r42->size[0];
      for (i16 = 0; i16 < br; i16++) {
        a->data[i16 + a->size[0] * i15] = P[(r42->data[i16] + 20 * (r41->
          data[i15] - 1)) - 1];
      }
    }

    emxInit_int32_T(&r43, 1);
    i15 = r43->size[0];
    r43->size[0] = r40->size[0];
    emxEnsureCapacity((emxArray__common *)r43, i15, (int32_T)sizeof(int32_T));
    i = r40->size[0];
    for (i15 = 0; i15 < i; i15++) {
      r43->data[i15] = r40->data[i15];
    }

    emxInit_int32_T(&r44, 1);
    i15 = r44->size[0];
    r44->size[0] = r39->size[0];
    emxEnsureCapacity((emxArray__common *)r44, i15, (int32_T)sizeof(int32_T));
    i = r39->size[0];
    for (i15 = 0; i15 < i; i15++) {
      r44->data[i15] = r39->data[i15];
    }

    i = a->size[1];
    for (i15 = 0; i15 < i; i15++) {
      br = a->size[0];
      for (i16 = 0; i16 < br; i16++) {
        P[r44->data[i16] + 20 * r43->data[i15]] = a->data[i16 + a->size[0] * i15]
          * 0.0F;
      }
    }

    emxFree_int32_T(&r44);
    emxFree_int32_T(&r43);

    /*  update only the relevant states    */
    for (i = 0; i < 20; i++) {
      b_selector[i] = !selector[i];
    }

    eml_li_find(b_selector, r39);
    i15 = r39->size[0];
    r39->size[0] = r39->size[0];
    emxEnsureCapacity((emxArray__common *)r39, i15, (int32_T)sizeof(int32_T));
    i = r39->size[0];
    for (i15 = 0; i15 < i; i15++) {
      r39->data[i15]--;
    }

    eml_li_find(selector, r40);
    i15 = r40->size[0];
    r40->size[0] = r40->size[0];
    emxEnsureCapacity((emxArray__common *)r40, i15, (int32_T)sizeof(int32_T));
    i = r40->size[0];
    for (i15 = 0; i15 < i; i15++) {
      r40->data[i15]--;
    }

    eml_li_find(selector, r41);
    for (i = 0; i < 20; i++) {
      b_selector[i] = !selector[i];
    }

    eml_li_find(b_selector, r42);
    i15 = a->size[0] * a->size[1];
    a->size[0] = r42->size[0];
    a->size[1] = r41->size[0];
    emxEnsureCapacity((emxArray__common *)a, i15, (int32_T)sizeof(real32_T));
    i = r41->size[0];
    for (i15 = 0; i15 < i; i15++) {
      br = r42->size[0];
      for (i16 = 0; i16 < br; i16++) {
        a->data[i16 + a->size[0] * i15] = P[(r42->data[i16] + 20 * (r41->
          data[i15] - 1)) - 1];
      }
    }

    emxFree_int32_T(&r42);
    emxInit_int32_T(&r45, 1);
    i15 = r45->size[0];
    r45->size[0] = r40->size[0];
    emxEnsureCapacity((emxArray__common *)r45, i15, (int32_T)sizeof(int32_T));
    i = r40->size[0];
    for (i15 = 0; i15 < i; i15++) {
      r45->data[i15] = r40->data[i15];
    }

    emxInit_int32_T(&r46, 1);
    i15 = r46->size[0];
    r46->size[0] = r39->size[0];
    emxEnsureCapacity((emxArray__common *)r46, i15, (int32_T)sizeof(int32_T));
    i = r39->size[0];
    for (i15 = 0; i15 < i; i15++) {
      r46->data[i15] = r39->data[i15];
    }

    i = a->size[1];
    for (i15 = 0; i15 < i; i15++) {
      br = a->size[0];
      for (i16 = 0; i16 < br; i16++) {
        P[r46->data[i16] + 20 * r45->data[i15]] = a->data[i16 + a->size[0] * i15]
          * 0.0F;
      }
    }

    emxFree_int32_T(&r46);
    emxFree_int32_T(&r45);

    /*  update only the relevant states      */
    /* P(~selector,~selector)=P(~selector,~selector)*0; */
    eml_li_find(selector, r39);
    i15 = r39->size[0];
    r39->size[0] = r39->size[0];
    emxEnsureCapacity((emxArray__common *)r39, i15, (int32_T)sizeof(int32_T));
    i = r39->size[0];
    for (i15 = 0; i15 < i; i15++) {
      r39->data[i15]--;
    }

    emxInit_real32_T(&b_a, 2);
    eml_li_find(selector, r40);
    i15 = b_a->size[0] * b_a->size[1];
    b_a->size[0] = 1;
    b_a->size[1] = r40->size[0];
    emxEnsureCapacity((emxArray__common *)b_a, i15, (int32_T)sizeof(real32_T));
    i = r40->size[0];
    for (i15 = 0; i15 < i; i15++) {
      b_a->data[b_a->size[0] * i15] = H[r40->data[i15] - 1];
    }

    emxInit_real32_T(&b_b, 2);
    eml_li_find(selector, r40);
    eml_li_find(selector, r41);
    i15 = b_b->size[0] * b_b->size[1];
    b_b->size[0] = r41->size[0];
    b_b->size[1] = r40->size[0];
    emxEnsureCapacity((emxArray__common *)b_b, i15, (int32_T)sizeof(real32_T));
    i = r40->size[0];
    for (i15 = 0; i15 < i; i15++) {
      br = r41->size[0];
      for (i16 = 0; i16 < br; i16++) {
        b_b->data[i16 + b_b->size[0] * i15] = P[(r41->data[i16] + 20 *
          (r40->data[i15] - 1)) - 1];
      }
    }

    emxFree_int32_T(&r41);
    emxInit_real32_T(&c_y, 2);
    if ((b_a->size[1] == 1) || (b_b->size[0] == 1)) {
      i15 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = 1;
      c_y->size[1] = b_b->size[1];
      emxEnsureCapacity((emxArray__common *)c_y, i15, (int32_T)sizeof(real32_T));
      i = b_b->size[1];
      for (i15 = 0; i15 < i; i15++) {
        c_y->data[c_y->size[0] * i15] = 0.0F;
        br = b_a->size[1];
        for (i16 = 0; i16 < br; i16++) {
          c_y->data[c_y->size[0] * i15] += b_a->data[b_a->size[0] * i16] *
            b_b->data[i16 + b_b->size[0] * i15];
        }
      }
    } else {
      unnamed_idx_1 = (uint32_T)b_b->size[1];
      i15 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = 1;
      emxEnsureCapacity((emxArray__common *)c_y, i15, (int32_T)sizeof(real32_T));
      i15 = c_y->size[0] * c_y->size[1];
      c_y->size[1] = (int32_T)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)c_y, i15, (int32_T)sizeof(real32_T));
      i = (int32_T)unnamed_idx_1;
      for (i15 = 0; i15 < i; i15++) {
        c_y->data[i15] = 0.0F;
      }

      if (b_b->size[1] == 0) {
      } else {
        for (i = 0; i < b_b->size[1]; i++) {
          for (ic = i; ic + 1 <= i + 1; ic++) {
            c_y->data[ic] = 0.0F;
          }
        }

        br = 0;
        for (i = 0; i < b_b->size[1]; i++) {
          ar = 0;
          i15 = br + b_a->size[1];
          for (ib = br; ib + 1 <= i15; ib++) {
            if (b_b->data[ib] != 0.0F) {
              ia = ar;
              for (ic = i; ic + 1 <= i + 1; ic++) {
                ia++;
                c_y->data[ic] += b_b->data[ib] * b_a->data[ia - 1];
              }
            }

            ar++;
          }

          br += b_a->size[1];
        }
      }
    }

    i = c_y->size[1];
    for (i15 = 0; i15 < i; i15++) {
      K[r39->data[i15]] = c_y->data[i15] / S;
    }

    emxFree_real32_T(&c_y);

    /*                */
    /* K=(S\([H(1,3),H(1,16)]*[P(3,:);P(16,:)]))'; % gain */
    dPdP0 = y_p_stat - x->QFF * dPdP0;

    /* disp(e); */
    if (dPdP0 / S * dPdP0 > 25.0F) {
      /* disp(['updatePressures chi2-test fail e=' num2str(e)]) */
    } else {
      b_emxInit_real32_T(&c_a, 1);
      p_stat_valid = TRUE;

      /*  update: */
      eml_li_find(selector, r39);
      i15 = c_a->size[0];
      c_a->size[0] = r39->size[0];
      emxEnsureCapacity((emxArray__common *)c_a, i15, (int32_T)sizeof(real32_T));
      i = r39->size[0];
      for (i15 = 0; i15 < i; i15++) {
        c_a->data[i15] = K[r39->data[i15] - 1];
      }

      eml_li_find(selector, r39);
      i15 = b_a->size[0] * b_a->size[1];
      b_a->size[0] = 1;
      b_a->size[1] = r39->size[0];
      emxEnsureCapacity((emxArray__common *)b_a, i15, (int32_T)sizeof(real32_T));
      i = r39->size[0];
      for (i15 = 0; i15 < i; i15++) {
        b_a->data[b_a->size[0] * i15] = H[r39->data[i15] - 1];
      }

      emxInit_real32_T(&KH, 2);
      i15 = KH->size[0] * KH->size[1];
      KH->size[0] = c_a->size[0];
      KH->size[1] = b_a->size[1];
      emxEnsureCapacity((emxArray__common *)KH, i15, (int32_T)sizeof(real32_T));
      i = c_a->size[0];
      for (i15 = 0; i15 < i; i15++) {
        br = b_a->size[1];
        for (i16 = 0; i16 < br; i16++) {
          KH->data[i15 + KH->size[0] * i16] = c_a->data[i15] * b_a->data
            [b_a->size[0] * i16];
        }
      }

      emxInit_real32_T(&r47, 2);
      eml_li_find(selector, r39);
      eml_li_find(selector, r40);
      i15 = r47->size[0] * r47->size[1];
      r47->size[0] = r40->size[0];
      r47->size[1] = r39->size[0];
      emxEnsureCapacity((emxArray__common *)r47, i15, (int32_T)sizeof(real32_T));
      i = r39->size[0];
      for (i15 = 0; i15 < i; i15++) {
        br = r40->size[0];
        for (i16 = 0; i16 < br; i16++) {
          r47->data[i16 + r47->size[0] * i15] = P[(r40->data[i16] + 20 *
            (r39->data[i15] - 1)) - 1];
        }
      }

      eml_li_find(selector, r39);
      eml_li_find(selector, r40);
      i15 = b_b->size[0] * b_b->size[1];
      b_b->size[0] = r40->size[0];
      b_b->size[1] = r39->size[0];
      emxEnsureCapacity((emxArray__common *)b_b, i15, (int32_T)sizeof(real32_T));
      i = r39->size[0];
      for (i15 = 0; i15 < i; i15++) {
        br = r40->size[0];
        for (i16 = 0; i16 < br; i16++) {
          b_b->data[i16 + b_b->size[0] * i15] = P[(r40->data[i16] + 20 *
            (r39->data[i15] - 1)) - 1];
        }
      }

      emxInit_real32_T(&C, 2);
      if ((KH->size[1] == 1) || (b_b->size[0] == 1)) {
        i15 = C->size[0] * C->size[1];
        C->size[0] = KH->size[0];
        C->size[1] = b_b->size[1];
        emxEnsureCapacity((emxArray__common *)C, i15, (int32_T)sizeof(real32_T));
        i = KH->size[0];
        for (i15 = 0; i15 < i; i15++) {
          br = b_b->size[1];
          for (i16 = 0; i16 < br; i16++) {
            C->data[i15 + C->size[0] * i16] = 0.0F;
            ar = KH->size[1];
            for (ib = 0; ib < ar; ib++) {
              C->data[i15 + C->size[0] * i16] += KH->data[i15 + KH->size[0] * ib]
                * b_b->data[ib + b_b->size[0] * i16];
            }
          }
        }
      } else {
        unnamed_idx_0 = (uint32_T)KH->size[0];
        unnamed_idx_1 = (uint32_T)b_b->size[1];
        i15 = C->size[0] * C->size[1];
        C->size[0] = (int32_T)unnamed_idx_0;
        emxEnsureCapacity((emxArray__common *)C, i15, (int32_T)sizeof(real32_T));
        i15 = C->size[0] * C->size[1];
        C->size[1] = (int32_T)unnamed_idx_1;
        emxEnsureCapacity((emxArray__common *)C, i15, (int32_T)sizeof(real32_T));
        i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
        for (i15 = 0; i15 < i; i15++) {
          C->data[i15] = 0.0F;
        }

        if ((KH->size[0] == 0) || (b_b->size[1] == 0)) {
        } else {
          c = KH->size[0] * (b_b->size[1] - 1);
          for (i = 0; i <= c; i += KH->size[0]) {
            i15 = i + KH->size[0];
            for (ic = i; ic + 1 <= i15; ic++) {
              C->data[ic] = 0.0F;
            }
          }

          br = 0;
          for (i = 0; i <= c; i += KH->size[0]) {
            ar = 0;
            i15 = br + KH->size[1];
            for (ib = br; ib + 1 <= i15; ib++) {
              if (b_b->data[ib] != 0.0F) {
                ia = ar;
                i16 = i + KH->size[0];
                for (ic = i; ic + 1 <= i16; ic++) {
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

      eml_li_find(selector, r39);
      eml_li_find(selector, r40);
      i15 = a->size[0] * a->size[1];
      a->size[0] = r40->size[0];
      a->size[1] = r39->size[0];
      emxEnsureCapacity((emxArray__common *)a, i15, (int32_T)sizeof(real32_T));
      i = r39->size[0];
      for (i15 = 0; i15 < i; i15++) {
        br = r40->size[0];
        for (i16 = 0; i16 < br; i16++) {
          a->data[i16 + a->size[0] * i15] = P[(r40->data[i16] + 20 * (r39->
            data[i15] - 1)) - 1];
        }
      }

      i15 = b_b->size[0] * b_b->size[1];
      b_b->size[0] = KH->size[1];
      b_b->size[1] = KH->size[0];
      emxEnsureCapacity((emxArray__common *)b_b, i15, (int32_T)sizeof(real32_T));
      i = KH->size[0];
      for (i15 = 0; i15 < i; i15++) {
        br = KH->size[1];
        for (i16 = 0; i16 < br; i16++) {
          b_b->data[i16 + b_b->size[0] * i15] = KH->data[i15 + KH->size[0] * i16];
        }
      }

      if ((a->size[1] == 1) || (b_b->size[0] == 1)) {
        i15 = KH->size[0] * KH->size[1];
        KH->size[0] = a->size[0];
        KH->size[1] = b_b->size[1];
        emxEnsureCapacity((emxArray__common *)KH, i15, (int32_T)sizeof(real32_T));
        i = a->size[0];
        for (i15 = 0; i15 < i; i15++) {
          br = b_b->size[1];
          for (i16 = 0; i16 < br; i16++) {
            KH->data[i15 + KH->size[0] * i16] = 0.0F;
            ar = a->size[1];
            for (ib = 0; ib < ar; ib++) {
              KH->data[i15 + KH->size[0] * i16] += a->data[i15 + a->size[0] * ib]
                * b_b->data[ib + b_b->size[0] * i16];
            }
          }
        }
      } else {
        unnamed_idx_0 = (uint32_T)a->size[0];
        unnamed_idx_1 = (uint32_T)b_b->size[1];
        i15 = KH->size[0] * KH->size[1];
        KH->size[0] = (int32_T)unnamed_idx_0;
        emxEnsureCapacity((emxArray__common *)KH, i15, (int32_T)sizeof(real32_T));
        i15 = KH->size[0] * KH->size[1];
        KH->size[1] = (int32_T)unnamed_idx_1;
        emxEnsureCapacity((emxArray__common *)KH, i15, (int32_T)sizeof(real32_T));
        i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
        for (i15 = 0; i15 < i; i15++) {
          KH->data[i15] = 0.0F;
        }

        if ((a->size[0] == 0) || (b_b->size[1] == 0)) {
        } else {
          c = a->size[0] * (b_b->size[1] - 1);
          for (i = 0; i <= c; i += a->size[0]) {
            i15 = i + a->size[0];
            for (ic = i; ic + 1 <= i15; ic++) {
              KH->data[ic] = 0.0F;
            }
          }

          br = 0;
          for (i = 0; i <= c; i += a->size[0]) {
            ar = 0;
            i15 = br + a->size[1];
            for (ib = br; ib + 1 <= i15; ib++) {
              if (b_b->data[ib] != 0.0F) {
                ia = ar;
                i16 = i + a->size[0];
                for (ic = i; ic + 1 <= i16; ic++) {
                  ia++;
                  KH->data[ic] += b_b->data[ib] * a->data[ia - 1];
                }
              }

              ar += a->size[0];
            }

            br += a->size[1];
          }
        }
      }

      eml_li_find(selector, r39);
      i15 = c_a->size[0];
      c_a->size[0] = r39->size[0];
      emxEnsureCapacity((emxArray__common *)c_a, i15, (int32_T)sizeof(real32_T));
      i = r39->size[0];
      for (i15 = 0; i15 < i; i15++) {
        c_a->data[i15] = K[r39->data[i15] - 1];
      }

      eml_li_find(selector, r39);
      i15 = b_a->size[0] * b_a->size[1];
      b_a->size[0] = 1;
      b_a->size[1] = r39->size[0];
      emxEnsureCapacity((emxArray__common *)b_a, i15, (int32_T)sizeof(real32_T));
      i = r39->size[0];
      for (i15 = 0; i15 < i; i15++) {
        b_a->data[b_a->size[0] * i15] = K[r39->data[i15] - 1];
      }

      eml_li_find(selector, r39);
      i15 = r39->size[0];
      r39->size[0] = r39->size[0];
      emxEnsureCapacity((emxArray__common *)r39, i15, (int32_T)sizeof(int32_T));
      i = r39->size[0];
      for (i15 = 0; i15 < i; i15++) {
        r39->data[i15]--;
      }

      eml_li_find(selector, r40);
      i15 = r40->size[0];
      r40->size[0] = r40->size[0];
      emxEnsureCapacity((emxArray__common *)r40, i15, (int32_T)sizeof(int32_T));
      i = r40->size[0];
      for (i15 = 0; i15 < i; i15++) {
        r40->data[i15]--;
      }

      emxInit_int32_T(&r48, 1);
      i15 = r48->size[0];
      r48->size[0] = r40->size[0];
      emxEnsureCapacity((emxArray__common *)r48, i15, (int32_T)sizeof(int32_T));
      i = r40->size[0];
      for (i15 = 0; i15 < i; i15++) {
        r48->data[i15] = r40->data[i15];
      }

      emxInit_int32_T(&r49, 1);
      i15 = r49->size[0];
      r49->size[0] = r39->size[0];
      emxEnsureCapacity((emxArray__common *)r49, i15, (int32_T)sizeof(int32_T));
      i = r39->size[0];
      for (i15 = 0; i15 < i; i15++) {
        r49->data[i15] = r39->data[i15];
      }

      i = c_a->size[0];
      for (i15 = 0; i15 < i; i15++) {
        br = b_a->size[1];
        for (i16 = 0; i16 < br; i16++) {
          y = c_a->data[i15] * S * b_a->data[b_a->size[0] * i16];
          P[r49->data[i15] + 20 * r48->data[i16]] = ((r47->data[i15 + r47->size
            [0] * i16] - C->data[i15 + C->size[0] * i16]) - KH->data[i15 +
            KH->size[0] * i16]) + y;
        }
      }

      emxFree_int32_T(&r49);
      emxFree_int32_T(&r48);
      emxFree_real32_T(&c_a);
      emxFree_real32_T(&C);
      emxFree_real32_T(&r47);
      emxFree_real32_T(&KH);
      for (i15 = 0; i15 < 20; i15++) {
        K[i15] *= dPdP0;
      }

      /*  effective state update: */
      for (i15 = 0; i15 < 3; i15++) {
        x->p[i15] += (real_T)K[i15];
      }

      for (i = 0; i < 3; i++) {
        dtheta_half[i] = 0.5F * K[i + 3];
      }

      S = (real32_T)sqrt(dot(dtheta_half, dtheta_half));
      if ((real32_T)fabs(S) < 0.01F) {
        dPdP0 = S * S;
        y = ((1.0F - 0.166666672F * dPdP0) + 0.00833333377F * (dPdP0 * dPdP0)) -
          0.000198412701F * (dPdP0 * dPdP0 * dPdP0);
      } else {
        y = (real32_T)sin(S) / S;
      }

      for (i = 0; i < 3; i++) {
        dq[i] = y * dtheta_half[i];
      }

      dq[3] = (real32_T)cos(S);

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
      for (i15 = 0; i15 < 4; i15++) {
        dq[i15] = 0.0F;
        for (i16 = 0; i16 < 4; i16++) {
          dPdP0 = dq[i15] + Q[i15 + (i16 << 2)] * x->q_NS[i16];
          dq[i15] = dPdP0;
        }
      }

      dPdP0 = 0.0F;
      i = 0;
      br = 0;
      for (ar = 0; ar < 4; ar++) {
        dPdP0 += dq[i] * dq[br];
        i++;
        br++;
      }

      dPdP0 = (real32_T)sqrt(dPdP0);
      for (i15 = 0; i15 < 4; i15++) {
        dq[i15] /= dPdP0;
      }

      for (i = 0; i < 4; i++) {
        x->q_NS[i] = dq[i];
      }

      for (i15 = 0; i15 < 3; i15++) {
        x->v_N[i15] += K[6 + i15];
      }

      for (i15 = 0; i15 < 3; i15++) {
        x->b_g[i15] += K[9 + i15];
      }

      for (i15 = 0; i15 < 3; i15++) {
        x->b_a[i15] += K[12 + i15];
      }

      x->QFF += K[15];
      for (i15 = 0; i15 < 3; i15++) {
        x->w[i15] += K[16 + i15];
      }

      x->K += K[19];
    }

    emxFree_int32_T(&r39);
    emxFree_int32_T(&r40);
    emxFree_real32_T(&b_b);
    emxFree_real32_T(&b_a);
    emxFree_real32_T(&a);
  }

  return p_stat_valid;
}

/* End of code generation (updatePressures.c) */
