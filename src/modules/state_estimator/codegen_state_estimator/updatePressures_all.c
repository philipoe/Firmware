/*
 * updatePressures_all.c
 *
 * Code generation for function 'updatePressures_all'
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
#include "dot.h"
#include "HyEst_emxutil.h"
#include "mldivide.h"
#include "pinv.h"
#include "diag.h"
#include "autogen_TasUpdate_withAerodynamics_alpha_set.h"
#include "autogen_TasUpdate_withAerodynamics.h"
#include "HyEst_rtwutil.h"
#include "HyEst_data.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void updatePressures_all(states_T *x, real32_T P[400], real32_T y_p_stat,
  real32_T y_p_dyn, real32_T T_h, real32_T ell_offset, const real32_T acc[3],
  const airplane_T airplane, boolean_T constWind, boolean_T constQFF, boolean_T
  constK, boolean_T *p_stat_valid, boolean_T *on_ground, boolean_T *aero_valid)
{
  real32_T rho;
  real32_T C_AB[9];
  real32_T C_SN[9];
  int32_T i17;
  int32_T i18;
  real32_T v_N_rel[3];
  int32_T i;
  real32_T v_S_rel[3];
  real32_T alpha;
  real32_T y;
  real32_T z_orig[7];
  boolean_T emergency;
  real32_T E_x_dash[36];
  real32_T E_z[28];
  real32_T e[4];
  real32_T X[9];
  real32_T b_C_SN[9];
  int32_T ar;
  real32_T fv2[180];
  static const int8_T iv5[20] = { 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0 };

  static const int8_T iv6[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const int8_T iv7[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0 };

  static const int8_T iv8[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1 };

  real32_T E_x[80];
  real32_T H[80];
  real32_T r_a[7];
  real32_T R[49];
  real32_T b_E_x[80];
  real32_T b_E_z[28];
  real32_T c_E_x[16];
  real32_T c_E_z[16];
  real32_T S[16];
  real32_T b_y[4];
  real32_T K[80];
  boolean_T selector[20];
  emxArray_int32_T *r50;
  boolean_T b_selector[20];
  emxArray_int32_T *r51;
  emxArray_real32_T *a;
  emxArray_int32_T *r52;
  emxArray_int32_T *r53;
  int32_T iy;
  emxArray_int32_T *r54;
  emxArray_int32_T *r55;
  emxArray_int32_T *r56;
  emxArray_int32_T *r57;
  emxArray_real32_T *b_a;
  emxArray_real32_T *b;
  emxArray_real32_T *c_y;
  uint32_T unnamed_idx_1;
  int32_T ic;
  int32_T br;
  int32_T ib;
  int32_T ia;
  emxArray_real32_T *c_a;
  emxArray_real32_T *KH;
  uint32_T unnamed_idx_0;
  emxArray_real32_T *r58;
  emxArray_real32_T *C;
  emxArray_real32_T *b_C;
  emxArray_real32_T *d_y;
  emxArray_int32_T *r59;
  emxArray_int32_T *r60;
  real32_T delta_x[20];
  real32_T Q[16];

  /*  */
  /*  [x,P,p_stat_valid,on_ground,aero_valid] =... */
  /*     UPDATEPRESSURES3(x,P,y_p_stat,y_p_dyn,T_h,ell_offset,... */
  /*     acc,airplane,constWind,constQFF,constK) */
  /*  */
  /*  Kalman filter update with static and dynamic pressure: version with lift */
  /*  polar. */
  /*  */
  /*  Author: lestefan */
  /*  date:  02/2013 */
  /*  */
  /*  Inputs: */
  /*    x:            states [p(lat*1e7,lon*1e7,h),q_NS,v_N,b_g,b_a,QFF,w,K] */
  /*    P:            covariance */
  /*    y_p_stat:     static pressure [kPa] */
  /*    y_p_dyn:      dynamic pressure [kPa] */
  /*    T_h:          ambient air temperature [°K] */
  /*    ell_offset:   offset between WGS-84 and MSL [m] */
  /*    acc:          acceleration (in sensor frame S) 3x1 acc_S [m/s^2] */
  /*    constWind:    keep wind constant [true/false] (use true on GPS out) */
  /*    constQFF:     keep QFF constant [true/false] (use true on GPS out)                     */
  /*    constK:       K (sideslip parameter) const. [true/false] (use true) */
  /*  */
  /*  Outputs: */
  /*    x:            updated states (see inputs) */
  /*    P:            updated covariance (see inputs) */
  /*    p_stat_valid: valid pressure [true/false] */
  /*    on_ground:    on ground [true/false] */
  /*    aero_valid:   aerodynamics valid [true/false] */
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
  /*  UPDATEPRESSURES, UPDATEPRESSURES2, PROPATATE, GETINITIALQ, MAGFIELD */
  /*  */
  /*  TODO: update in the past, re-propagate. This might not be fast enough... */
  /* eml.inline('always'); */
  /* p_stat_valid = false; */
  *on_ground = FALSE;
  *aero_valid = FALSE;
  *p_stat_valid = TRUE;
  if (y_p_stat < 0.0F) {
    *p_stat_valid = FALSE;
  } else {
    /*  get some global parameters */
    /*  air density */
    rho = y_p_stat * 1000.0F / (287.058F * T_h);

    /*  aerodynamics scale */
    /*  measuerements */
    /*  calculate the velocity (x body, scalar) from the differential pressure */
    /*  coordinate trafo */
    /*  set float type */
    C_AB[0] = ((x->q_NS[0] * x->q_NS[0] - x->q_NS[1] * x->q_NS[1]) - x->q_NS[2] *
               x->q_NS[2]) + x->q_NS[3] * x->q_NS[3];
    C_AB[3] = x->q_NS[0] * x->q_NS[1] * 2.0F + x->q_NS[2] * x->q_NS[3] * 2.0F;
    C_AB[6] = x->q_NS[0] * x->q_NS[2] * 2.0F - x->q_NS[1] * x->q_NS[3] * 2.0F;
    C_AB[1] = x->q_NS[0] * x->q_NS[1] * 2.0F - x->q_NS[2] * x->q_NS[3] * 2.0F;
    C_AB[4] = ((-x->q_NS[0] * x->q_NS[0] + x->q_NS[1] * x->q_NS[1]) - x->q_NS[2]
               * x->q_NS[2]) + x->q_NS[3] * x->q_NS[3];
    C_AB[7] = x->q_NS[0] * x->q_NS[3] * 2.0F + x->q_NS[1] * x->q_NS[2] * 2.0F;
    C_AB[2] = x->q_NS[0] * x->q_NS[2] * 2.0F + x->q_NS[1] * x->q_NS[3] * 2.0F;
    C_AB[5] = x->q_NS[0] * x->q_NS[3] * -2.0F + x->q_NS[1] * x->q_NS[2] * 2.0F;
    C_AB[8] = ((-x->q_NS[0] * x->q_NS[0] - x->q_NS[1] * x->q_NS[1]) + x->q_NS[2]
               * x->q_NS[2]) + x->q_NS[3] * x->q_NS[3];
    for (i17 = 0; i17 < 3; i17++) {
      for (i18 = 0; i18 < 3; i18++) {
        C_SN[i18 + 3 * i17] = C_AB[i17 + 3 * i18];
      }
    }

    /*  Quaternion to DCM */
    /*  helper states */
    for (i = 0; i < 3; i++) {
      v_N_rel[i] = x->v_N[i] - x->w[i];
    }

    /*   */
    for (i17 = 0; i17 < 3; i17++) {
      v_S_rel[i17] = 0.0F;
      for (i18 = 0; i18 < 3; i18++) {
        v_S_rel[i17] += C_SN[i17 + 3 * i18] * v_N_rel[i18];
      }
    }

    alpha = rt_atan2f_snf(v_S_rel[2], v_S_rel[0]);

    /* disp(alpha/pi*180) */
    if (y_p_dyn >= 0.0F) {
      y = y_p_dyn;
    } else {
      y = 0.0F;
    }

    if ((real32_T)sqrt(2.0F * y * 1000.0F / rho) < airplane.vmin * (real32_T)
        sqrt(airplane.ref_mass[0] * (real32_T)sqrt(rt_powf_snf(acc[2], 2.0F)) *
             airplane.rho0 / (airplane.ref_mass[1] * 9.81F * rho)) * 0.5F) {
      /* disp('on ground, stand still') */
      *p_stat_valid = b_updatePressures(x, P, y_p_stat, T_h, ell_offset,
        constQFF, constK);
      *on_ground = TRUE;
    } else {
      /*  get error and Jacobians */
      for (i = 0; i < 3; i++) {
        z_orig[i] = acc[i];
      }

      z_orig[3] = y_p_stat;
      z_orig[4] = y_p_dyn;
      z_orig[5] = T_h;
      z_orig[6] = 0.0F;
      emergency = FALSE;
      if (alpha * 180.0F / 3.14159274F < airplane.alpha_min) {
        z_orig[6] = airplane.alpha_min;
        C_AB[0] = (real32_T)x->p[2] - ell_offset;
        for (i = 0; i < 3; i++) {
          C_AB[i + 1] = v_S_rel[i];
        }

        for (i = 0; i < 3; i++) {
          C_AB[i + 4] = x->b_a[i];
        }

        C_AB[7] = x->QFF;
        C_AB[8] = x->K;
        c_autogen_TasUpdate_withAerodyn(C_AB, z_orig, airplane.ref_mass[0],
          airplane.A, e, E_z, E_x_dash);
        emergency = TRUE;

        /* fprintf('c_L_min '); */
      } else if (alpha * 180.0F / 3.14159274F > airplane.alpha_max) {
        z_orig[6] = airplane.alpha_max;
        C_AB[0] = (real32_T)x->p[2] - ell_offset;
        for (i = 0; i < 3; i++) {
          C_AB[i + 1] = v_S_rel[i];
        }

        for (i = 0; i < 3; i++) {
          C_AB[i + 4] = x->b_a[i];
        }

        C_AB[7] = x->QFF;
        C_AB[8] = x->K;
        c_autogen_TasUpdate_withAerodyn(C_AB, z_orig, airplane.ref_mass[0],
          airplane.A, e, E_z, E_x_dash);
        emergency = TRUE;

        /* fprintf('c_L_max '); */
      } else {
        C_AB[0] = (real32_T)x->p[2] - ell_offset;
        for (i = 0; i < 3; i++) {
          C_AB[i + 1] = v_S_rel[i];
        }

        for (i = 0; i < 3; i++) {
          C_AB[i + 4] = x->b_a[i];
        }

        C_AB[7] = x->QFF;
        C_AB[8] = x->K;
        d_autogen_TasUpdate_withAerodyn(C_AB, z_orig, airplane.c_L_alpha,
          airplane.ref_mass[0], airplane.A, e, E_z, E_x_dash);
        for (i17 = 0; i17 < 3; i17++) {
          E_z[12 + i17] *= 0.0F;
          E_z[20 + i17] *= 0.0F;
        }

        /* fprintf('regular '); */
      }

      /*  apply chain rule: de/dx=de/dv_S_rel*dv_S_rel/dx */
      /*  set float type */
      X[0] = 0.0F;
      X[3] = -v_N_rel[2];
      X[6] = v_N_rel[1];
      X[1] = v_N_rel[2];
      X[4] = 0.0F;
      X[7] = -v_N_rel[0];
      X[2] = -v_N_rel[1];
      X[5] = v_N_rel[0];
      X[8] = 0.0F;
      for (i17 = 0; i17 < 3; i17++) {
        for (i18 = 0; i18 < 3; i18++) {
          C_AB[i18 + 3 * i17] = -C_SN[i18 + 3 * i17];
        }
      }

      for (i17 = 0; i17 < 3; i17++) {
        for (i18 = 0; i18 < 3; i18++) {
          b_C_SN[i17 + 3 * i18] = 0.0F;
          for (ar = 0; ar < 3; ar++) {
            b_C_SN[i17 + 3 * i18] += C_AB[i17 + 3 * ar] * X[ar + 3 * i18];
          }
        }
      }

      for (i17 = 0; i17 < 20; i17++) {
        fv2[9 * i17] = (real32_T)iv5[i17];
      }

      for (i17 = 0; i17 < 3; i17++) {
        for (i18 = 0; i18 < 3; i18++) {
          fv2[(i18 + 9 * i17) + 1] = 0.0F;
        }
      }

      for (i17 = 0; i17 < 3; i17++) {
        for (i18 = 0; i18 < 3; i18++) {
          fv2[(i18 + 9 * (i17 + 3)) + 1] = b_C_SN[i18 + 3 * i17];
        }
      }

      for (i17 = 0; i17 < 3; i17++) {
        for (i18 = 0; i18 < 3; i18++) {
          fv2[(i18 + 9 * (i17 + 6)) + 1] = C_SN[i18 + 3 * i17];
        }
      }

      for (i17 = 0; i17 < 7; i17++) {
        for (i18 = 0; i18 < 3; i18++) {
          fv2[(i18 + 9 * (i17 + 9)) + 1] = 0.0F;
        }
      }

      for (i17 = 0; i17 < 3; i17++) {
        for (i18 = 0; i18 < 3; i18++) {
          fv2[(i18 + 9 * (i17 + 16)) + 1] = -C_SN[i18 + 3 * i17];
        }
      }

      for (i17 = 0; i17 < 3; i17++) {
        fv2[i17 + 172] = 0.0F;
      }

      for (i17 = 0; i17 < 20; i17++) {
        for (i18 = 0; i18 < 3; i18++) {
          fv2[(i18 + 9 * i17) + 4] = (real32_T)iv6[i18 + 3 * i17];
        }

        fv2[7 + 9 * i17] = (real32_T)iv7[i17];
        fv2[8 + 9 * i17] = (real32_T)iv8[i17];
      }

      for (i17 = 0; i17 < 4; i17++) {
        for (i18 = 0; i18 < 20; i18++) {
          E_x[i17 + (i18 << 2)] = 0.0F;
          for (ar = 0; ar < 9; ar++) {
            E_x[i17 + (i18 << 2)] += E_x_dash[i17 + (ar << 2)] * fv2[ar + 9 *
              i18];
          }
        }
      }

      for (i17 = 0; i17 < 80; i17++) {
        H[i17] = -E_x[i17];
      }

      /*  measurement covariance */
      rho = rt_powf_snf(configuration.sigma_a_d, 2.0F);

      /*  conservative... */
      r_a[0] = rho;
      r_a[1] = 100.0F * rho;
      r_a[2] = rho;
      r_a[3] = rt_powf_snf(configuration.sigma_psmd, 2.0F);
      r_a[4] = rt_powf_snf(configuration.sigma_pdmd, 2.0F);
      r_a[5] = rt_powf_snf(configuration.sigma_Td, 2.0F);
      r_a[6] = 0.0625F;
      c_diag(r_a, R);

      /*  residual covariance: */
      for (i17 = 0; i17 < 4; i17++) {
        for (i18 = 0; i18 < 20; i18++) {
          b_E_x[i17 + (i18 << 2)] = 0.0F;
          for (ar = 0; ar < 20; ar++) {
            b_E_x[i17 + (i18 << 2)] += E_x[i17 + (ar << 2)] * P[ar + 20 * i18];
          }
        }
      }

      for (i17 = 0; i17 < 4; i17++) {
        for (i18 = 0; i18 < 7; i18++) {
          b_E_z[i17 + (i18 << 2)] = 0.0F;
          for (ar = 0; ar < 7; ar++) {
            b_E_z[i17 + (i18 << 2)] += E_z[i17 + (ar << 2)] * R[ar + 7 * i18];
          }
        }

        for (i18 = 0; i18 < 4; i18++) {
          c_E_x[i17 + (i18 << 2)] = 0.0F;
          for (ar = 0; ar < 20; ar++) {
            c_E_x[i17 + (i18 << 2)] += b_E_x[i17 + (ar << 2)] * E_x[i18 + (ar <<
              2)];
          }

          c_E_z[i17 + (i18 << 2)] = 0.0F;
          for (ar = 0; ar < 7; ar++) {
            c_E_z[i17 + (i18 << 2)] += b_E_z[i17 + (ar << 2)] * E_z[i18 + (ar <<
              2)];
          }
        }
      }

      for (i17 = 0; i17 < 4; i17++) {
        for (i18 = 0; i18 < 4; i18++) {
          S[i18 + (i17 << 2)] = c_E_x[i18 + (i17 << 2)] + c_E_z[i18 + (i17 << 2)];
        }
      }

      /* if(rcond(S)<1e-6); */
      /*     disp(S) */
      /* end */
      /*  z-chi2: */
      b_pinv(S, c_E_x);
      rho = 0.0F;
      for (i17 = 0; i17 < 4; i17++) {
        b_y[i17] = 0.0F;
        for (i18 = 0; i18 < 4; i18++) {
          b_y[i17] += e[i18] * c_E_x[i18 + (i17 << 2)];
        }

        rho += b_y[i17] * e[i17];
      }

      if (emergency) {
        rho = 0.0F;

        /*  force!! */
      }

      if (rho > 50.0F) {
        /* disp(['pressures_all chi2=' num2str(chi2)]); */
        *p_stat_valid = b_updatePressures(x, P, y_p_stat, T_h, ell_offset,
          constQFF, constK);
      } else {
        memset(&K[0], 0, 80U * sizeof(real32_T));
        for (i = 0; i < 20; i++) {
          selector[i] = TRUE;
        }

        if (constK) {
          selector[19] = FALSE;
        }

        if (constQFF) {
          selector[15] = FALSE;
        }

        emxInit_int32_T(&r50, 1);

        /*  remove correlations of fixed states with active states */
        eml_li_find(selector, r50);
        i17 = r50->size[0];
        r50->size[0] = r50->size[0];
        emxEnsureCapacity((emxArray__common *)r50, i17, (int32_T)sizeof(int32_T));
        i = r50->size[0];
        for (i17 = 0; i17 < i; i17++) {
          r50->data[i17]--;
        }

        for (i = 0; i < 20; i++) {
          b_selector[i] = !selector[i];
        }

        emxInit_int32_T(&r51, 1);
        eml_li_find(b_selector, r51);
        i17 = r51->size[0];
        r51->size[0] = r51->size[0];
        emxEnsureCapacity((emxArray__common *)r51, i17, (int32_T)sizeof(int32_T));
        i = r51->size[0];
        for (i17 = 0; i17 < i; i17++) {
          r51->data[i17]--;
        }

        for (i = 0; i < 20; i++) {
          b_selector[i] = !selector[i];
        }

        emxInit_real32_T(&a, 2);
        emxInit_int32_T(&r52, 1);
        emxInit_int32_T(&r53, 1);
        eml_li_find(b_selector, r52);
        eml_li_find(selector, r53);
        i17 = a->size[0] * a->size[1];
        a->size[0] = r53->size[0];
        a->size[1] = r52->size[0];
        emxEnsureCapacity((emxArray__common *)a, i17, (int32_T)sizeof(real32_T));
        i = r52->size[0];
        for (i17 = 0; i17 < i; i17++) {
          iy = r53->size[0];
          for (i18 = 0; i18 < iy; i18++) {
            a->data[i18 + a->size[0] * i17] = P[(r53->data[i18] + 20 *
              (r52->data[i17] - 1)) - 1];
          }
        }

        emxInit_int32_T(&r54, 1);
        i17 = r54->size[0];
        r54->size[0] = r51->size[0];
        emxEnsureCapacity((emxArray__common *)r54, i17, (int32_T)sizeof(int32_T));
        i = r51->size[0];
        for (i17 = 0; i17 < i; i17++) {
          r54->data[i17] = r51->data[i17];
        }

        emxInit_int32_T(&r55, 1);
        i17 = r55->size[0];
        r55->size[0] = r50->size[0];
        emxEnsureCapacity((emxArray__common *)r55, i17, (int32_T)sizeof(int32_T));
        i = r50->size[0];
        for (i17 = 0; i17 < i; i17++) {
          r55->data[i17] = r50->data[i17];
        }

        i = a->size[1];
        for (i17 = 0; i17 < i; i17++) {
          iy = a->size[0];
          for (i18 = 0; i18 < iy; i18++) {
            P[r55->data[i18] + 20 * r54->data[i17]] = a->data[i18 + a->size[0] *
              i17] * 0.0F;
          }
        }

        emxFree_int32_T(&r55);
        emxFree_int32_T(&r54);
        for (i = 0; i < 20; i++) {
          b_selector[i] = !selector[i];
        }

        eml_li_find(b_selector, r50);
        i17 = r50->size[0];
        r50->size[0] = r50->size[0];
        emxEnsureCapacity((emxArray__common *)r50, i17, (int32_T)sizeof(int32_T));
        i = r50->size[0];
        for (i17 = 0; i17 < i; i17++) {
          r50->data[i17]--;
        }

        eml_li_find(selector, r51);
        i17 = r51->size[0];
        r51->size[0] = r51->size[0];
        emxEnsureCapacity((emxArray__common *)r51, i17, (int32_T)sizeof(int32_T));
        i = r51->size[0];
        for (i17 = 0; i17 < i; i17++) {
          r51->data[i17]--;
        }

        eml_li_find(selector, r52);
        for (i = 0; i < 20; i++) {
          b_selector[i] = !selector[i];
        }

        eml_li_find(b_selector, r53);
        i17 = a->size[0] * a->size[1];
        a->size[0] = r53->size[0];
        a->size[1] = r52->size[0];
        emxEnsureCapacity((emxArray__common *)a, i17, (int32_T)sizeof(real32_T));
        i = r52->size[0];
        for (i17 = 0; i17 < i; i17++) {
          iy = r53->size[0];
          for (i18 = 0; i18 < iy; i18++) {
            a->data[i18 + a->size[0] * i17] = P[(r53->data[i18] + 20 *
              (r52->data[i17] - 1)) - 1];
          }
        }

        emxFree_int32_T(&r53);
        emxInit_int32_T(&r56, 1);
        i17 = r56->size[0];
        r56->size[0] = r51->size[0];
        emxEnsureCapacity((emxArray__common *)r56, i17, (int32_T)sizeof(int32_T));
        i = r51->size[0];
        for (i17 = 0; i17 < i; i17++) {
          r56->data[i17] = r51->data[i17];
        }

        emxInit_int32_T(&r57, 1);
        i17 = r57->size[0];
        r57->size[0] = r50->size[0];
        emxEnsureCapacity((emxArray__common *)r57, i17, (int32_T)sizeof(int32_T));
        i = r50->size[0];
        for (i17 = 0; i17 < i; i17++) {
          r57->data[i17] = r50->data[i17];
        }

        i = a->size[1];
        for (i17 = 0; i17 < i; i17++) {
          iy = a->size[0];
          for (i18 = 0; i18 < iy; i18++) {
            P[r57->data[i18] + 20 * r56->data[i17]] = a->data[i18 + a->size[0] *
              i17] * 0.0F;
          }
        }

        emxFree_int32_T(&r57);
        emxFree_int32_T(&r56);

        /* P(~selector,~selector)=P(~selector,~selector)*0; */
        eml_li_find(selector, r50);
        i17 = r50->size[0];
        r50->size[0] = r50->size[0];
        emxEnsureCapacity((emxArray__common *)r50, i17, (int32_T)sizeof(int32_T));
        i = r50->size[0];
        for (i17 = 0; i17 < i; i17++) {
          r50->data[i17]--;
        }

        emxInit_real32_T(&b_a, 2);
        eml_li_find(selector, r51);
        i17 = b_a->size[0] * b_a->size[1];
        b_a->size[0] = 4;
        b_a->size[1] = r51->size[0];
        emxEnsureCapacity((emxArray__common *)b_a, i17, (int32_T)sizeof(real32_T));
        i = r51->size[0];
        for (i17 = 0; i17 < i; i17++) {
          for (i18 = 0; i18 < 4; i18++) {
            b_a->data[i18 + b_a->size[0] * i17] = H[i18 + ((r51->data[i17] - 1) <<
              2)];
          }
        }

        emxInit_real32_T(&b, 2);
        eml_li_find(selector, r51);
        eml_li_find(selector, r52);
        i17 = b->size[0] * b->size[1];
        b->size[0] = r52->size[0];
        b->size[1] = r51->size[0];
        emxEnsureCapacity((emxArray__common *)b, i17, (int32_T)sizeof(real32_T));
        i = r51->size[0];
        for (i17 = 0; i17 < i; i17++) {
          iy = r52->size[0];
          for (i18 = 0; i18 < iy; i18++) {
            b->data[i18 + b->size[0] * i17] = P[(r52->data[i18] + 20 *
              (r51->data[i17] - 1)) - 1];
          }
        }

        emxFree_int32_T(&r52);
        emxInit_real32_T(&c_y, 2);
        if ((b_a->size[1] == 1) || (b->size[0] == 1)) {
          i17 = c_y->size[0] * c_y->size[1];
          c_y->size[0] = 4;
          c_y->size[1] = b->size[1];
          emxEnsureCapacity((emxArray__common *)c_y, i17, (int32_T)sizeof
                            (real32_T));
          for (i17 = 0; i17 < 4; i17++) {
            i = b->size[1];
            for (i18 = 0; i18 < i; i18++) {
              c_y->data[i17 + c_y->size[0] * i18] = 0.0F;
              iy = b_a->size[1];
              for (ar = 0; ar < iy; ar++) {
                c_y->data[i17 + c_y->size[0] * i18] += b_a->data[i17 + b_a->
                  size[0] * ar] * b->data[ar + b->size[0] * i18];
              }
            }
          }
        } else {
          unnamed_idx_1 = (uint32_T)b->size[1];
          i17 = c_y->size[0] * c_y->size[1];
          c_y->size[0] = 4;
          emxEnsureCapacity((emxArray__common *)c_y, i17, (int32_T)sizeof
                            (real32_T));
          i17 = c_y->size[0] * c_y->size[1];
          c_y->size[1] = (int32_T)unnamed_idx_1;
          emxEnsureCapacity((emxArray__common *)c_y, i17, (int32_T)sizeof
                            (real32_T));
          i = (int32_T)unnamed_idx_1 << 2;
          for (i17 = 0; i17 < i; i17++) {
            c_y->data[i17] = 0.0F;
          }

          if (b->size[1] == 0) {
          } else {
            i = (b->size[1] - 1) << 2;
            for (iy = 0; iy <= i; iy += 4) {
              for (ic = iy; ic + 1 <= iy + 4; ic++) {
                c_y->data[ic] = 0.0F;
              }
            }

            br = 0;
            for (iy = 0; iy <= i; iy += 4) {
              ar = 0;
              i17 = br + b_a->size[1];
              for (ib = br; ib + 1 <= i17; ib++) {
                if (b->data[ib] != 0.0F) {
                  ia = ar;
                  for (ic = iy; ic + 1 <= iy + 4; ic++) {
                    ia++;
                    c_y->data[ic] += b->data[ib] * b_a->data[ia - 1];
                  }
                }

                ar += 4;
              }

              br += b_a->size[1];
            }
          }
        }

        emxInit_real32_T(&c_a, 2);
        b_mldivide(S, c_y, b_a);
        i17 = c_a->size[0] * c_a->size[1];
        c_a->size[0] = b_a->size[1];
        c_a->size[1] = 4;
        emxEnsureCapacity((emxArray__common *)c_a, i17, (int32_T)sizeof(real32_T));
        emxFree_real32_T(&c_y);
        for (i17 = 0; i17 < 4; i17++) {
          i = b_a->size[1];
          for (i18 = 0; i18 < i; i18++) {
            c_a->data[i18 + c_a->size[0] * i17] = b_a->data[i17 + b_a->size[0] *
              i18];
          }
        }

        for (i17 = 0; i17 < 4; i17++) {
          i = c_a->size[0];
          for (i18 = 0; i18 < i; i18++) {
            K[r50->data[i18] + 20 * i17] = c_a->data[i18 + c_a->size[0] * i17];
          }
        }

        /*  update: */
        eml_li_find(selector, r50);
        i17 = c_a->size[0] * c_a->size[1];
        c_a->size[0] = r50->size[0];
        c_a->size[1] = 4;
        emxEnsureCapacity((emxArray__common *)c_a, i17, (int32_T)sizeof(real32_T));
        for (i17 = 0; i17 < 4; i17++) {
          i = r50->size[0];
          for (i18 = 0; i18 < i; i18++) {
            c_a->data[i18 + c_a->size[0] * i17] = K[(r50->data[i18] + 20 * i17)
              - 1];
          }
        }

        eml_li_find(selector, r50);
        i17 = b_a->size[0] * b_a->size[1];
        b_a->size[0] = 4;
        b_a->size[1] = r50->size[0];
        emxEnsureCapacity((emxArray__common *)b_a, i17, (int32_T)sizeof(real32_T));
        i = r50->size[0];
        for (i17 = 0; i17 < i; i17++) {
          for (i18 = 0; i18 < 4; i18++) {
            b_a->data[i18 + b_a->size[0] * i17] = H[i18 + ((r50->data[i17] - 1) <<
              2)];
          }
        }

        emxInit_real32_T(&KH, 2);
        unnamed_idx_0 = (uint32_T)c_a->size[0];
        unnamed_idx_1 = (uint32_T)b_a->size[1];
        i17 = KH->size[0] * KH->size[1];
        KH->size[0] = (int32_T)unnamed_idx_0;
        emxEnsureCapacity((emxArray__common *)KH, i17, (int32_T)sizeof(real32_T));
        i17 = KH->size[0] * KH->size[1];
        KH->size[1] = (int32_T)unnamed_idx_1;
        emxEnsureCapacity((emxArray__common *)KH, i17, (int32_T)sizeof(real32_T));
        i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
        for (i17 = 0; i17 < i; i17++) {
          KH->data[i17] = 0.0F;
        }

        if ((c_a->size[0] == 0) || (b_a->size[1] == 0)) {
        } else {
          i = c_a->size[0] * (b_a->size[1] - 1);
          for (iy = 0; iy <= i; iy += c_a->size[0]) {
            i17 = iy + c_a->size[0];
            for (ic = iy; ic + 1 <= i17; ic++) {
              KH->data[ic] = 0.0F;
            }
          }

          br = 0;
          for (iy = 0; iy <= i; iy += c_a->size[0]) {
            ar = 0;
            for (ib = br; ib + 1 <= br + 4; ib++) {
              if (b_a->data[ib] != 0.0F) {
                ia = ar;
                i17 = iy + c_a->size[0];
                for (ic = iy; ic + 1 <= i17; ic++) {
                  ia++;
                  KH->data[ic] += b_a->data[ib] * c_a->data[ia - 1];
                }
              }

              ar += c_a->size[0];
            }

            br += 4;
          }
        }

        emxInit_real32_T(&r58, 2);
        eml_li_find(selector, r50);
        eml_li_find(selector, r51);
        i17 = r58->size[0] * r58->size[1];
        r58->size[0] = r51->size[0];
        r58->size[1] = r50->size[0];
        emxEnsureCapacity((emxArray__common *)r58, i17, (int32_T)sizeof(real32_T));
        i = r50->size[0];
        for (i17 = 0; i17 < i; i17++) {
          iy = r51->size[0];
          for (i18 = 0; i18 < iy; i18++) {
            r58->data[i18 + r58->size[0] * i17] = P[(r51->data[i18] + 20 *
              (r50->data[i17] - 1)) - 1];
          }
        }

        eml_li_find(selector, r50);
        eml_li_find(selector, r51);
        i17 = b->size[0] * b->size[1];
        b->size[0] = r51->size[0];
        b->size[1] = r50->size[0];
        emxEnsureCapacity((emxArray__common *)b, i17, (int32_T)sizeof(real32_T));
        i = r50->size[0];
        for (i17 = 0; i17 < i; i17++) {
          iy = r51->size[0];
          for (i18 = 0; i18 < iy; i18++) {
            b->data[i18 + b->size[0] * i17] = P[(r51->data[i18] + 20 *
              (r50->data[i17] - 1)) - 1];
          }
        }

        emxInit_real32_T(&C, 2);
        if ((KH->size[1] == 1) || (b->size[0] == 1)) {
          i17 = C->size[0] * C->size[1];
          C->size[0] = KH->size[0];
          C->size[1] = b->size[1];
          emxEnsureCapacity((emxArray__common *)C, i17, (int32_T)sizeof(real32_T));
          i = KH->size[0];
          for (i17 = 0; i17 < i; i17++) {
            iy = b->size[1];
            for (i18 = 0; i18 < iy; i18++) {
              C->data[i17 + C->size[0] * i18] = 0.0F;
              br = KH->size[1];
              for (ar = 0; ar < br; ar++) {
                C->data[i17 + C->size[0] * i18] += KH->data[i17 + KH->size[0] *
                  ar] * b->data[ar + b->size[0] * i18];
              }
            }
          }
        } else {
          unnamed_idx_0 = (uint32_T)KH->size[0];
          unnamed_idx_1 = (uint32_T)b->size[1];
          i17 = C->size[0] * C->size[1];
          C->size[0] = (int32_T)unnamed_idx_0;
          emxEnsureCapacity((emxArray__common *)C, i17, (int32_T)sizeof(real32_T));
          i17 = C->size[0] * C->size[1];
          C->size[1] = (int32_T)unnamed_idx_1;
          emxEnsureCapacity((emxArray__common *)C, i17, (int32_T)sizeof(real32_T));
          i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
          for (i17 = 0; i17 < i; i17++) {
            C->data[i17] = 0.0F;
          }

          if ((KH->size[0] == 0) || (b->size[1] == 0)) {
          } else {
            i = KH->size[0] * (b->size[1] - 1);
            for (iy = 0; iy <= i; iy += KH->size[0]) {
              i17 = iy + KH->size[0];
              for (ic = iy; ic + 1 <= i17; ic++) {
                C->data[ic] = 0.0F;
              }
            }

            br = 0;
            for (iy = 0; iy <= i; iy += KH->size[0]) {
              ar = 0;
              i17 = br + KH->size[1];
              for (ib = br; ib + 1 <= i17; ib++) {
                if (b->data[ib] != 0.0F) {
                  ia = ar;
                  i18 = iy + KH->size[0];
                  for (ic = iy; ic + 1 <= i18; ic++) {
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

        eml_li_find(selector, r50);
        eml_li_find(selector, r51);
        i17 = a->size[0] * a->size[1];
        a->size[0] = r51->size[0];
        a->size[1] = r50->size[0];
        emxEnsureCapacity((emxArray__common *)a, i17, (int32_T)sizeof(real32_T));
        i = r50->size[0];
        for (i17 = 0; i17 < i; i17++) {
          iy = r51->size[0];
          for (i18 = 0; i18 < iy; i18++) {
            a->data[i18 + a->size[0] * i17] = P[(r51->data[i18] + 20 *
              (r50->data[i17] - 1)) - 1];
          }
        }

        i17 = b->size[0] * b->size[1];
        b->size[0] = KH->size[1];
        b->size[1] = KH->size[0];
        emxEnsureCapacity((emxArray__common *)b, i17, (int32_T)sizeof(real32_T));
        i = KH->size[0];
        for (i17 = 0; i17 < i; i17++) {
          iy = KH->size[1];
          for (i18 = 0; i18 < iy; i18++) {
            b->data[i18 + b->size[0] * i17] = KH->data[i17 + KH->size[0] * i18];
          }
        }

        emxInit_real32_T(&b_C, 2);
        if ((a->size[1] == 1) || (b->size[0] == 1)) {
          i17 = b_C->size[0] * b_C->size[1];
          b_C->size[0] = a->size[0];
          b_C->size[1] = b->size[1];
          emxEnsureCapacity((emxArray__common *)b_C, i17, (int32_T)sizeof
                            (real32_T));
          i = a->size[0];
          for (i17 = 0; i17 < i; i17++) {
            iy = b->size[1];
            for (i18 = 0; i18 < iy; i18++) {
              b_C->data[i17 + b_C->size[0] * i18] = 0.0F;
              br = a->size[1];
              for (ar = 0; ar < br; ar++) {
                b_C->data[i17 + b_C->size[0] * i18] += a->data[i17 + a->size[0] *
                  ar] * b->data[ar + b->size[0] * i18];
              }
            }
          }
        } else {
          unnamed_idx_0 = (uint32_T)a->size[0];
          unnamed_idx_1 = (uint32_T)b->size[1];
          i17 = b_C->size[0] * b_C->size[1];
          b_C->size[0] = (int32_T)unnamed_idx_0;
          emxEnsureCapacity((emxArray__common *)b_C, i17, (int32_T)sizeof
                            (real32_T));
          i17 = b_C->size[0] * b_C->size[1];
          b_C->size[1] = (int32_T)unnamed_idx_1;
          emxEnsureCapacity((emxArray__common *)b_C, i17, (int32_T)sizeof
                            (real32_T));
          i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
          for (i17 = 0; i17 < i; i17++) {
            b_C->data[i17] = 0.0F;
          }

          if ((a->size[0] == 0) || (b->size[1] == 0)) {
          } else {
            i = a->size[0] * (b->size[1] - 1);
            for (iy = 0; iy <= i; iy += a->size[0]) {
              i17 = iy + a->size[0];
              for (ic = iy; ic + 1 <= i17; ic++) {
                b_C->data[ic] = 0.0F;
              }
            }

            br = 0;
            for (iy = 0; iy <= i; iy += a->size[0]) {
              ar = 0;
              i17 = br + a->size[1];
              for (ib = br; ib + 1 <= i17; ib++) {
                if (b->data[ib] != 0.0F) {
                  ia = ar;
                  i18 = iy + a->size[0];
                  for (ic = iy; ic + 1 <= i18; ic++) {
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

        emxFree_real32_T(&b);
        emxFree_real32_T(&a);
        eml_li_find(selector, r50);
        i17 = c_a->size[0] * c_a->size[1];
        c_a->size[0] = r50->size[0];
        c_a->size[1] = 4;
        emxEnsureCapacity((emxArray__common *)c_a, i17, (int32_T)sizeof(real32_T));
        for (i17 = 0; i17 < 4; i17++) {
          i = r50->size[0];
          for (i18 = 0; i18 < i; i18++) {
            c_a->data[i18 + c_a->size[0] * i17] = K[(r50->data[i18] + 20 * i17)
              - 1];
          }
        }

        emxInit_real32_T(&d_y, 2);
        unnamed_idx_0 = (uint32_T)c_a->size[0];
        i17 = d_y->size[0] * d_y->size[1];
        d_y->size[0] = (int32_T)unnamed_idx_0;
        d_y->size[1] = 4;
        emxEnsureCapacity((emxArray__common *)d_y, i17, (int32_T)sizeof(real32_T));
        i = (int32_T)unnamed_idx_0 << 2;
        for (i17 = 0; i17 < i; i17++) {
          d_y->data[i17] = 0.0F;
        }

        if (c_a->size[0] == 0) {
        } else {
          i = c_a->size[0] * 3;
          for (iy = 0; iy <= i; iy += c_a->size[0]) {
            i17 = iy + c_a->size[0];
            for (ic = iy; ic + 1 <= i17; ic++) {
              d_y->data[ic] = 0.0F;
            }
          }

          br = 0;
          for (iy = 0; iy <= i; iy += c_a->size[0]) {
            ar = 0;
            for (ib = br; ib + 1 <= br + 4; ib++) {
              if (S[ib] != 0.0F) {
                ia = ar;
                i17 = iy + c_a->size[0];
                for (ic = iy; ic + 1 <= i17; ic++) {
                  ia++;
                  d_y->data[ic] += S[ib] * c_a->data[ia - 1];
                }
              }

              ar += c_a->size[0];
            }

            br += 4;
          }
        }

        emxFree_real32_T(&c_a);
        eml_li_find(selector, r50);
        i17 = b_a->size[0] * b_a->size[1];
        b_a->size[0] = 4;
        b_a->size[1] = r50->size[0];
        emxEnsureCapacity((emxArray__common *)b_a, i17, (int32_T)sizeof(real32_T));
        i = r50->size[0];
        for (i17 = 0; i17 < i; i17++) {
          for (i18 = 0; i18 < 4; i18++) {
            b_a->data[i18 + b_a->size[0] * i17] = K[(r50->data[i17] + 20 * i18)
              - 1];
          }
        }

        unnamed_idx_0 = (uint32_T)d_y->size[0];
        unnamed_idx_1 = (uint32_T)b_a->size[1];
        i17 = KH->size[0] * KH->size[1];
        KH->size[0] = (int32_T)unnamed_idx_0;
        emxEnsureCapacity((emxArray__common *)KH, i17, (int32_T)sizeof(real32_T));
        i17 = KH->size[0] * KH->size[1];
        KH->size[1] = (int32_T)unnamed_idx_1;
        emxEnsureCapacity((emxArray__common *)KH, i17, (int32_T)sizeof(real32_T));
        i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
        for (i17 = 0; i17 < i; i17++) {
          KH->data[i17] = 0.0F;
        }

        if ((d_y->size[0] == 0) || (b_a->size[1] == 0)) {
        } else {
          i = d_y->size[0] * (b_a->size[1] - 1);
          for (iy = 0; iy <= i; iy += d_y->size[0]) {
            i17 = iy + d_y->size[0];
            for (ic = iy; ic + 1 <= i17; ic++) {
              KH->data[ic] = 0.0F;
            }
          }

          br = 0;
          for (iy = 0; iy <= i; iy += d_y->size[0]) {
            ar = 0;
            for (ib = br; ib + 1 <= br + 4; ib++) {
              if (b_a->data[ib] != 0.0F) {
                ia = ar;
                i17 = iy + d_y->size[0];
                for (ic = iy; ic + 1 <= i17; ic++) {
                  ia++;
                  KH->data[ic] += b_a->data[ib] * d_y->data[ia - 1];
                }
              }

              ar += d_y->size[0];
            }

            br += 4;
          }
        }

        emxFree_real32_T(&d_y);
        emxFree_real32_T(&b_a);
        eml_li_find(selector, r50);
        i17 = r50->size[0];
        r50->size[0] = r50->size[0];
        emxEnsureCapacity((emxArray__common *)r50, i17, (int32_T)sizeof(int32_T));
        i = r50->size[0];
        for (i17 = 0; i17 < i; i17++) {
          r50->data[i17]--;
        }

        eml_li_find(selector, r51);
        i17 = r51->size[0];
        r51->size[0] = r51->size[0];
        emxEnsureCapacity((emxArray__common *)r51, i17, (int32_T)sizeof(int32_T));
        i = r51->size[0];
        for (i17 = 0; i17 < i; i17++) {
          r51->data[i17]--;
        }

        emxInit_int32_T(&r59, 1);
        i17 = r59->size[0];
        r59->size[0] = r51->size[0];
        emxEnsureCapacity((emxArray__common *)r59, i17, (int32_T)sizeof(int32_T));
        i = r51->size[0];
        for (i17 = 0; i17 < i; i17++) {
          r59->data[i17] = r51->data[i17];
        }

        emxFree_int32_T(&r51);
        emxInit_int32_T(&r60, 1);
        i17 = r60->size[0];
        r60->size[0] = r50->size[0];
        emxEnsureCapacity((emxArray__common *)r60, i17, (int32_T)sizeof(int32_T));
        i = r50->size[0];
        for (i17 = 0; i17 < i; i17++) {
          r60->data[i17] = r50->data[i17];
        }

        emxFree_int32_T(&r50);
        i = r58->size[1];
        for (i17 = 0; i17 < i; i17++) {
          iy = r58->size[0];
          for (i18 = 0; i18 < iy; i18++) {
            P[r60->data[i18] + 20 * r59->data[i17]] = ((r58->data[i18 +
              r58->size[0] * i17] - C->data[i18 + C->size[0] * i17]) - b_C->
              data[i18 + b_C->size[0] * i17]) + KH->data[i18 + KH->size[0] * i17];
          }
        }

        emxFree_int32_T(&r60);
        emxFree_int32_T(&r59);
        emxFree_real32_T(&b_C);
        emxFree_real32_T(&C);
        emxFree_real32_T(&r58);
        emxFree_real32_T(&KH);
        for (i17 = 0; i17 < 20; i17++) {
          delta_x[i17] = 0.0F;
          for (i18 = 0; i18 < 4; i18++) {
            delta_x[i17] += K[i17 + 20 * i18] * e[i18];
          }
        }

        /*  effective state update: */
        for (i17 = 0; i17 < 3; i17++) {
          x->p[i17] += (real_T)delta_x[i17];
        }

        for (i = 0; i < 3; i++) {
          v_N_rel[i] = 0.5F * delta_x[i + 3];
        }

        alpha = (real32_T)sqrt(dot(v_N_rel, v_N_rel));
        if ((real32_T)fabs(alpha) < 0.01F) {
          rho = alpha * alpha;
          y = ((1.0F - 0.166666672F * rho) + 0.00833333377F * (rho * rho)) -
            0.000198412701F * (rho * rho * rho);
        } else {
          y = (real32_T)sin(alpha) / alpha;
        }

        for (i = 0; i < 3; i++) {
          e[i] = y * v_N_rel[i];
        }

        e[3] = (real32_T)cos(alpha);

        /*  optimization (rectification of sinc) */
        /*  [ q3, q2, -q1, q0] */
        /*  [ -q2, q3, q0, q1] */
        /*  [ q1, -q0, q3, q2] */
        /*  [ -q0, -q1, -q2, q3] */
        /*  set float type */
        Q[0] = e[3];
        Q[4] = e[2];
        Q[8] = -e[1];
        Q[12] = e[0];
        Q[1] = -e[2];
        Q[5] = e[3];
        Q[9] = e[0];
        Q[13] = e[1];
        Q[2] = e[1];
        Q[6] = -e[0];
        Q[10] = e[3];
        Q[14] = e[2];
        Q[3] = -e[0];
        Q[7] = -e[1];
        Q[11] = -e[2];
        Q[15] = e[3];
        for (i17 = 0; i17 < 4; i17++) {
          e[i17] = 0.0F;
          for (i18 = 0; i18 < 4; i18++) {
            rho = e[i17] + Q[i17 + (i18 << 2)] * x->q_NS[i18];
            e[i17] = rho;
          }
        }

        rho = 0.0F;
        i = 0;
        iy = 0;
        for (br = 0; br < 4; br++) {
          rho += e[i] * e[iy];
          i++;
          iy++;
        }

        rho = (real32_T)sqrt(rho);
        for (i17 = 0; i17 < 4; i17++) {
          e[i17] /= rho;
        }

        for (i = 0; i < 4; i++) {
          x->q_NS[i] = e[i];
        }

        for (i17 = 0; i17 < 3; i17++) {
          x->v_N[i17] += delta_x[6 + i17];
        }

        for (i17 = 0; i17 < 3; i17++) {
          x->b_g[i17] += delta_x[9 + i17];
        }

        for (i17 = 0; i17 < 3; i17++) {
          x->b_a[i17] += delta_x[12 + i17];
        }

        x->QFF += delta_x[15];
        for (i17 = 0; i17 < 3; i17++) {
          x->w[i17] += delta_x[16 + i17];
        }

        x->K += delta_x[19];
      }
    }
  }
}

/* End of code generation (updatePressures_all.c) */
