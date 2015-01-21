/*
 * updatePressures2.c
 *
 * Code generation for function 'updatePressures2'
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
#include "dot.h"
#include "HyEst_emxutil.h"
#include "mldivide.h"
#include "mpower.h"
#include "diag.h"
#include "autogen_pressure2.h"
#include "HyEst_rtwutil.h"
#include "HyEst_data.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void updatePressures2(states_T *x, real32_T P[400], real32_T y_p_stat, real32_T
                      y_p_dyn, real32_T T_h, real32_T ell_offset, const real32_T
                      acc[3], const airplane_T airplane, boolean_T constWind,
                      boolean_T constQFF, boolean_T constK, boolean_T
                      *p_stat_valid, boolean_T *on_ground, boolean_T
                      *aerodyn_valid)
{
  real32_T rho;
  real32_T dtheta_half_norm;
  real32_T C_AB[9];
  real32_T C_SN[9];
  int32_T i23;
  int32_T i24;
  real32_T v_N_rel[3];
  int32_T i;
  real32_T b_C_SN[3];
  real32_T b_acc[6];
  real32_T c_C_SN[9];
  real32_T R[36];
  real32_T E_z[24];
  real32_T dq[4];
  real32_T X[9];
  int32_T ar;
  real32_T fv3[180];
  static const int8_T iv9[20] = { 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0 };

  static const int8_T iv10[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const int8_T iv11[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0 };

  static const int8_T iv12[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1 };

  real32_T E_x[80];
  real32_T H[80];
  real32_T fv4[6];
  real32_T b_E_x[80];
  real32_T b_E_z[24];
  real32_T c_E_x[16];
  real32_T c_E_z[16];
  real32_T S[16];
  real32_T y[4];
  real32_T K[80];
  boolean_T selector[20];
  emxArray_int32_T *r71;
  boolean_T b_selector[20];
  emxArray_int32_T *r72;
  emxArray_real32_T *a;
  emxArray_int32_T *r73;
  emxArray_int32_T *r74;
  int32_T iy;
  emxArray_int32_T *r75;
  emxArray_int32_T *r76;
  emxArray_int32_T *r77;
  emxArray_int32_T *r78;
  emxArray_real32_T *b_a;
  emxArray_real32_T *b;
  emxArray_real32_T *b_y;
  uint32_T unnamed_idx_1;
  int32_T ic;
  int32_T br;
  int32_T ib;
  int32_T ia;
  emxArray_real32_T *c_a;
  emxArray_real32_T *KH;
  uint32_T unnamed_idx_0;
  emxArray_real32_T *r79;
  emxArray_real32_T *C;
  emxArray_real32_T *b_C;
  emxArray_real32_T *c_y;
  emxArray_int32_T *r80;
  emxArray_int32_T *r81;
  real32_T delta_x[20];
  real32_T Q[16];

  /*  */
  /*  [x,P,p_stat_valid,on_ground,aero_valid] =... */
  /*     UPDATEPRESSURES3(x,P,y_p_stat,y_p_dyn,T_h,ell_offset,... */
  /*     acc,airplane,m,rho0,vmin,constWind,constQFF,constK) */
  /*  */
  /*  Kalman filter update with static and dynamic pressure: version with  */
  /*  velocity polar. */
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
  /*    c_LD_v:       v_down(IAS) polynom parameters 3x1  */
  /*    m:            actual and reference masses 2x1 [kg] */
  /*    rho0:         reference air density [kg/m^3] */
  /*    vmin:         stall speed at reference mass and air density [m/s] */
  /*    constWind:    keep wind constant [true/false] (use true on GPS out) */
  /*    constQFF:     keep QFF constant [true/false] (use true on GPS out) */
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
  /*  UPDATEPRESSURES, UPDATEPRESSURES3, PROPATATE, GETINITIALQ, MAGFIELD */
  /*  */
  /*  TODO: update in the past, re-propagate. This might not be fast enough... */
  /* eml.inline('always'); */
  /* p_stat_valid = false; */
  *on_ground = FALSE;
  *aerodyn_valid = FALSE;
  *p_stat_valid = TRUE;

  /*  get some global parameters */
  /*  air density */
  rho = y_p_stat * 1000.0F / (287.058F * T_h);

  /*  aerodynamics scale */
  /*  measuerements */
  if ((y_p_dyn < 0.0F) || rtIsNaNF(y_p_dyn)) {
    dtheta_half_norm = 0.0F;
  } else {
    dtheta_half_norm = y_p_dyn;
  }

  /* disp(constQFF) */
  /* fprintf('%f ',vx_dash) */
  if ((real32_T)sqrt(2.0F * dtheta_half_norm * 1000.0F / rho) < airplane.vmin *
      (real32_T)sqrt(airplane.ref_mass[0] * (real32_T)sqrt((rt_powf_snf(acc[0],
          2.0F) + rt_powf_snf(acc[1], 2.0F)) + rt_powf_snf(acc[2], 2.0F)) *
                     airplane.rho0 / (airplane.ref_mass[1] * 9.81F * rho))) {
    /* fprintf('on ground.\n') */
    *p_stat_valid = b_updatePressures(x, P, y_p_stat, T_h, ell_offset, constQFF,
      constK);
    *on_ground = TRUE;
  } else {
    /* fprintf('airborne!\n') */
    *aerodyn_valid = TRUE;

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
    for (i23 = 0; i23 < 3; i23++) {
      for (i24 = 0; i24 < 3; i24++) {
        C_SN[i24 + 3 * i23] = C_AB[i23 + 3 * i24];
      }
    }

    /*  helper states */
    for (i = 0; i < 3; i++) {
      v_N_rel[i] = x->v_N[i] - x->w[i];
    }

    /* disp(num2str(m(1)*sqrt(acc(1)^2+acc(3)^2))); */
    /* disp(num2str(m(1)*(acc(1)*v_S_rel(3)-acc(3)*v_S_rel(1))/sqrt(v_S_rel(3)^2+v_S_rel(1)^2))); */
    /* disp('-----') */
    /*  Deduct the component into flight direction... Kind of hack-ish. */
    /*  But doing it right makes the solution diverge for some reason. */
    /* acc2=acc-(acc(1)*v_S_rel(1)+acc(3)*v_S_rel(3))/(acc(1)^2+acc(3)^2)*acc; */
    /*  get error and Jacobians */
    C_AB[0] = (real32_T)x->p[2] - ell_offset;
    for (i23 = 0; i23 < 3; i23++) {
      b_C_SN[i23] = 0.0F;
      for (i24 = 0; i24 < 3; i24++) {
        b_C_SN[i23] += C_SN[i23 + 3 * i24] * v_N_rel[i24];
      }

      C_AB[i23 + 1] = b_C_SN[i23];
    }

    for (i23 = 0; i23 < 3; i23++) {
      C_AB[i23 + 4] = x->b_a[i23];
    }

    C_AB[7] = x->QFF;
    C_AB[8] = x->K;
    for (i = 0; i < 3; i++) {
      b_acc[i] = acc[i];
      for (i23 = 0; i23 < 3; i23++) {
        c_C_SN[i23 + 3 * i] = -C_SN[i23 + 3 * i];
      }
    }

    b_acc[3] = y_p_stat;
    b_acc[4] = y_p_dyn;
    b_acc[5] = T_h;
    autogen_pressure2(C_AB, b_acc, airplane.LD_v, airplane.ref_mass[0],
                      airplane.ref_mass[1], airplane.rho0, airplane.A, dq, E_z,
                      R);

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
    for (i23 = 0; i23 < 3; i23++) {
      for (i24 = 0; i24 < 3; i24++) {
        C_AB[i23 + 3 * i24] = 0.0F;
        for (ar = 0; ar < 3; ar++) {
          C_AB[i23 + 3 * i24] += c_C_SN[i23 + 3 * ar] * X[ar + 3 * i24];
        }
      }
    }

    for (i23 = 0; i23 < 20; i23++) {
      fv3[9 * i23] = (real32_T)iv9[i23];
    }

    for (i23 = 0; i23 < 3; i23++) {
      for (i24 = 0; i24 < 3; i24++) {
        fv3[(i24 + 9 * i23) + 1] = 0.0F;
      }
    }

    for (i23 = 0; i23 < 3; i23++) {
      for (i24 = 0; i24 < 3; i24++) {
        fv3[(i24 + 9 * (i23 + 3)) + 1] = C_AB[i24 + 3 * i23];
      }
    }

    for (i23 = 0; i23 < 3; i23++) {
      for (i24 = 0; i24 < 3; i24++) {
        fv3[(i24 + 9 * (i23 + 6)) + 1] = C_SN[i24 + 3 * i23];
      }
    }

    for (i23 = 0; i23 < 7; i23++) {
      for (i24 = 0; i24 < 3; i24++) {
        fv3[(i24 + 9 * (i23 + 9)) + 1] = 0.0F;
      }
    }

    for (i23 = 0; i23 < 3; i23++) {
      for (i24 = 0; i24 < 3; i24++) {
        fv3[(i24 + 9 * (i23 + 16)) + 1] = -C_SN[i24 + 3 * i23];
      }
    }

    for (i23 = 0; i23 < 3; i23++) {
      fv3[i23 + 172] = 0.0F;
    }

    for (i23 = 0; i23 < 20; i23++) {
      for (i24 = 0; i24 < 3; i24++) {
        fv3[(i24 + 9 * i23) + 4] = (real32_T)iv10[i24 + 3 * i23];
      }

      fv3[7 + 9 * i23] = (real32_T)iv11[i23];
      fv3[8 + 9 * i23] = (real32_T)iv12[i23];
    }

    for (i23 = 0; i23 < 4; i23++) {
      for (i24 = 0; i24 < 20; i24++) {
        E_x[i23 + (i24 << 2)] = 0.0F;
        for (ar = 0; ar < 9; ar++) {
          E_x[i23 + (i24 << 2)] += R[i23 + (ar << 2)] * fv3[ar + 9 * i24];
        }
      }
    }

    for (i23 = 0; i23 < 80; i23++) {
      H[i23] = -E_x[i23];
    }

    /*  measurement covariance */
    fv4[0] = 0.1F;
    fv4[1] = 0.1F;
    fv4[2] = 0.1F;
    fv4[3] = rt_powf_snf(configuration.sigma_psmd, 2.0F);
    fv4[4] = rt_powf_snf(configuration.sigma_pdmd, 2.0F);
    fv4[5] = rt_powf_snf(configuration.sigma_Td, 2.0F);
    d_diag(fv4, R);

    /*  residual covariance: */
    for (i23 = 0; i23 < 4; i23++) {
      for (i24 = 0; i24 < 20; i24++) {
        b_E_x[i23 + (i24 << 2)] = 0.0F;
        for (ar = 0; ar < 20; ar++) {
          b_E_x[i23 + (i24 << 2)] += E_x[i23 + (ar << 2)] * P[ar + 20 * i24];
        }
      }
    }

    for (i23 = 0; i23 < 4; i23++) {
      for (i24 = 0; i24 < 6; i24++) {
        b_E_z[i23 + (i24 << 2)] = 0.0F;
        for (ar = 0; ar < 6; ar++) {
          b_E_z[i23 + (i24 << 2)] += E_z[i23 + (ar << 2)] * R[ar + 6 * i24];
        }
      }

      for (i24 = 0; i24 < 4; i24++) {
        c_E_x[i23 + (i24 << 2)] = 0.0F;
        for (ar = 0; ar < 20; ar++) {
          c_E_x[i23 + (i24 << 2)] += b_E_x[i23 + (ar << 2)] * E_x[i24 + (ar << 2)];
        }

        c_E_z[i23 + (i24 << 2)] = 0.0F;
        for (ar = 0; ar < 6; ar++) {
          c_E_z[i23 + (i24 << 2)] += b_E_z[i23 + (ar << 2)] * E_z[i24 + (ar << 2)];
        }
      }
    }

    for (i23 = 0; i23 < 4; i23++) {
      for (i24 = 0; i24 < 4; i24++) {
        S[i24 + (i23 << 2)] = c_E_x[i24 + (i23 << 2)] + c_E_z[i24 + (i23 << 2)];
      }
    }

    /*  z-chi2: */
    /* disp(E_x) */
    /* disp(E_z) */
    /* disp(S) */
    mpower(S, c_E_x);
    rho = 0.0F;
    for (i23 = 0; i23 < 4; i23++) {
      y[i23] = 0.0F;
      for (i24 = 0; i24 < 4; i24++) {
        y[i23] += dq[i24] * c_E_x[i24 + (i23 << 2)];
      }

      rho += y[i23] * dq[i23];
    }

    /* disp(num2str(chi2_z)); */
    /*  consistency checks TODO */
    /* disp(e'*S^(-1)*e); */
    if (rho > 25.0F) {
      /* disp('pressures1d') */
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

      emxInit_int32_T(&r71, 1);

      /*  remove correlations of fixed states with active states */
      eml_li_find(selector, r71);
      i23 = r71->size[0];
      r71->size[0] = r71->size[0];
      emxEnsureCapacity((emxArray__common *)r71, i23, (int32_T)sizeof(int32_T));
      i = r71->size[0];
      for (i23 = 0; i23 < i; i23++) {
        r71->data[i23]--;
      }

      for (i = 0; i < 20; i++) {
        b_selector[i] = !selector[i];
      }

      emxInit_int32_T(&r72, 1);
      eml_li_find(b_selector, r72);
      i23 = r72->size[0];
      r72->size[0] = r72->size[0];
      emxEnsureCapacity((emxArray__common *)r72, i23, (int32_T)sizeof(int32_T));
      i = r72->size[0];
      for (i23 = 0; i23 < i; i23++) {
        r72->data[i23]--;
      }

      for (i = 0; i < 20; i++) {
        b_selector[i] = !selector[i];
      }

      emxInit_real32_T(&a, 2);
      emxInit_int32_T(&r73, 1);
      emxInit_int32_T(&r74, 1);
      eml_li_find(b_selector, r73);
      eml_li_find(selector, r74);
      i23 = a->size[0] * a->size[1];
      a->size[0] = r74->size[0];
      a->size[1] = r73->size[0];
      emxEnsureCapacity((emxArray__common *)a, i23, (int32_T)sizeof(real32_T));
      i = r73->size[0];
      for (i23 = 0; i23 < i; i23++) {
        iy = r74->size[0];
        for (i24 = 0; i24 < iy; i24++) {
          a->data[i24 + a->size[0] * i23] = P[(r74->data[i24] + 20 * (r73->
            data[i23] - 1)) - 1];
        }
      }

      emxInit_int32_T(&r75, 1);
      i23 = r75->size[0];
      r75->size[0] = r72->size[0];
      emxEnsureCapacity((emxArray__common *)r75, i23, (int32_T)sizeof(int32_T));
      i = r72->size[0];
      for (i23 = 0; i23 < i; i23++) {
        r75->data[i23] = r72->data[i23];
      }

      emxInit_int32_T(&r76, 1);
      i23 = r76->size[0];
      r76->size[0] = r71->size[0];
      emxEnsureCapacity((emxArray__common *)r76, i23, (int32_T)sizeof(int32_T));
      i = r71->size[0];
      for (i23 = 0; i23 < i; i23++) {
        r76->data[i23] = r71->data[i23];
      }

      i = a->size[1];
      for (i23 = 0; i23 < i; i23++) {
        iy = a->size[0];
        for (i24 = 0; i24 < iy; i24++) {
          P[r76->data[i24] + 20 * r75->data[i23]] = a->data[i24 + a->size[0] *
            i23] * 0.0F;
        }
      }

      emxFree_int32_T(&r76);
      emxFree_int32_T(&r75);
      for (i = 0; i < 20; i++) {
        b_selector[i] = !selector[i];
      }

      eml_li_find(b_selector, r71);
      i23 = r71->size[0];
      r71->size[0] = r71->size[0];
      emxEnsureCapacity((emxArray__common *)r71, i23, (int32_T)sizeof(int32_T));
      i = r71->size[0];
      for (i23 = 0; i23 < i; i23++) {
        r71->data[i23]--;
      }

      eml_li_find(selector, r72);
      i23 = r72->size[0];
      r72->size[0] = r72->size[0];
      emxEnsureCapacity((emxArray__common *)r72, i23, (int32_T)sizeof(int32_T));
      i = r72->size[0];
      for (i23 = 0; i23 < i; i23++) {
        r72->data[i23]--;
      }

      eml_li_find(selector, r73);
      for (i = 0; i < 20; i++) {
        b_selector[i] = !selector[i];
      }

      eml_li_find(b_selector, r74);
      i23 = a->size[0] * a->size[1];
      a->size[0] = r74->size[0];
      a->size[1] = r73->size[0];
      emxEnsureCapacity((emxArray__common *)a, i23, (int32_T)sizeof(real32_T));
      i = r73->size[0];
      for (i23 = 0; i23 < i; i23++) {
        iy = r74->size[0];
        for (i24 = 0; i24 < iy; i24++) {
          a->data[i24 + a->size[0] * i23] = P[(r74->data[i24] + 20 * (r73->
            data[i23] - 1)) - 1];
        }
      }

      emxFree_int32_T(&r74);
      emxInit_int32_T(&r77, 1);
      i23 = r77->size[0];
      r77->size[0] = r72->size[0];
      emxEnsureCapacity((emxArray__common *)r77, i23, (int32_T)sizeof(int32_T));
      i = r72->size[0];
      for (i23 = 0; i23 < i; i23++) {
        r77->data[i23] = r72->data[i23];
      }

      emxInit_int32_T(&r78, 1);
      i23 = r78->size[0];
      r78->size[0] = r71->size[0];
      emxEnsureCapacity((emxArray__common *)r78, i23, (int32_T)sizeof(int32_T));
      i = r71->size[0];
      for (i23 = 0; i23 < i; i23++) {
        r78->data[i23] = r71->data[i23];
      }

      i = a->size[1];
      for (i23 = 0; i23 < i; i23++) {
        iy = a->size[0];
        for (i24 = 0; i24 < iy; i24++) {
          P[r78->data[i24] + 20 * r77->data[i23]] = a->data[i24 + a->size[0] *
            i23] * 0.0F;
        }
      }

      emxFree_int32_T(&r78);
      emxFree_int32_T(&r77);
      eml_li_find(selector, r71);
      i23 = r71->size[0];
      r71->size[0] = r71->size[0];
      emxEnsureCapacity((emxArray__common *)r71, i23, (int32_T)sizeof(int32_T));
      i = r71->size[0];
      for (i23 = 0; i23 < i; i23++) {
        r71->data[i23]--;
      }

      emxInit_real32_T(&b_a, 2);
      eml_li_find(selector, r72);
      i23 = b_a->size[0] * b_a->size[1];
      b_a->size[0] = 4;
      b_a->size[1] = r72->size[0];
      emxEnsureCapacity((emxArray__common *)b_a, i23, (int32_T)sizeof(real32_T));
      i = r72->size[0];
      for (i23 = 0; i23 < i; i23++) {
        for (i24 = 0; i24 < 4; i24++) {
          b_a->data[i24 + b_a->size[0] * i23] = H[i24 + ((r72->data[i23] - 1) <<
            2)];
        }
      }

      emxInit_real32_T(&b, 2);
      eml_li_find(selector, r72);
      eml_li_find(selector, r73);
      i23 = b->size[0] * b->size[1];
      b->size[0] = r73->size[0];
      b->size[1] = r72->size[0];
      emxEnsureCapacity((emxArray__common *)b, i23, (int32_T)sizeof(real32_T));
      i = r72->size[0];
      for (i23 = 0; i23 < i; i23++) {
        iy = r73->size[0];
        for (i24 = 0; i24 < iy; i24++) {
          b->data[i24 + b->size[0] * i23] = P[(r73->data[i24] + 20 * (r72->
            data[i23] - 1)) - 1];
        }
      }

      emxFree_int32_T(&r73);
      emxInit_real32_T(&b_y, 2);
      if ((b_a->size[1] == 1) || (b->size[0] == 1)) {
        i23 = b_y->size[0] * b_y->size[1];
        b_y->size[0] = 4;
        b_y->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)b_y, i23, (int32_T)sizeof(real32_T));
        for (i23 = 0; i23 < 4; i23++) {
          i = b->size[1];
          for (i24 = 0; i24 < i; i24++) {
            b_y->data[i23 + b_y->size[0] * i24] = 0.0F;
            iy = b_a->size[1];
            for (ar = 0; ar < iy; ar++) {
              b_y->data[i23 + b_y->size[0] * i24] += b_a->data[i23 + b_a->size[0]
                * ar] * b->data[ar + b->size[0] * i24];
            }
          }
        }
      } else {
        unnamed_idx_1 = (uint32_T)b->size[1];
        i23 = b_y->size[0] * b_y->size[1];
        b_y->size[0] = 4;
        emxEnsureCapacity((emxArray__common *)b_y, i23, (int32_T)sizeof(real32_T));
        i23 = b_y->size[0] * b_y->size[1];
        b_y->size[1] = (int32_T)unnamed_idx_1;
        emxEnsureCapacity((emxArray__common *)b_y, i23, (int32_T)sizeof(real32_T));
        i = (int32_T)unnamed_idx_1 << 2;
        for (i23 = 0; i23 < i; i23++) {
          b_y->data[i23] = 0.0F;
        }

        if (b->size[1] == 0) {
        } else {
          i = (b->size[1] - 1) << 2;
          for (iy = 0; iy <= i; iy += 4) {
            for (ic = iy; ic + 1 <= iy + 4; ic++) {
              b_y->data[ic] = 0.0F;
            }
          }

          br = 0;
          for (iy = 0; iy <= i; iy += 4) {
            ar = 0;
            i23 = br + b_a->size[1];
            for (ib = br; ib + 1 <= i23; ib++) {
              if (b->data[ib] != 0.0F) {
                ia = ar;
                for (ic = iy; ic + 1 <= iy + 4; ic++) {
                  ia++;
                  b_y->data[ic] += b->data[ib] * b_a->data[ia - 1];
                }
              }

              ar += 4;
            }

            br += b_a->size[1];
          }
        }
      }

      emxInit_real32_T(&c_a, 2);
      b_mldivide(S, b_y, b_a);
      i23 = c_a->size[0] * c_a->size[1];
      c_a->size[0] = b_a->size[1];
      c_a->size[1] = 4;
      emxEnsureCapacity((emxArray__common *)c_a, i23, (int32_T)sizeof(real32_T));
      emxFree_real32_T(&b_y);
      for (i23 = 0; i23 < 4; i23++) {
        i = b_a->size[1];
        for (i24 = 0; i24 < i; i24++) {
          c_a->data[i24 + c_a->size[0] * i23] = b_a->data[i23 + b_a->size[0] *
            i24];
        }
      }

      for (i23 = 0; i23 < 4; i23++) {
        i = c_a->size[0];
        for (i24 = 0; i24 < i; i24++) {
          K[r71->data[i24] + 20 * i23] = c_a->data[i24 + c_a->size[0] * i23];
        }
      }

      /*  update: */
      eml_li_find(selector, r71);
      i23 = c_a->size[0] * c_a->size[1];
      c_a->size[0] = r71->size[0];
      c_a->size[1] = 4;
      emxEnsureCapacity((emxArray__common *)c_a, i23, (int32_T)sizeof(real32_T));
      for (i23 = 0; i23 < 4; i23++) {
        i = r71->size[0];
        for (i24 = 0; i24 < i; i24++) {
          c_a->data[i24 + c_a->size[0] * i23] = K[(r71->data[i24] + 20 * i23) -
            1];
        }
      }

      eml_li_find(selector, r71);
      i23 = b_a->size[0] * b_a->size[1];
      b_a->size[0] = 4;
      b_a->size[1] = r71->size[0];
      emxEnsureCapacity((emxArray__common *)b_a, i23, (int32_T)sizeof(real32_T));
      i = r71->size[0];
      for (i23 = 0; i23 < i; i23++) {
        for (i24 = 0; i24 < 4; i24++) {
          b_a->data[i24 + b_a->size[0] * i23] = H[i24 + ((r71->data[i23] - 1) <<
            2)];
        }
      }

      emxInit_real32_T(&KH, 2);
      unnamed_idx_0 = (uint32_T)c_a->size[0];
      unnamed_idx_1 = (uint32_T)b_a->size[1];
      i23 = KH->size[0] * KH->size[1];
      KH->size[0] = (int32_T)unnamed_idx_0;
      emxEnsureCapacity((emxArray__common *)KH, i23, (int32_T)sizeof(real32_T));
      i23 = KH->size[0] * KH->size[1];
      KH->size[1] = (int32_T)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)KH, i23, (int32_T)sizeof(real32_T));
      i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
      for (i23 = 0; i23 < i; i23++) {
        KH->data[i23] = 0.0F;
      }

      if ((c_a->size[0] == 0) || (b_a->size[1] == 0)) {
      } else {
        i = c_a->size[0] * (b_a->size[1] - 1);
        for (iy = 0; iy <= i; iy += c_a->size[0]) {
          i23 = iy + c_a->size[0];
          for (ic = iy; ic + 1 <= i23; ic++) {
            KH->data[ic] = 0.0F;
          }
        }

        br = 0;
        for (iy = 0; iy <= i; iy += c_a->size[0]) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 4; ib++) {
            if (b_a->data[ib] != 0.0F) {
              ia = ar;
              i23 = iy + c_a->size[0];
              for (ic = iy; ic + 1 <= i23; ic++) {
                ia++;
                KH->data[ic] += b_a->data[ib] * c_a->data[ia - 1];
              }
            }

            ar += c_a->size[0];
          }

          br += 4;
        }
      }

      emxInit_real32_T(&r79, 2);
      eml_li_find(selector, r71);
      eml_li_find(selector, r72);
      i23 = r79->size[0] * r79->size[1];
      r79->size[0] = r72->size[0];
      r79->size[1] = r71->size[0];
      emxEnsureCapacity((emxArray__common *)r79, i23, (int32_T)sizeof(real32_T));
      i = r71->size[0];
      for (i23 = 0; i23 < i; i23++) {
        iy = r72->size[0];
        for (i24 = 0; i24 < iy; i24++) {
          r79->data[i24 + r79->size[0] * i23] = P[(r72->data[i24] + 20 *
            (r71->data[i23] - 1)) - 1];
        }
      }

      eml_li_find(selector, r71);
      eml_li_find(selector, r72);
      i23 = b->size[0] * b->size[1];
      b->size[0] = r72->size[0];
      b->size[1] = r71->size[0];
      emxEnsureCapacity((emxArray__common *)b, i23, (int32_T)sizeof(real32_T));
      i = r71->size[0];
      for (i23 = 0; i23 < i; i23++) {
        iy = r72->size[0];
        for (i24 = 0; i24 < iy; i24++) {
          b->data[i24 + b->size[0] * i23] = P[(r72->data[i24] + 20 * (r71->
            data[i23] - 1)) - 1];
        }
      }

      emxInit_real32_T(&C, 2);
      if ((KH->size[1] == 1) || (b->size[0] == 1)) {
        i23 = C->size[0] * C->size[1];
        C->size[0] = KH->size[0];
        C->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)C, i23, (int32_T)sizeof(real32_T));
        i = KH->size[0];
        for (i23 = 0; i23 < i; i23++) {
          iy = b->size[1];
          for (i24 = 0; i24 < iy; i24++) {
            C->data[i23 + C->size[0] * i24] = 0.0F;
            br = KH->size[1];
            for (ar = 0; ar < br; ar++) {
              C->data[i23 + C->size[0] * i24] += KH->data[i23 + KH->size[0] * ar]
                * b->data[ar + b->size[0] * i24];
            }
          }
        }
      } else {
        unnamed_idx_0 = (uint32_T)KH->size[0];
        unnamed_idx_1 = (uint32_T)b->size[1];
        i23 = C->size[0] * C->size[1];
        C->size[0] = (int32_T)unnamed_idx_0;
        emxEnsureCapacity((emxArray__common *)C, i23, (int32_T)sizeof(real32_T));
        i23 = C->size[0] * C->size[1];
        C->size[1] = (int32_T)unnamed_idx_1;
        emxEnsureCapacity((emxArray__common *)C, i23, (int32_T)sizeof(real32_T));
        i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
        for (i23 = 0; i23 < i; i23++) {
          C->data[i23] = 0.0F;
        }

        if ((KH->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          i = KH->size[0] * (b->size[1] - 1);
          for (iy = 0; iy <= i; iy += KH->size[0]) {
            i23 = iy + KH->size[0];
            for (ic = iy; ic + 1 <= i23; ic++) {
              C->data[ic] = 0.0F;
            }
          }

          br = 0;
          for (iy = 0; iy <= i; iy += KH->size[0]) {
            ar = 0;
            i23 = br + KH->size[1];
            for (ib = br; ib + 1 <= i23; ib++) {
              if (b->data[ib] != 0.0F) {
                ia = ar;
                i24 = iy + KH->size[0];
                for (ic = iy; ic + 1 <= i24; ic++) {
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

      eml_li_find(selector, r71);
      eml_li_find(selector, r72);
      i23 = a->size[0] * a->size[1];
      a->size[0] = r72->size[0];
      a->size[1] = r71->size[0];
      emxEnsureCapacity((emxArray__common *)a, i23, (int32_T)sizeof(real32_T));
      i = r71->size[0];
      for (i23 = 0; i23 < i; i23++) {
        iy = r72->size[0];
        for (i24 = 0; i24 < iy; i24++) {
          a->data[i24 + a->size[0] * i23] = P[(r72->data[i24] + 20 * (r71->
            data[i23] - 1)) - 1];
        }
      }

      i23 = b->size[0] * b->size[1];
      b->size[0] = KH->size[1];
      b->size[1] = KH->size[0];
      emxEnsureCapacity((emxArray__common *)b, i23, (int32_T)sizeof(real32_T));
      i = KH->size[0];
      for (i23 = 0; i23 < i; i23++) {
        iy = KH->size[1];
        for (i24 = 0; i24 < iy; i24++) {
          b->data[i24 + b->size[0] * i23] = KH->data[i23 + KH->size[0] * i24];
        }
      }

      emxInit_real32_T(&b_C, 2);
      if ((a->size[1] == 1) || (b->size[0] == 1)) {
        i23 = b_C->size[0] * b_C->size[1];
        b_C->size[0] = a->size[0];
        b_C->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)b_C, i23, (int32_T)sizeof(real32_T));
        i = a->size[0];
        for (i23 = 0; i23 < i; i23++) {
          iy = b->size[1];
          for (i24 = 0; i24 < iy; i24++) {
            b_C->data[i23 + b_C->size[0] * i24] = 0.0F;
            br = a->size[1];
            for (ar = 0; ar < br; ar++) {
              b_C->data[i23 + b_C->size[0] * i24] += a->data[i23 + a->size[0] *
                ar] * b->data[ar + b->size[0] * i24];
            }
          }
        }
      } else {
        unnamed_idx_0 = (uint32_T)a->size[0];
        unnamed_idx_1 = (uint32_T)b->size[1];
        i23 = b_C->size[0] * b_C->size[1];
        b_C->size[0] = (int32_T)unnamed_idx_0;
        emxEnsureCapacity((emxArray__common *)b_C, i23, (int32_T)sizeof(real32_T));
        i23 = b_C->size[0] * b_C->size[1];
        b_C->size[1] = (int32_T)unnamed_idx_1;
        emxEnsureCapacity((emxArray__common *)b_C, i23, (int32_T)sizeof(real32_T));
        i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
        for (i23 = 0; i23 < i; i23++) {
          b_C->data[i23] = 0.0F;
        }

        if ((a->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          i = a->size[0] * (b->size[1] - 1);
          for (iy = 0; iy <= i; iy += a->size[0]) {
            i23 = iy + a->size[0];
            for (ic = iy; ic + 1 <= i23; ic++) {
              b_C->data[ic] = 0.0F;
            }
          }

          br = 0;
          for (iy = 0; iy <= i; iy += a->size[0]) {
            ar = 0;
            i23 = br + a->size[1];
            for (ib = br; ib + 1 <= i23; ib++) {
              if (b->data[ib] != 0.0F) {
                ia = ar;
                i24 = iy + a->size[0];
                for (ic = iy; ic + 1 <= i24; ic++) {
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
      eml_li_find(selector, r71);
      i23 = c_a->size[0] * c_a->size[1];
      c_a->size[0] = r71->size[0];
      c_a->size[1] = 4;
      emxEnsureCapacity((emxArray__common *)c_a, i23, (int32_T)sizeof(real32_T));
      for (i23 = 0; i23 < 4; i23++) {
        i = r71->size[0];
        for (i24 = 0; i24 < i; i24++) {
          c_a->data[i24 + c_a->size[0] * i23] = K[(r71->data[i24] + 20 * i23) -
            1];
        }
      }

      emxInit_real32_T(&c_y, 2);
      unnamed_idx_0 = (uint32_T)c_a->size[0];
      i23 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = (int32_T)unnamed_idx_0;
      c_y->size[1] = 4;
      emxEnsureCapacity((emxArray__common *)c_y, i23, (int32_T)sizeof(real32_T));
      i = (int32_T)unnamed_idx_0 << 2;
      for (i23 = 0; i23 < i; i23++) {
        c_y->data[i23] = 0.0F;
      }

      if (c_a->size[0] == 0) {
      } else {
        i = c_a->size[0] * 3;
        for (iy = 0; iy <= i; iy += c_a->size[0]) {
          i23 = iy + c_a->size[0];
          for (ic = iy; ic + 1 <= i23; ic++) {
            c_y->data[ic] = 0.0F;
          }
        }

        br = 0;
        for (iy = 0; iy <= i; iy += c_a->size[0]) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 4; ib++) {
            if (S[ib] != 0.0F) {
              ia = ar;
              i23 = iy + c_a->size[0];
              for (ic = iy; ic + 1 <= i23; ic++) {
                ia++;
                c_y->data[ic] += S[ib] * c_a->data[ia - 1];
              }
            }

            ar += c_a->size[0];
          }

          br += 4;
        }
      }

      emxFree_real32_T(&c_a);
      eml_li_find(selector, r71);
      i23 = b_a->size[0] * b_a->size[1];
      b_a->size[0] = 4;
      b_a->size[1] = r71->size[0];
      emxEnsureCapacity((emxArray__common *)b_a, i23, (int32_T)sizeof(real32_T));
      i = r71->size[0];
      for (i23 = 0; i23 < i; i23++) {
        for (i24 = 0; i24 < 4; i24++) {
          b_a->data[i24 + b_a->size[0] * i23] = K[(r71->data[i23] + 20 * i24) -
            1];
        }
      }

      unnamed_idx_0 = (uint32_T)c_y->size[0];
      unnamed_idx_1 = (uint32_T)b_a->size[1];
      i23 = KH->size[0] * KH->size[1];
      KH->size[0] = (int32_T)unnamed_idx_0;
      emxEnsureCapacity((emxArray__common *)KH, i23, (int32_T)sizeof(real32_T));
      i23 = KH->size[0] * KH->size[1];
      KH->size[1] = (int32_T)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)KH, i23, (int32_T)sizeof(real32_T));
      i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
      for (i23 = 0; i23 < i; i23++) {
        KH->data[i23] = 0.0F;
      }

      if ((c_y->size[0] == 0) || (b_a->size[1] == 0)) {
      } else {
        i = c_y->size[0] * (b_a->size[1] - 1);
        for (iy = 0; iy <= i; iy += c_y->size[0]) {
          i23 = iy + c_y->size[0];
          for (ic = iy; ic + 1 <= i23; ic++) {
            KH->data[ic] = 0.0F;
          }
        }

        br = 0;
        for (iy = 0; iy <= i; iy += c_y->size[0]) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 4; ib++) {
            if (b_a->data[ib] != 0.0F) {
              ia = ar;
              i23 = iy + c_y->size[0];
              for (ic = iy; ic + 1 <= i23; ic++) {
                ia++;
                KH->data[ic] += b_a->data[ib] * c_y->data[ia - 1];
              }
            }

            ar += c_y->size[0];
          }

          br += 4;
        }
      }

      emxFree_real32_T(&c_y);
      emxFree_real32_T(&b_a);
      eml_li_find(selector, r71);
      i23 = r71->size[0];
      r71->size[0] = r71->size[0];
      emxEnsureCapacity((emxArray__common *)r71, i23, (int32_T)sizeof(int32_T));
      i = r71->size[0];
      for (i23 = 0; i23 < i; i23++) {
        r71->data[i23]--;
      }

      eml_li_find(selector, r72);
      i23 = r72->size[0];
      r72->size[0] = r72->size[0];
      emxEnsureCapacity((emxArray__common *)r72, i23, (int32_T)sizeof(int32_T));
      i = r72->size[0];
      for (i23 = 0; i23 < i; i23++) {
        r72->data[i23]--;
      }

      emxInit_int32_T(&r80, 1);
      i23 = r80->size[0];
      r80->size[0] = r72->size[0];
      emxEnsureCapacity((emxArray__common *)r80, i23, (int32_T)sizeof(int32_T));
      i = r72->size[0];
      for (i23 = 0; i23 < i; i23++) {
        r80->data[i23] = r72->data[i23];
      }

      emxFree_int32_T(&r72);
      emxInit_int32_T(&r81, 1);
      i23 = r81->size[0];
      r81->size[0] = r71->size[0];
      emxEnsureCapacity((emxArray__common *)r81, i23, (int32_T)sizeof(int32_T));
      i = r71->size[0];
      for (i23 = 0; i23 < i; i23++) {
        r81->data[i23] = r71->data[i23];
      }

      emxFree_int32_T(&r71);
      i = r79->size[1];
      for (i23 = 0; i23 < i; i23++) {
        iy = r79->size[0];
        for (i24 = 0; i24 < iy; i24++) {
          P[r81->data[i24] + 20 * r80->data[i23]] = ((r79->data[i24 + r79->size
            [0] * i23] - C->data[i24 + C->size[0] * i23]) - b_C->data[i24 +
            b_C->size[0] * i23]) + KH->data[i24 + KH->size[0] * i23];
        }
      }

      emxFree_int32_T(&r81);
      emxFree_int32_T(&r80);
      emxFree_real32_T(&b_C);
      emxFree_real32_T(&C);
      emxFree_real32_T(&r79);
      emxFree_real32_T(&KH);
      for (i23 = 0; i23 < 20; i23++) {
        delta_x[i23] = 0.0F;
        for (i24 = 0; i24 < 4; i24++) {
          delta_x[i23] += K[i23 + 20 * i24] * dq[i24];
        }
      }

      /*  effective state update: */
      for (i23 = 0; i23 < 3; i23++) {
        x->p[i23] += (real_T)delta_x[i23];
      }

      for (i = 0; i < 3; i++) {
        v_N_rel[i] = 0.5F * delta_x[i + 3];
      }

      dtheta_half_norm = (real32_T)sqrt(dot(v_N_rel, v_N_rel));
      if ((real32_T)fabs(dtheta_half_norm) < 0.01F) {
        rho = dtheta_half_norm * dtheta_half_norm;
        rho = ((1.0F - 0.166666672F * rho) + 0.00833333377F * (rho * rho)) -
          0.000198412701F * (rho * rho * rho);
      } else {
        rho = (real32_T)sin(dtheta_half_norm) / dtheta_half_norm;
      }

      for (i = 0; i < 3; i++) {
        dq[i] = rho * v_N_rel[i];
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
      for (i23 = 0; i23 < 4; i23++) {
        dq[i23] = 0.0F;
        for (i24 = 0; i24 < 4; i24++) {
          rho = dq[i23] + Q[i23 + (i24 << 2)] * x->q_NS[i24];
          dq[i23] = rho;
        }
      }

      rho = 0.0F;
      i = 0;
      iy = 0;
      for (br = 0; br < 4; br++) {
        rho += dq[i] * dq[iy];
        i++;
        iy++;
      }

      rho = (real32_T)sqrt(rho);
      for (i23 = 0; i23 < 4; i23++) {
        dq[i23] /= rho;
      }

      for (i = 0; i < 4; i++) {
        x->q_NS[i] = dq[i];
      }

      for (i23 = 0; i23 < 3; i23++) {
        x->v_N[i23] += delta_x[6 + i23];
      }

      for (i23 = 0; i23 < 3; i23++) {
        x->b_g[i23] += delta_x[9 + i23];
      }

      for (i23 = 0; i23 < 3; i23++) {
        x->b_a[i23] += delta_x[12 + i23];
      }

      x->QFF += delta_x[15];
      for (i23 = 0; i23 < 3; i23++) {
        x->w[i23] += delta_x[16 + i23];
      }

      x->K += delta_x[19];
    }
  }
}

/* End of code generation (updatePressures2.c) */
