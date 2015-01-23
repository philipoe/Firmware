/*
 * updateCompass2.c
 *
 * Code generation for function 'updateCompass2'
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
#include "cross.h"
#include "diag.h"
#include "HyEst_rtwutil.h"
#include "HyEst_data.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void updateCompass2(states_T *x, real32_T P[400], const real32_T y_Compass[3],
                    const real32_T b_N[3], const real32_T acc[3], boolean_T
                    constWind, boolean_T constQFF, boolean_T constK)
{
  real32_T r;
  real32_T b_r[3];
  real32_T R_Compass[9];
  real32_T C_NS[9];
  int32_T i13;
  real32_T y_Compass_N[3];
  int32_T i14;
  real32_T b_p_N[3];
  static const int8_T b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 0 };

  real32_T y_p_N[3];
  real32_T X[9];
  real32_T b_X[9];
  real32_T y;
  int32_T ar;
  real32_T N[9];
  real32_T dtheta_half_norm;
  int32_T ib;
  int32_T i;
  real32_T c_X[9];
  static const int8_T a[3] = { 0, 0, 1 };

  real32_T b_a[3];
  real32_T E_z[3];
  real32_T d_X[9];
  real32_T e_X[9];
  real32_T K[20];
  real32_T e;
  static const int8_T iv4[3] = { 0, 0, 1 };

  real32_T H[20];
  real32_T b_y[3];
  boolean_T selector[20];
  emxArray_int32_T *r28;
  boolean_T b_selector[20];
  emxArray_int32_T *r29;
  emxArray_real32_T *c_a;
  emxArray_int32_T *r30;
  emxArray_int32_T *r31;
  int32_T br;
  emxArray_int32_T *r32;
  emxArray_int32_T *r33;
  emxArray_int32_T *r34;
  emxArray_int32_T *r35;
  emxArray_real32_T *d_a;
  emxArray_real32_T *b_b;
  emxArray_real32_T *c_y;
  uint32_T unnamed_idx_1;
  int32_T ic;
  int32_T ia;
  emxArray_real32_T *e_a;
  emxArray_real32_T *KH;
  emxArray_real32_T *r36;
  emxArray_real32_T *C;
  uint32_T unnamed_idx_0;
  int32_T c;
  emxArray_int32_T *r37;
  emxArray_int32_T *r38;
  real32_T dq[4];
  real32_T Q[16];

  /*  */
  /*  [x,P] = UPDATECOMPASS2(x,P,y_Compass, b_N, e_acc, constWind, constQFF,constK) */
  /*  */
  /*  Kalman filter update with 3-axis compass measurements. */
  /*  */
  /*  Author: lestefan */
  /*  date:  02/2013 */
  /*  */
  /*  Inputs: */
  /*    x:            states [p(lat*1e7,lon*1e7,h),q_NS,v_N,b_g,b_a,QFF,w,K] */
  /*    P:            covariance */
  /*    y_Compass:    magnetic field measurement 3x1 (in sensor frame S) [uT] */
  /*    b_N:          true magnetic flux density 3x1 (nav frame N) [uT] */
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
  /*    configuration.sigma_a_d:   accelerometer standard diviation [m/s^2] */
  /*    configuration.sigma_aw_c:  accelerometer bias noise [m/s^3/sqrt(Hz)] */
  /*    configuration.sigma_g_c:   gyro noise [rad/s/sqrt(Hz)] */
  /*    configuration.sigma_gw_c:  gyro bias noise [rad/s^2/sqrt(Hz)] */
  /*    configuration.sigma_pw_c:  static pressure bias noise [kPa/s/sqrt(Hz)] */
  /*    configuration.sigma_w_c:   wind drift noise [m/s^2/sqrt(Hz)] */
  /*    configuration.sigma_psmd:  static pressure measurement variance [kPa] */
  /*    configuration.sigma_pdmd:  dynamic pressure measurement variance [kPa] */
  /*    configuration.sigma_Td=1:  temperature measurement variance [°K] */
  /*    configuration.sigma_md=1:  magnetometer noise [uT] */
  /*  */
  /*  See also UPDATEPOSITION, UPDATEVELNED, UPDATEPRESSURES,  */
  /*  UPDATEPRESSURES2, UPDATEPRESSURES3, PROPATATE, GETINITIALQ */
  /*  */
  r = rt_powf_snf(configuration.sigma_md, 2.0F);

  /* R_Compass=single(diag(configuration.sigma_md^2*single([1 1 1]))); */
  b_r[0] = r;
  b_r[1] = r;
  b_r[2] = r;
  b_diag(b_r, R_Compass);

  /* *configuration.sigma_md; */
  /*  TODO: update in the past, re-propagate. This might not be fast enough... */
  /*  set float type */
  C_NS[0] = ((x->q_NS[0] * x->q_NS[0] - x->q_NS[1] * x->q_NS[1]) - x->q_NS[2] *
             x->q_NS[2]) + x->q_NS[3] * x->q_NS[3];
  C_NS[3] = x->q_NS[0] * x->q_NS[1] * 2.0F + x->q_NS[2] * x->q_NS[3] * 2.0F;
  C_NS[6] = x->q_NS[0] * x->q_NS[2] * 2.0F - x->q_NS[1] * x->q_NS[3] * 2.0F;
  C_NS[1] = x->q_NS[0] * x->q_NS[1] * 2.0F - x->q_NS[2] * x->q_NS[3] * 2.0F;
  C_NS[4] = ((-x->q_NS[0] * x->q_NS[0] + x->q_NS[1] * x->q_NS[1]) - x->q_NS[2] *
             x->q_NS[2]) + x->q_NS[3] * x->q_NS[3];
  C_NS[7] = x->q_NS[0] * x->q_NS[3] * 2.0F + x->q_NS[1] * x->q_NS[2] * 2.0F;
  C_NS[2] = x->q_NS[0] * x->q_NS[2] * 2.0F + x->q_NS[1] * x->q_NS[3] * 2.0F;
  C_NS[5] = x->q_NS[0] * x->q_NS[3] * -2.0F + x->q_NS[1] * x->q_NS[2] * 2.0F;
  C_NS[8] = ((-x->q_NS[0] * x->q_NS[0] - x->q_NS[1] * x->q_NS[1]) + x->q_NS[2] *
             x->q_NS[2]) + x->q_NS[3] * x->q_NS[3];

  /*  error term: */
  for (i13 = 0; i13 < 3; i13++) {
    y_Compass_N[i13] = 0.0F;
    for (i14 = 0; i14 < 3; i14++) {
      y_Compass_N[i13] += C_NS[i13 + 3 * i14] * y_Compass[i14];
    }

    /* b_S=C_NS'*b_N; */
    /*  error and Jacobians: */
    /* [e,J]=autogen_angleJacobian2d(y_Compass_N,b_N); */
    /* dz=1e-3; */
    /* y_Compass_N_1p=C_NS*(y_Compass+[dz;0;0]); */
    /* y_Compass_N_2p=C_NS*(y_Compass+[0;dz;0]); */
    /* y_Compass_N_3p=C_NS*(y_Compass+[0;0;dz]); */
    /* y_Compass_N_1m=C_NS*(y_Compass-[dz;0;0]); */
    /* y_Compass_N_2m=C_NS*(y_Compass-[0;dz;0]); */
    /* y_Compass_N_3m=C_NS*(y_Compass-[0;0;dz]); */
    /* [e1,J]=autogen_angleJacobian2d(y_Compass_N_1p,b_N); */
    /* [e2,J]=autogen_angleJacobian2d(y_Compass_N_2p,b_N); */
    /* [e3,J]=autogen_angleJacobian2d(y_Compass_N_3p,b_N); */
    /* [e1,J]=autogen_angleJacobian2d(y_Compass_N_1m,b_N); */
    /* [e2,J]=autogen_angleJacobian2d(y_Compass_N_2m,b_N); */
    /* [e3,J]=autogen_angleJacobian2d(y_Compass_N_3m,b_N); */
    /* E_x=[single(zeros(1,3)),J*crossMx(y_Compass_N),single(zeros(1,14))]; */
    /* E_z=J*C_NS; */
    b_p_N[i13] = 0.0F;
    for (i14 = 0; i14 < 3; i14++) {
      b_p_N[i13] += (real32_T)b[i13 + 3 * i14] * b_N[i14];
    }
  }

  for (i13 = 0; i13 < 3; i13++) {
    y_p_N[i13] = 0.0F;
    for (i14 = 0; i14 < 3; i14++) {
      y_p_N[i13] += (real32_T)b[i13 + 3 * i14] * y_Compass_N[i14];
    }
  }

  /*  set float type */
  X[0] = 0.0F;
  X[3] = -y_p_N[2];
  X[6] = y_p_N[1];
  X[1] = y_p_N[2];
  X[4] = 0.0F;
  X[7] = -y_p_N[0];
  X[2] = -y_p_N[1];
  X[5] = y_p_N[0];
  X[8] = 0.0F;

  /*  set float type */
  b_X[0] = 0.0F;
  b_X[3] = -y_p_N[2];
  b_X[6] = y_p_N[1];
  b_X[1] = y_p_N[2];
  b_X[4] = 0.0F;
  b_X[7] = -y_p_N[0];
  b_X[2] = -y_p_N[1];
  b_X[5] = y_p_N[0];
  b_X[8] = 0.0F;
  y = 0.0F;
  for (ar = 0; ar < 3; ar++) {
    y += y_p_N[ar] * y_p_N[ar];
  }

  r = rt_powf_snf((real32_T)sqrt(y), 3.0F);
  for (i13 = 0; i13 < 3; i13++) {
    for (i14 = 0; i14 < 3; i14++) {
      dtheta_half_norm = 0.0F;
      for (ib = 0; ib < 3; ib++) {
        dtheta_half_norm += X[i13 + 3 * ib] * b_X[i14 + 3 * ib];
      }

      N[i13 + 3 * i14] = dtheta_half_norm / r;
    }
  }

  y = 0.0F;
  for (ar = 0; ar < 3; ar++) {
    y += b_p_N[ar] * b_p_N[ar];
  }

  r = (real32_T)sqrt(y);
  for (i = 0; i < 3; i++) {
    b_r[i] = -b_p_N[i] / r;
  }

  /*  set float type */
  c_X[0] = 0.0F;
  c_X[3] = -b_r[2];
  c_X[6] = b_r[1];
  c_X[1] = b_r[2];
  c_X[4] = 0.0F;
  c_X[7] = -b_r[0];
  c_X[2] = -b_r[1];
  c_X[5] = b_r[0];
  c_X[8] = 0.0F;
  for (i13 = 0; i13 < 3; i13++) {
    b_r[i13] = 0.0F;
    for (i14 = 0; i14 < 3; i14++) {
      b_r[i13] += (real32_T)a[i14] * c_X[i14 + 3 * i13];
    }
  }

  for (i13 = 0; i13 < 3; i13++) {
    b_a[i13] = 0.0F;
    for (i14 = 0; i14 < 3; i14++) {
      b_a[i13] += b_r[i14] * N[i14 + 3 * i13];
    }
  }

  for (i13 = 0; i13 < 3; i13++) {
    b_r[i13] = 0.0F;
    for (i14 = 0; i14 < 3; i14++) {
      b_r[i13] += b_a[i14] * (real32_T)b[i14 + 3 * i13];
    }
  }

  y = 0.0F;
  for (i13 = 0; i13 < 3; i13++) {
    E_z[i13] = 0.0F;
    for (i14 = 0; i14 < 3; i14++) {
      E_z[i13] += b_r[i14] * C_NS[i14 + 3 * i13];
    }

    y += b_p_N[i13] * b_p_N[i13];
  }

  r = (real32_T)sqrt(y);
  for (i = 0; i < 3; i++) {
    b_r[i] = -b_p_N[i] / r;
  }

  /*  set float type */
  d_X[0] = 0.0F;
  d_X[3] = -b_r[2];
  d_X[6] = b_r[1];
  d_X[1] = b_r[2];
  d_X[4] = 0.0F;
  d_X[7] = -b_r[0];
  d_X[2] = -b_r[1];
  d_X[5] = b_r[0];
  d_X[8] = 0.0F;

  /*  set float type */
  e_X[0] = 0.0F;
  e_X[3] = -y_Compass_N[2];
  e_X[6] = y_Compass_N[1];
  e_X[1] = y_Compass_N[2];
  e_X[4] = 0.0F;
  e_X[7] = -y_Compass_N[0];
  e_X[2] = -y_Compass_N[1];
  e_X[5] = y_Compass_N[0];
  e_X[8] = 0.0F;
  for (i13 = 0; i13 < 3; i13++) {
    b_r[i13] = 0.0F;
    for (i14 = 0; i14 < 3; i14++) {
      b_r[i13] += (real32_T)a[i14] * d_X[i14 + 3 * i13];
    }
  }

  for (i13 = 0; i13 < 3; i13++) {
    b_a[i13] = 0.0F;
    for (i14 = 0; i14 < 3; i14++) {
      b_a[i13] += b_r[i14] * N[i14 + 3 * i13];
    }
  }

  for (i13 = 0; i13 < 3; i13++) {
    b_r[i13] = 0.0F;
    for (i14 = 0; i14 < 3; i14++) {
      b_r[i13] += b_a[i14] * (real32_T)b[i14 + 3 * i13];
    }
  }

  for (i13 = 0; i13 < 3; i13++) {
    b_a[i13] = 0.0F;
    for (i14 = 0; i14 < 3; i14++) {
      b_a[i13] += b_r[i14] * e_X[i14 + 3 * i13];
    }

    K[i13] = 0.0F;
  }

  for (i13 = 0; i13 < 3; i13++) {
    K[i13 + 3] = b_a[i13];
  }

  for (i13 = 0; i13 < 14; i13++) {
    K[i13 + 6] = 0.0F;
  }

  y = 0.0F;
  for (ar = 0; ar < 3; ar++) {
    y += y_p_N[ar] * y_p_N[ar];
  }

  r = (real32_T)sqrt(y);
  y = 0.0F;
  for (ar = 0; ar < 3; ar++) {
    y += b_p_N[ar] * b_p_N[ar];
    b_a[ar] = y_p_N[ar] / r;
  }

  r = (real32_T)sqrt(y);
  for (i = 0; i < 3; i++) {
    b_r[i] = b_p_N[i] / r;
  }

  cross(b_a, b_r, y_Compass_N);
  e = 0.0F;
  for (ar = 0; ar < 3; ar++) {
    e += (real32_T)iv4[ar] * y_Compass_N[ar];
  }

  /* e1p=Pi_z*cross(Pi_xy0*y_Compass_N_1p/sqrt(y_Compass_N_1p'*y_Compass_N_1p),Pi_xy0*e_N); */
  /* e2p=Pi_z*cross(Pi_xy0*y_Compass_N_2p/sqrt(y_Compass_N_2p'*y_Compass_N_2p),Pi_xy0*e_N); */
  /* e3p=Pi_z*cross(Pi_xy0*y_Compass_N_3p/sqrt(y_Compass_N_3p'*y_Compass_N_3p),Pi_xy0*e_N); */
  /* e1m=Pi_z*cross(Pi_xy0*y_Compass_N_1m/sqrt(y_Compass_N_1m'*y_Compass_N_1m),Pi_xy0*e_N); */
  /* e2m=Pi_z*cross(Pi_xy0*y_Compass_N_2m/sqrt(y_Compass_N_2m'*y_Compass_N_2m),Pi_xy0*e_N); */
  /* e3m=Pi_z*cross(Pi_xy0*y_Compass_N_3m/sqrt(y_Compass_N_3m'*y_Compass_N_3m),Pi_xy0*e_N); */
  /* num_diff_Ez = [(e1p-e1m)/(2*dz) (e2p-e2m)/(2*dz) (e3p-e3m)/(2*dz)]; */
  /*  e_acc=single([0;0;1]); % rather random, but if mounted as advised, it's ok */
  /*  norm_acc_sq=acc'*acc; */
  /*  if norm_acc_sq>1 */
  /*      e_acc=acc/sqrt(norm_acc_sq); */
  /*  end */
  /*  [e,E_b_S,E_z]=autogen_angleJacobian2dAccDir(y_Compass,b_S,e_acc); */
  /*  E_x=[single(zeros(1,3)),-E_b_S*C_NS'*crossMx(b_N),single(zeros(1,14))]; */
  for (i13 = 0; i13 < 20; i13++) {
    H[i13] = -K[i13];
  }

  for (i13 = 0; i13 < 3; i13++) {
    b_y[i13] = 0.0F;
    for (i14 = 0; i14 < 3; i14++) {
      y = b_y[i13] + K[3 + i14] * P[(i14 + 20 * (3 + i13)) + 3];
      b_y[i13] = y;
    }
  }

  y = 0.0F;
  r = 0.0F;
  for (ar = 0; ar < 3; ar++) {
    y += b_y[ar] * K[3 + ar];
    b_y[ar] = 0.0F;
    for (i13 = 0; i13 < 3; i13++) {
      b_y[ar] += E_z[i13] * R_Compass[i13 + 3 * ar];
    }

    r += b_y[ar] * E_z[ar];
  }

  r += y;
  if (e * rt_powf_snf(r, -1.0F) * e > 100.0F) {
    /* disp(['magnetometer chi2 ' num2str(chi2)]); */
  } else {
    /* K=(S\(H(1:3,4:6)*P(4:6,:)))'; % gain */
    for (i = 0; i < 20; i++) {
      K[i] = 0.0F;

      /*  set float type */
      selector[i] = TRUE;
    }

    if (constK) {
      selector[19] = FALSE;
    }

    if (constQFF) {
      selector[15] = FALSE;
    }

    emxInit_int32_T(&r28, 1);

    /*  remove correlations of fixed states with active states */
    eml_li_find(selector, r28);
    i13 = r28->size[0];
    r28->size[0] = r28->size[0];
    emxEnsureCapacity((emxArray__common *)r28, i13, (int32_T)sizeof(int32_T));
    i = r28->size[0];
    for (i13 = 0; i13 < i; i13++) {
      r28->data[i13]--;
    }

    for (i = 0; i < 20; i++) {
      b_selector[i] = !selector[i];
    }

    emxInit_int32_T(&r29, 1);
    eml_li_find(b_selector, r29);
    i13 = r29->size[0];
    r29->size[0] = r29->size[0];
    emxEnsureCapacity((emxArray__common *)r29, i13, (int32_T)sizeof(int32_T));
    i = r29->size[0];
    for (i13 = 0; i13 < i; i13++) {
      r29->data[i13]--;
    }

    for (i = 0; i < 20; i++) {
      b_selector[i] = !selector[i];
    }

    emxInit_real32_T(&c_a, 2);
    emxInit_int32_T(&r30, 1);
    emxInit_int32_T(&r31, 1);
    eml_li_find(b_selector, r30);
    eml_li_find(selector, r31);
    i13 = c_a->size[0] * c_a->size[1];
    c_a->size[0] = r31->size[0];
    c_a->size[1] = r30->size[0];
    emxEnsureCapacity((emxArray__common *)c_a, i13, (int32_T)sizeof(real32_T));
    i = r30->size[0];
    for (i13 = 0; i13 < i; i13++) {
      br = r31->size[0];
      for (i14 = 0; i14 < br; i14++) {
        c_a->data[i14 + c_a->size[0] * i13] = P[(r31->data[i14] + 20 *
          (r30->data[i13] - 1)) - 1];
      }
    }

    emxInit_int32_T(&r32, 1);
    i13 = r32->size[0];
    r32->size[0] = r29->size[0];
    emxEnsureCapacity((emxArray__common *)r32, i13, (int32_T)sizeof(int32_T));
    i = r29->size[0];
    for (i13 = 0; i13 < i; i13++) {
      r32->data[i13] = r29->data[i13];
    }

    emxInit_int32_T(&r33, 1);
    i13 = r33->size[0];
    r33->size[0] = r28->size[0];
    emxEnsureCapacity((emxArray__common *)r33, i13, (int32_T)sizeof(int32_T));
    i = r28->size[0];
    for (i13 = 0; i13 < i; i13++) {
      r33->data[i13] = r28->data[i13];
    }

    i = c_a->size[1];
    for (i13 = 0; i13 < i; i13++) {
      br = c_a->size[0];
      for (i14 = 0; i14 < br; i14++) {
        P[r33->data[i14] + 20 * r32->data[i13]] = c_a->data[i14 + c_a->size[0] *
          i13] * 0.0F;
      }
    }

    emxFree_int32_T(&r33);
    emxFree_int32_T(&r32);
    for (i = 0; i < 20; i++) {
      b_selector[i] = !selector[i];
    }

    eml_li_find(b_selector, r28);
    i13 = r28->size[0];
    r28->size[0] = r28->size[0];
    emxEnsureCapacity((emxArray__common *)r28, i13, (int32_T)sizeof(int32_T));
    i = r28->size[0];
    for (i13 = 0; i13 < i; i13++) {
      r28->data[i13]--;
    }

    eml_li_find(selector, r29);
    i13 = r29->size[0];
    r29->size[0] = r29->size[0];
    emxEnsureCapacity((emxArray__common *)r29, i13, (int32_T)sizeof(int32_T));
    i = r29->size[0];
    for (i13 = 0; i13 < i; i13++) {
      r29->data[i13]--;
    }

    eml_li_find(selector, r30);
    for (i = 0; i < 20; i++) {
      b_selector[i] = !selector[i];
    }

    eml_li_find(b_selector, r31);
    i13 = c_a->size[0] * c_a->size[1];
    c_a->size[0] = r31->size[0];
    c_a->size[1] = r30->size[0];
    emxEnsureCapacity((emxArray__common *)c_a, i13, (int32_T)sizeof(real32_T));
    i = r30->size[0];
    for (i13 = 0; i13 < i; i13++) {
      br = r31->size[0];
      for (i14 = 0; i14 < br; i14++) {
        c_a->data[i14 + c_a->size[0] * i13] = P[(r31->data[i14] + 20 *
          (r30->data[i13] - 1)) - 1];
      }
    }

    emxFree_int32_T(&r31);
    emxInit_int32_T(&r34, 1);
    i13 = r34->size[0];
    r34->size[0] = r29->size[0];
    emxEnsureCapacity((emxArray__common *)r34, i13, (int32_T)sizeof(int32_T));
    i = r29->size[0];
    for (i13 = 0; i13 < i; i13++) {
      r34->data[i13] = r29->data[i13];
    }

    emxInit_int32_T(&r35, 1);
    i13 = r35->size[0];
    r35->size[0] = r28->size[0];
    emxEnsureCapacity((emxArray__common *)r35, i13, (int32_T)sizeof(int32_T));
    i = r28->size[0];
    for (i13 = 0; i13 < i; i13++) {
      r35->data[i13] = r28->data[i13];
    }

    i = c_a->size[1];
    for (i13 = 0; i13 < i; i13++) {
      br = c_a->size[0];
      for (i14 = 0; i14 < br; i14++) {
        P[r35->data[i14] + 20 * r34->data[i13]] = c_a->data[i14 + c_a->size[0] *
          i13] * 0.0F;
      }
    }

    emxFree_int32_T(&r35);
    emxFree_int32_T(&r34);

    /* P(~selector,~selector)=P(~selector,~selector)*0; */
    eml_li_find(selector, r28);
    i13 = r28->size[0];
    r28->size[0] = r28->size[0];
    emxEnsureCapacity((emxArray__common *)r28, i13, (int32_T)sizeof(int32_T));
    i = r28->size[0];
    for (i13 = 0; i13 < i; i13++) {
      r28->data[i13]--;
    }

    emxInit_real32_T(&d_a, 2);
    eml_li_find(selector, r29);
    i13 = d_a->size[0] * d_a->size[1];
    d_a->size[0] = 1;
    d_a->size[1] = r29->size[0];
    emxEnsureCapacity((emxArray__common *)d_a, i13, (int32_T)sizeof(real32_T));
    i = r29->size[0];
    for (i13 = 0; i13 < i; i13++) {
      d_a->data[d_a->size[0] * i13] = H[r29->data[i13] - 1];
    }

    emxInit_real32_T(&b_b, 2);
    eml_li_find(selector, r29);
    eml_li_find(selector, r30);
    i13 = b_b->size[0] * b_b->size[1];
    b_b->size[0] = r30->size[0];
    b_b->size[1] = r29->size[0];
    emxEnsureCapacity((emxArray__common *)b_b, i13, (int32_T)sizeof(real32_T));
    i = r29->size[0];
    for (i13 = 0; i13 < i; i13++) {
      br = r30->size[0];
      for (i14 = 0; i14 < br; i14++) {
        b_b->data[i14 + b_b->size[0] * i13] = P[(r30->data[i14] + 20 *
          (r29->data[i13] - 1)) - 1];
      }
    }

    emxFree_int32_T(&r30);
    emxInit_real32_T(&c_y, 2);
    if ((d_a->size[1] == 1) || (b_b->size[0] == 1)) {
      i13 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = 1;
      c_y->size[1] = b_b->size[1];
      emxEnsureCapacity((emxArray__common *)c_y, i13, (int32_T)sizeof(real32_T));
      i = b_b->size[1];
      for (i13 = 0; i13 < i; i13++) {
        c_y->data[c_y->size[0] * i13] = 0.0F;
        br = d_a->size[1];
        for (i14 = 0; i14 < br; i14++) {
          c_y->data[c_y->size[0] * i13] += d_a->data[d_a->size[0] * i14] *
            b_b->data[i14 + b_b->size[0] * i13];
        }
      }
    } else {
      unnamed_idx_1 = (uint32_T)b_b->size[1];
      i13 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = 1;
      emxEnsureCapacity((emxArray__common *)c_y, i13, (int32_T)sizeof(real32_T));
      i13 = c_y->size[0] * c_y->size[1];
      c_y->size[1] = (int32_T)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)c_y, i13, (int32_T)sizeof(real32_T));
      i = (int32_T)unnamed_idx_1;
      for (i13 = 0; i13 < i; i13++) {
        c_y->data[i13] = 0.0F;
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
          i13 = br + d_a->size[1];
          for (ib = br; ib + 1 <= i13; ib++) {
            if (b_b->data[ib] != 0.0F) {
              ia = ar;
              for (ic = i; ic + 1 <= i + 1; ic++) {
                ia++;
                c_y->data[ic] += b_b->data[ib] * d_a->data[ia - 1];
              }
            }

            ar++;
          }

          br += d_a->size[1];
        }
      }
    }

    i = c_y->size[1];
    for (i13 = 0; i13 < i; i13++) {
      K[r28->data[i13]] = c_y->data[i13] / r;
    }

    emxFree_real32_T(&c_y);
    b_emxInit_real32_T(&e_a, 1);

    /*  check Jacobian */
    /*  delta=1e-12; */
    /*  db=[-C_SN*crossMx(b_N)*[delta;0;0],-C_SN*crossMx(b_N)*[0;delta;0],-C_SN*crossMx(b_N)*[0;0;delta]]; */
    /*  q1=quatUpdate(x(4:7),[delta;0;0]); */
    /*  q2=quatUpdate(x(4:7),[0;delta;0]); */
    /*  q3=quatUpdate(x(4:7),[0;0;delta]); */
    /*  db_finiteDiff=[quat2r(q1)'*b_N-C_SN*b_N, quat2r(q2)'*b_N-C_SN*b_N, quat2r(q3)'*b_N-C_SN*b_N]; */
    /*  db-db_finiteDiff */
    /*  update: */
    eml_li_find(selector, r28);
    i13 = e_a->size[0];
    e_a->size[0] = r28->size[0];
    emxEnsureCapacity((emxArray__common *)e_a, i13, (int32_T)sizeof(real32_T));
    i = r28->size[0];
    for (i13 = 0; i13 < i; i13++) {
      e_a->data[i13] = K[r28->data[i13] - 1];
    }

    eml_li_find(selector, r28);
    i13 = d_a->size[0] * d_a->size[1];
    d_a->size[0] = 1;
    d_a->size[1] = r28->size[0];
    emxEnsureCapacity((emxArray__common *)d_a, i13, (int32_T)sizeof(real32_T));
    i = r28->size[0];
    for (i13 = 0; i13 < i; i13++) {
      d_a->data[d_a->size[0] * i13] = H[r28->data[i13] - 1];
    }

    emxInit_real32_T(&KH, 2);
    i13 = KH->size[0] * KH->size[1];
    KH->size[0] = e_a->size[0];
    KH->size[1] = d_a->size[1];
    emxEnsureCapacity((emxArray__common *)KH, i13, (int32_T)sizeof(real32_T));
    i = e_a->size[0];
    for (i13 = 0; i13 < i; i13++) {
      br = d_a->size[1];
      for (i14 = 0; i14 < br; i14++) {
        KH->data[i13 + KH->size[0] * i14] = e_a->data[i13] * d_a->data[d_a->
          size[0] * i14];
      }
    }

    emxInit_real32_T(&r36, 2);
    eml_li_find(selector, r28);
    eml_li_find(selector, r29);
    i13 = r36->size[0] * r36->size[1];
    r36->size[0] = r29->size[0];
    r36->size[1] = r28->size[0];
    emxEnsureCapacity((emxArray__common *)r36, i13, (int32_T)sizeof(real32_T));
    i = r28->size[0];
    for (i13 = 0; i13 < i; i13++) {
      br = r29->size[0];
      for (i14 = 0; i14 < br; i14++) {
        r36->data[i14 + r36->size[0] * i13] = P[(r29->data[i14] + 20 *
          (r28->data[i13] - 1)) - 1];
      }
    }

    eml_li_find(selector, r28);
    eml_li_find(selector, r29);
    i13 = b_b->size[0] * b_b->size[1];
    b_b->size[0] = r29->size[0];
    b_b->size[1] = r28->size[0];
    emxEnsureCapacity((emxArray__common *)b_b, i13, (int32_T)sizeof(real32_T));
    i = r28->size[0];
    for (i13 = 0; i13 < i; i13++) {
      br = r29->size[0];
      for (i14 = 0; i14 < br; i14++) {
        b_b->data[i14 + b_b->size[0] * i13] = P[(r29->data[i14] + 20 *
          (r28->data[i13] - 1)) - 1];
      }
    }

    emxInit_real32_T(&C, 2);
    if ((KH->size[1] == 1) || (b_b->size[0] == 1)) {
      i13 = C->size[0] * C->size[1];
      C->size[0] = KH->size[0];
      C->size[1] = b_b->size[1];
      emxEnsureCapacity((emxArray__common *)C, i13, (int32_T)sizeof(real32_T));
      i = KH->size[0];
      for (i13 = 0; i13 < i; i13++) {
        br = b_b->size[1];
        for (i14 = 0; i14 < br; i14++) {
          C->data[i13 + C->size[0] * i14] = 0.0F;
          ar = KH->size[1];
          for (ib = 0; ib < ar; ib++) {
            C->data[i13 + C->size[0] * i14] += KH->data[i13 + KH->size[0] * ib] *
              b_b->data[ib + b_b->size[0] * i14];
          }
        }
      }
    } else {
      unnamed_idx_0 = (uint32_T)KH->size[0];
      unnamed_idx_1 = (uint32_T)b_b->size[1];
      i13 = C->size[0] * C->size[1];
      C->size[0] = (int32_T)unnamed_idx_0;
      emxEnsureCapacity((emxArray__common *)C, i13, (int32_T)sizeof(real32_T));
      i13 = C->size[0] * C->size[1];
      C->size[1] = (int32_T)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)C, i13, (int32_T)sizeof(real32_T));
      i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
      for (i13 = 0; i13 < i; i13++) {
        C->data[i13] = 0.0F;
      }

      if ((KH->size[0] == 0) || (b_b->size[1] == 0)) {
      } else {
        c = KH->size[0] * (b_b->size[1] - 1);
        for (i = 0; i <= c; i += KH->size[0]) {
          i13 = i + KH->size[0];
          for (ic = i; ic + 1 <= i13; ic++) {
            C->data[ic] = 0.0F;
          }
        }

        br = 0;
        for (i = 0; i <= c; i += KH->size[0]) {
          ar = 0;
          i13 = br + KH->size[1];
          for (ib = br; ib + 1 <= i13; ib++) {
            if (b_b->data[ib] != 0.0F) {
              ia = ar;
              i14 = i + KH->size[0];
              for (ic = i; ic + 1 <= i14; ic++) {
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

    eml_li_find(selector, r28);
    eml_li_find(selector, r29);
    i13 = c_a->size[0] * c_a->size[1];
    c_a->size[0] = r29->size[0];
    c_a->size[1] = r28->size[0];
    emxEnsureCapacity((emxArray__common *)c_a, i13, (int32_T)sizeof(real32_T));
    i = r28->size[0];
    for (i13 = 0; i13 < i; i13++) {
      br = r29->size[0];
      for (i14 = 0; i14 < br; i14++) {
        c_a->data[i14 + c_a->size[0] * i13] = P[(r29->data[i14] + 20 *
          (r28->data[i13] - 1)) - 1];
      }
    }

    i13 = b_b->size[0] * b_b->size[1];
    b_b->size[0] = KH->size[1];
    b_b->size[1] = KH->size[0];
    emxEnsureCapacity((emxArray__common *)b_b, i13, (int32_T)sizeof(real32_T));
    i = KH->size[0];
    for (i13 = 0; i13 < i; i13++) {
      br = KH->size[1];
      for (i14 = 0; i14 < br; i14++) {
        b_b->data[i14 + b_b->size[0] * i13] = KH->data[i13 + KH->size[0] * i14];
      }
    }

    if ((c_a->size[1] == 1) || (b_b->size[0] == 1)) {
      i13 = KH->size[0] * KH->size[1];
      KH->size[0] = c_a->size[0];
      KH->size[1] = b_b->size[1];
      emxEnsureCapacity((emxArray__common *)KH, i13, (int32_T)sizeof(real32_T));
      i = c_a->size[0];
      for (i13 = 0; i13 < i; i13++) {
        br = b_b->size[1];
        for (i14 = 0; i14 < br; i14++) {
          KH->data[i13 + KH->size[0] * i14] = 0.0F;
          ar = c_a->size[1];
          for (ib = 0; ib < ar; ib++) {
            KH->data[i13 + KH->size[0] * i14] += c_a->data[i13 + c_a->size[0] *
              ib] * b_b->data[ib + b_b->size[0] * i14];
          }
        }
      }
    } else {
      unnamed_idx_0 = (uint32_T)c_a->size[0];
      unnamed_idx_1 = (uint32_T)b_b->size[1];
      i13 = KH->size[0] * KH->size[1];
      KH->size[0] = (int32_T)unnamed_idx_0;
      emxEnsureCapacity((emxArray__common *)KH, i13, (int32_T)sizeof(real32_T));
      i13 = KH->size[0] * KH->size[1];
      KH->size[1] = (int32_T)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)KH, i13, (int32_T)sizeof(real32_T));
      i = (int32_T)unnamed_idx_0 * (int32_T)unnamed_idx_1;
      for (i13 = 0; i13 < i; i13++) {
        KH->data[i13] = 0.0F;
      }

      if ((c_a->size[0] == 0) || (b_b->size[1] == 0)) {
      } else {
        c = c_a->size[0] * (b_b->size[1] - 1);
        for (i = 0; i <= c; i += c_a->size[0]) {
          i13 = i + c_a->size[0];
          for (ic = i; ic + 1 <= i13; ic++) {
            KH->data[ic] = 0.0F;
          }
        }

        br = 0;
        for (i = 0; i <= c; i += c_a->size[0]) {
          ar = 0;
          i13 = br + c_a->size[1];
          for (ib = br; ib + 1 <= i13; ib++) {
            if (b_b->data[ib] != 0.0F) {
              ia = ar;
              i14 = i + c_a->size[0];
              for (ic = i; ic + 1 <= i14; ic++) {
                ia++;
                KH->data[ic] += b_b->data[ib] * c_a->data[ia - 1];
              }
            }

            ar += c_a->size[0];
          }

          br += c_a->size[1];
        }
      }
    }

    emxFree_real32_T(&b_b);
    emxFree_real32_T(&c_a);
    eml_li_find(selector, r28);
    i13 = e_a->size[0];
    e_a->size[0] = r28->size[0];
    emxEnsureCapacity((emxArray__common *)e_a, i13, (int32_T)sizeof(real32_T));
    i = r28->size[0];
    for (i13 = 0; i13 < i; i13++) {
      e_a->data[i13] = K[r28->data[i13] - 1];
    }

    eml_li_find(selector, r28);
    i13 = d_a->size[0] * d_a->size[1];
    d_a->size[0] = 1;
    d_a->size[1] = r28->size[0];
    emxEnsureCapacity((emxArray__common *)d_a, i13, (int32_T)sizeof(real32_T));
    i = r28->size[0];
    for (i13 = 0; i13 < i; i13++) {
      d_a->data[d_a->size[0] * i13] = K[r28->data[i13] - 1];
    }

    eml_li_find(selector, r28);
    i13 = r28->size[0];
    r28->size[0] = r28->size[0];
    emxEnsureCapacity((emxArray__common *)r28, i13, (int32_T)sizeof(int32_T));
    i = r28->size[0];
    for (i13 = 0; i13 < i; i13++) {
      r28->data[i13]--;
    }

    eml_li_find(selector, r29);
    i13 = r29->size[0];
    r29->size[0] = r29->size[0];
    emxEnsureCapacity((emxArray__common *)r29, i13, (int32_T)sizeof(int32_T));
    i = r29->size[0];
    for (i13 = 0; i13 < i; i13++) {
      r29->data[i13]--;
    }

    emxInit_int32_T(&r37, 1);
    i13 = r37->size[0];
    r37->size[0] = r29->size[0];
    emxEnsureCapacity((emxArray__common *)r37, i13, (int32_T)sizeof(int32_T));
    i = r29->size[0];
    for (i13 = 0; i13 < i; i13++) {
      r37->data[i13] = r29->data[i13];
    }

    emxFree_int32_T(&r29);
    emxInit_int32_T(&r38, 1);
    i13 = r38->size[0];
    r38->size[0] = r28->size[0];
    emxEnsureCapacity((emxArray__common *)r38, i13, (int32_T)sizeof(int32_T));
    i = r28->size[0];
    for (i13 = 0; i13 < i; i13++) {
      r38->data[i13] = r28->data[i13];
    }

    emxFree_int32_T(&r28);
    i = e_a->size[0];
    for (i13 = 0; i13 < i; i13++) {
      br = d_a->size[1];
      for (i14 = 0; i14 < br; i14++) {
        dtheta_half_norm = e_a->data[i13] * r * d_a->data[d_a->size[0] * i14];
        P[r38->data[i13] + 20 * r37->data[i14]] = ((r36->data[i13 + r36->size[0]
          * i14] - C->data[i13 + C->size[0] * i14]) - KH->data[i13 + KH->size[0]
          * i14]) + dtheta_half_norm;
      }
    }

    emxFree_int32_T(&r38);
    emxFree_int32_T(&r37);
    emxFree_real32_T(&e_a);
    emxFree_real32_T(&d_a);
    emxFree_real32_T(&C);
    emxFree_real32_T(&r36);
    emxFree_real32_T(&KH);
    for (i13 = 0; i13 < 20; i13++) {
      K[i13] *= e;
    }

    for (i13 = 0; i13 < 3; i13++) {
      x->p[i13] += (real_T)K[i13];
    }

    for (i = 0; i < 3; i++) {
      y_Compass_N[i] = 0.5F * K[i + 3];
    }

    dtheta_half_norm = (real32_T)sqrt(dot(y_Compass_N, y_Compass_N));
    if ((real32_T)fabs(dtheta_half_norm) < 0.01F) {
      r = dtheta_half_norm * dtheta_half_norm;
      y = ((1.0F - 0.166666672F * r) + 0.00833333377F * (r * r)) -
        0.000198412701F * (r * r * r);
    } else {
      y = (real32_T)sin(dtheta_half_norm) / dtheta_half_norm;
    }

    for (i = 0; i < 3; i++) {
      dq[i] = y * y_Compass_N[i];
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
    for (i13 = 0; i13 < 4; i13++) {
      dq[i13] = 0.0F;
      for (i14 = 0; i14 < 4; i14++) {
        r = dq[i13] + Q[i13 + (i14 << 2)] * x->q_NS[i14];
        dq[i13] = r;
      }
    }

    r = 0.0F;
    i = 0;
    br = 0;
    for (ar = 0; ar < 4; ar++) {
      r += dq[i] * dq[br];
      i++;
      br++;
    }

    r = (real32_T)sqrt(r);
    for (i13 = 0; i13 < 4; i13++) {
      dq[i13] /= r;
    }

    for (i = 0; i < 4; i++) {
      x->q_NS[i] = dq[i];
    }

    /*  Correct the angular state only with tilt D (tilt N and tilt E are Zero) */
    for (i13 = 0; i13 < 3; i13++) {
      x->v_N[i13] += K[6 + i13];
    }

    for (i13 = 0; i13 < 3; i13++) {
      x->b_g[i13] += K[9 + i13];
    }

    for (i13 = 0; i13 < 3; i13++) {
      x->b_a[i13] += K[12 + i13];
    }

    x->QFF += K[15];
    for (i13 = 0; i13 < 3; i13++) {
      x->w[i13] += K[16 + i13];
    }

    x->K += K[19];
  }
}

/* End of code generation (updateCompass2.c) */
