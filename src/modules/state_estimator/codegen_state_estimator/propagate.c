/*
 * propagate.c
 *
 * Code generation for function 'propagate'
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
#include "autogen_FPF_const_wind.h"
#include "autogen_FPF.h"
#include "dot.h"
#include "HyEst_rtwutil.h"
#include "HyEst_data.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void propagate(states_T *x, real32_T P[400], const real32_T y_indirect[6],
               real32_T dt, boolean_T const_wind, const real32_T wind[2])
{
  real32_T C_WS[9];
  real32_T omega[3];
  int32_T i;
  real32_T acc[3];
  real32_T denom;
  real32_T scaling[3];
  real32_T c;
  real32_T b_c;
  real32_T y;
  real32_T dtheta_half_norm;
  real32_T dq[4];
  real32_T Q[16];
  int32_T iy;
  int32_T k;
  real32_T fv0[3];
  real32_T fv1[400];

  /*  */
  /*  [x,P] = PROPAGATE(x,P,y_indirect, dt) */
  /*  */
  /*  State propagation with indirect measurements */
  /*  */
  /*  Author: lestefan */
  /*  date:  02/2013 */
  /*  */
  /*  Inputs: */
  /*    x:            states [p(lat*1e7,lon*1e7,h),q_NS,v_N,b_g,b_a,QFF,w,K] */
  /*    P:            covariance */
  /*    y_indirect:   IMU readings 6x1 [gyro;accelerometers] [rad/s;m/s^2] */
  /*    dt:           time increment [s] */
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
  /*  See also UPDATEPOSITION, UPDATEVELNED, UPDATECOMPASS, UPDATEPRESSURES, */
  /*  UPDATEPRESSURES2, UPDATEPRESSURES3, GETINITIALQ, MAGFIELD */
  /* coder.ceval('propagate', coder.ref(x)); */
  /* global x_memory */
  /*  preparations */
  /*  ------------ */
  /*  set float type */
  C_WS[0] = ((x->q_NS[0] * x->q_NS[0] - x->q_NS[1] * x->q_NS[1]) - x->q_NS[2] *
             x->q_NS[2]) + x->q_NS[3] * x->q_NS[3];
  C_WS[3] = x->q_NS[0] * x->q_NS[1] * 2.0F + x->q_NS[2] * x->q_NS[3] * 2.0F;
  C_WS[6] = x->q_NS[0] * x->q_NS[2] * 2.0F - x->q_NS[1] * x->q_NS[3] * 2.0F;
  C_WS[1] = x->q_NS[0] * x->q_NS[1] * 2.0F - x->q_NS[2] * x->q_NS[3] * 2.0F;
  C_WS[4] = ((-x->q_NS[0] * x->q_NS[0] + x->q_NS[1] * x->q_NS[1]) - x->q_NS[2] *
             x->q_NS[2]) + x->q_NS[3] * x->q_NS[3];
  C_WS[7] = x->q_NS[0] * x->q_NS[3] * 2.0F + x->q_NS[1] * x->q_NS[2] * 2.0F;
  C_WS[2] = x->q_NS[0] * x->q_NS[2] * 2.0F + x->q_NS[1] * x->q_NS[3] * 2.0F;
  C_WS[5] = x->q_NS[0] * x->q_NS[3] * -2.0F + x->q_NS[1] * x->q_NS[2] * 2.0F;
  C_WS[8] = ((-x->q_NS[0] * x->q_NS[0] - x->q_NS[1] * x->q_NS[1]) + x->q_NS[2] *
             x->q_NS[2]) + x->q_NS[3] * x->q_NS[3];

  /*  decompose indirect measurements */
  for (i = 0; i < 3; i++) {
    omega[i] = y_indirect[i] - x->b_g[i];
  }

  /*  Gryo and accel bais compensention */
  for (i = 0; i < 3; i++) {
    acc[i] = y_indirect[i + 3] - x->b_a[i];
  }

  /*  calculate some intermediate variables */
  denom = (real32_T)sqrt(1.0F - 0.00669438F * rt_powf_snf((real32_T)sin
    ((real32_T)x->p[0] / 1.0E+7F), 2.0F));
  scaling[0] = 1.0E+7F / (6.3354395E+6F / rt_powf_snf(denom, 3.0F) + (real32_T)
    x->p[2]);
  scaling[1] = 1.0E+7F / ((6.378137E+6F / denom + (real32_T)x->p[2]) * (real32_T)
    cos((real32_T)x->p[0] / 1.0E+7F));
  scaling[2] = -1.0F;

  /*  this is Rn rewrite with constants */
  /*  gravity */
  c = rt_powf_snf((real32_T)sin((real32_T)x->p[0] / 1.0E+7F), 2.0F);
  b_c = rt_powf_snf((real32_T)sin(2.0F * (real32_T)x->p[0] / 1.0E+7F), 2.0F);
  y = (3.0877E-6F - 4.4E-9F * rt_powf_snf((real32_T)sin((real32_T)x->p[0] /
         1.0E+7F), 2.0F)) * (real32_T)x->p[2];

  /*  propagate the states */
  /*  -------------------- */
  /*  start with the pose */
  for (i = 0; i < 3; i++) {
    x->p[i] += (real_T)(dt * x->v_N[i] * scaling[i]);
  }

  /*  position state propogation  */
  for (i = 0; i < 3; i++) {
    omega[i] = 0.5F * (-dt * omega[i]);
  }

  dtheta_half_norm = (real32_T)sqrt(dot(omega, omega));
  if ((real32_T)fabs(dtheta_half_norm) < 0.01F) {
    denom = dtheta_half_norm * dtheta_half_norm;
    denom = ((1.0F - 0.166666672F * denom) + 0.00833333377F * (denom * denom)) -
      0.000198412701F * (denom * denom * denom);
  } else {
    denom = (real32_T)sin(dtheta_half_norm) / dtheta_half_norm;
  }

  for (i = 0; i < 3; i++) {
    dq[i] = denom * omega[i];
  }

  dq[3] = (real32_T)cos(dtheta_half_norm);

  /*  [ q3, -q2, q1, q0] */
  /*  [ q2, q3, -q0, q1] */
  /*  [ -q1, q0, q3, q2] */
  /*  [ -q0, -q1, -q2, q3] */
  /*  set float type */
  Q[0] = dq[3];
  Q[4] = -dq[2];
  Q[8] = dq[1];
  Q[12] = dq[0];
  Q[1] = dq[2];
  Q[5] = dq[3];
  Q[9] = -dq[0];
  Q[13] = dq[1];
  Q[2] = -dq[1];
  Q[6] = dq[0];
  Q[10] = dq[3];
  Q[14] = dq[2];
  Q[3] = -dq[0];
  Q[7] = -dq[1];
  Q[11] = -dq[2];
  Q[15] = dq[3];
  for (i = 0; i < 4; i++) {
    dq[i] = 0.0F;
    for (iy = 0; iy < 4; iy++) {
      denom = dq[i] + Q[i + (iy << 2)] * x->q_NS[iy];
      dq[i] = denom;
    }
  }

  denom = 0.0F;
  i = 0;
  iy = 0;
  for (k = 0; k < 4; k++) {
    denom += dq[i] * dq[iy];
    i++;
    iy++;
  }

  denom = (real32_T)sqrt(denom);
  for (i = 0; i < 4; i++) {
    dq[i] /= denom;
  }

  for (i = 0; i < 4; i++) {
    x->q_NS[i] = dq[i];
  }

  /*  Quaternion propatation */
  /*  the rest is straightforward */
  for (i = 0; i < 3; i++) {
    omega[i] = 0.0F;
    for (iy = 0; iy < 3; iy++) {
      omega[i] += C_WS[i + 3 * iy] * acc[iy];
    }
  }

  fv0[0] = 0.0F;
  fv0[1] = 0.0F;
  fv0[2] = 9.78032684F * ((1.0F + 0.0053024F * c) - 5.8E-6F * b_c) - y;
  for (i = 0; i < 3; i++) {
    x->v_N[i] += (omega[i] + fv0[i]) * dt;
  }

  /*  speed - note: z-down!!               % velocity NED propagation (accelerometer to NED and integration) */
  /*  acc biases */
  denom = 1.0F - dt / configuration.tau;
  for (i = 0; i < 3; i++) {
    x->b_a[i] *= denom;
  }

  /*  accelerometer correlated bias compensation                                                 */
  /*  wind: experimental: negative feedback... */
  if (const_wind) {
    y = dt / configuration.tau_w;
    for (i = 0; i < 2; i++) {
      x->w[i] -= y * (x->w[i] - wind[i]);
    }

    /* disp(x.w(1:2,:)') */
  }

  /*  save history */
  /* x_memory=[x_memory(:,2:size(x_memory,2)),x]; */
  /*  propagate the covariance */
  /*  ------------------------ */
  /*  Jacobian */
  /*  F=eye(20); */
  /*  F(  1:3,  7:9)=F(  1:3,  7:9)+diag(scaling)*dt; */
  /*  F(  4:6,10:12)=F(  4:6,10:12)+C_WS*dt; */
  /*  F(  7:9,  4:6)=F(  7:9,  4:6)+crossMx(C_WS*acc)*dt; */
  /*  F(  7:9,13:15)=F(  7:9,13:15)-C_WS*dt; */
  /*  F(13:15,13:15)=F(13:15,13:15)-eye(3)*dt/configuration.tau; */
  /* P=F*(P*F');                % about 30,000 floating point operations! */
  /*  this is the same but optimized, but untractable using the symbolic */
  /*  toolbox: */
  if (const_wind) {
    autogen_FPF_const_wind(P, C_WS, acc, scaling, configuration.tau,
      configuration.tau_w, dt, fv1);
    for (i = 0; i < 20; i++) {
      for (iy = 0; iy < 20; iy++) {
        P[iy + 20 * i] = fv1[iy + 20 * i];
      }
    }
  } else {
    autogen_FPF(P, C_WS, acc, scaling, configuration.tau, dt, fv1);
    for (i = 0; i < 20; i++) {
      for (iy = 0; iy < 20; iy++) {
        P[iy + 20 * i] = fv1[iy + 20 * i];
      }
    }
  }

  /*  add process noise */
  denom = configuration.sigma_g_c * configuration.sigma_g_c * dt;
  P[63] += denom;
  P[84] += denom;
  P[105] += denom;
  denom = configuration.sigma_a_c * configuration.sigma_a_c * dt;
  P[126] += denom;
  P[147] += denom;
  P[168] += denom;
  denom = configuration.sigma_gw_c * configuration.sigma_gw_c * dt;
  P[189] += denom;
  P[210] += denom;
  P[231] += denom;
  denom = configuration.sigma_aw_c * configuration.sigma_aw_c * dt;
  P[252] += denom;
  P[273] += denom;
  P[294] += denom;
  P[315] += configuration.sigma_pw_c * configuration.sigma_pw_c * dt;
  denom = configuration.sigma_w_c * configuration.sigma_w_c * dt;
  if (P[336] < 100.0F) {
    /*  when the wind variance is slower then (10 m/sec)^2 then add wind noise */
    P[336] += denom;

    /*  set float type          */
  }

  if (P[357] < 100.0F) {
    /*  when the wind variance is slower then (10 m/sec)^2 then add wind noise */
    P[357] += denom;

    /*  set float type */
  }

  if (P[378] < 49.0F) {
    /*  when the wind variance is slower then (7 m/sec)^2 then add wind noise */
    P[378] += denom;

    /*  set float type */
  }

  /*  force symmetric */
  /* eigenvalues=eig(P); */
  /* if(min(real(eigenvalues))<0) */
  /* disp('P not positive semidefinite!!') */
  /* end */
  /* P=0.5*P+0.5*P'; */
}

/* End of code generation (propagate.c) */
