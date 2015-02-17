/*
 * initStates.c
 *
 * Code generation for function 'initStates'
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
#include "norm.h"
#include "HyEst_rtwutil.h"
#include "HyEst_data.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void initStates(const real_T pos[3], const real32_T acc_S[3], const real32_T
                gyr_S[3], const real32_T v_S_rel[3], const real32_T b_S[3],
                const real32_T b_N[3], states_T *x, real32_T P[400])
{
  real32_T acc[3];
  real32_T a[3];
  int32_T i;
  real32_T rot1[3];
  real32_T norm1;
  real32_T theta1;
  static const int8_T iv0[3] = { 0, 0, 1 };

  real32_T q1[4];
  real32_T R1[9];
  int32_T i0;
  real32_T rot2[3];
  real32_T b_a[3];
  real32_T b_acc[3];
  real32_T b_b_N[3];
  real32_T theta2;
  real32_T q2[4];
  real32_T Q[16];
  real32_T v[20];

  /*  */
  /*  [x,P] = INITSTATES(pos,acc_S,gyr_S,v_S_rel,b_S,b_N) */
  /*  */
  /*  State propagation with indirect measurements */
  /*  */
  /*  Author: lestefan */
  /*  date:  07/2013 */
  /*  */
  /*  Inputs: */
  /*    pos:          position: wgs84 [lat,lon,alt] [rad,rad,m] */
  /*    acc_S:        accelerometer readings 3x1 [m/s^2] */
  /*    gyr:          gyro readings 3x1 [rad/s] */
  /*    v_S_rel:      air speed 3x1 (in frame S) [m/s] - don't use ground speed */
  /*    b_S:          measured magnetic flux density 3x1 (sensor frame) [uT] */
  /*    b_N:          true magnetic field 3x1 (nav frame N) [uT] */
  /*  */
  /*  Outputs: */
  /*    x:            states [p(lat*1e7,lon*1e7,h),q_NS,v_N,b_g,b_a,QFF,w,K] */
  /*    P:            covariance */
  /*  */
  /*  See also UPDATEPOSITION, UPDATEVELNED, UPDATECOMPASS, UPDATEPRESSURES, */
  /*  UPDATEPRESSURES2, UPDATEPRESSURES3, PROPAGATE, MAGFIELD */
  /*  get the magnetic field model / initialize position: */
  x->p[0] = pos[0] * 1.0E+7;

  /*  x(1): state LAT */
  x->p[1] = pos[1] * 1.0E+7;

  /*  x(2): state LON */
  x->p[2] = pos[2];

  /*  x(3): state ALT */
  /*  */
  /*  [x,P] = GETINITIALQ(acc_S,gyr,v_S_rel,b_S,b_N) */
  /*  */
  /*  State propagation with indirect measurements */
  /*  */
  /*  Author: lestefan */
  /*  date:  02/2013 */
  /*  */
  /*  Inputs: */
  /*    acc_S:        accelerometer readings 3x1 [m/s^2] */
  /*    gyr:          gyro readings 3x1 [rad/s] */
  /*    v_S_rel:      air speed 3x1 (in frame S) [m/s] - don't use ground speed */
  /*    b_S:          measured magnetic flux density 3x1 (sensor frame) [uT] */
  /*    b_N:          true magnetic field 3x1 (nav frame N) [uT] */
  /*  */
  /*  Outputs: */
  /*    q:            orientation quaternion q_NS */
  /*  */
  /*  See also UPDATEPOSITION, UPDATEVELNED, UPDATECOMPASS, UPDATEPRESSURES, */
  /*  UPDATEPRESSURES2, UPDATEPRESSURES3, PROPAGATE, MAGFIELD */
  /*  deduct the centripetal acceleration (usefull for air initialization) */
  acc[0] = acc_S[0] - (gyr_S[1] * v_S_rel[2] - gyr_S[2] * v_S_rel[1]);
  acc[1] = acc_S[1] - (gyr_S[2] * v_S_rel[0] - gyr_S[0] * v_S_rel[2]);
  acc[2] = acc_S[2] - (gyr_S[0] * v_S_rel[1] - gyr_S[1] * v_S_rel[0]);

  /*  correct centripetal acceleration */
  for (i = 0; i < 3; i++) {
    a[i] = -acc[i];
  }

  rot1[0] = a[1] - a[2] * 0.0F;
  rot1[1] = a[2] * 0.0F - a[0];
  rot1[2] = a[0] * 0.0F - a[1] * 0.0F;

  /*  calculating theta1: */
  /* n=zeros(3,1); */
  /* theta1=0; */
  norm1 = (real32_T)sqrt((rot1[0] * rot1[0] + rot1[1] * rot1[1]) + rot1[2] *
    rot1[2]);
  if ((real_T)norm1 > 1.0E-13) {
    /*  zero division protection */
    for (i = 0; i < 3; i++) {
      rot1[i] /= norm1;
    }

    theta1 = (real32_T)asin(norm1 / norm(acc));
  } else {
    for (i = 0; i < 3; i++) {
      rot1[i] = (real32_T)iv0[i];
    }

    theta1 = 0.0F;
  }

  norm1 = 0.0F;
  for (i = 0; i < 3; i++) {
    norm1 += -acc[i] * (real32_T)iv0[i];
  }

  if (norm1 < 0.0F) {
    theta1 = 3.14159274F - theta1;
  }

  norm1 = -(real32_T)sin(theta1 / 2.0F);
  for (i = 0; i < 3; i++) {
    q1[i] = norm1 * rot1[i];
  }

  q1[3] = (real32_T)cos(theta1 / 2.0F);

  /*  q1 quaternion of the leveling  */
  /*  set float type */
  R1[0] = ((q1[0] * q1[0] - q1[1] * q1[1]) - q1[2] * q1[2]) + q1[3] * q1[3];
  R1[3] = q1[0] * q1[1] * 2.0F + q1[2] * q1[3] * 2.0F;
  R1[6] = q1[0] * q1[2] * 2.0F - q1[1] * q1[3] * 2.0F;
  R1[1] = q1[0] * q1[1] * 2.0F - q1[2] * q1[3] * 2.0F;
  R1[4] = ((-q1[0] * q1[0] + q1[1] * q1[1]) - q1[2] * q1[2]) + q1[3] * q1[3];
  R1[7] = q1[0] * q1[3] * 2.0F + q1[1] * q1[2] * 2.0F;
  R1[2] = q1[0] * q1[2] * 2.0F + q1[1] * q1[3] * 2.0F;
  R1[5] = q1[0] * q1[3] * -2.0F + q1[1] * q1[2] * 2.0F;
  R1[8] = ((-q1[0] * q1[0] - q1[1] * q1[1]) + q1[2] * q1[2]) + q1[3] * q1[3];

  /*  DCM for theta and phi (only accelerometers used) */
  for (i = 0; i < 3; i++) {
    acc[i] = 0.0F;
    for (i0 = 0; i0 < 3; i0++) {
      acc[i] += R1[i + 3 * i0] * b_S[i0];
    }
  }

  /*  Leveling of the magnetic flux using the accelerometer measurements */
  rot2[0] = acc[1] * 0.0F - 0.0F * b_N[1];
  rot2[1] = 0.0F * b_N[0] - acc[0] * 0.0F;
  rot2[2] = acc[0] * b_N[1] - acc[1] * b_N[0];

  /*  Rotate the the measurement with respect to the magnetic model(only x and y) and get the Azimuth */
  norm1 = (real32_T)sqrt((rot2[0] * rot2[0] + rot2[1] * rot2[1]) + rot2[2] *
    rot2[2]);
  b_a[0] = acc[0];
  b_a[1] = acc[1];
  b_a[2] = 0.0F;
  rot1[0] = b_N[0];
  rot1[1] = b_N[1];
  rot1[2] = 0.0F;
  theta1 = 0.0F;
  for (i = 0; i < 3; i++) {
    theta1 += b_a[i] * rot1[i];
  }

  /*  get the sign of the rotaion product of the Azimuth (+/-) */
  if ((real_T)norm1 > 1.0E-9) {
    for (i = 0; i < 3; i++) {
      rot2[i] /= norm1;
    }

    b_acc[0] = acc[0];
    b_acc[1] = acc[1];
    b_acc[2] = 0.0F;
    b_b_N[0] = b_N[0];
    b_b_N[1] = b_N[1];
    b_b_N[2] = 0.0F;
    theta2 = (real32_T)asin(norm1 / (norm(b_acc) * norm(b_b_N)));

    /*  Calculation of theta (Azimuth) */
  } else {
    for (i = 0; i < 3; i++) {
      rot2[i] = (real32_T)iv0[i];
    }

    /*  protection zero div */
    theta2 = 0.0F;
  }

  if (theta1 < 0.0F) {
    theta2 = 3.14159274F - theta2;
  }

  norm1 = -(real32_T)sin(theta2 / 2.0F);
  for (i = 0; i < 3; i++) {
    q2[i] = norm1 * rot2[i];
  }

  q2[3] = (real32_T)cos(theta2 / 2.0F);

  /*  q2 quaternion of the Azimuth */
  /*  [ q3, q2, -q1, q0] */
  /*  [ -q2, q3, q0, q1] */
  /*  [ q1, -q0, q3, q2] */
  /*  [ -q0, -q1, -q2, q3] */
  /*  set float type */
  Q[0] = q2[3];
  Q[4] = q2[2];
  Q[8] = -q2[1];
  Q[12] = q2[0];
  Q[1] = -q2[2];
  Q[5] = q2[3];
  Q[9] = q2[0];
  Q[13] = q2[1];
  Q[2] = q2[1];
  Q[6] = -q2[0];
  Q[10] = q2[3];
  Q[14] = q2[2];
  Q[3] = -q2[0];
  Q[7] = -q2[1];
  Q[11] = -q2[2];
  Q[15] = q2[3];
  for (i = 0; i < 4; i++) {
    q2[i] = 0.0F;
    for (i0 = 0; i0 < 4; i0++) {
      q2[i] += Q[i + (i0 << 2)] * q1[i0];
    }
  }

  /*  quaternion multiplication (make matrix from q2) */
  x->q_NS[0] = q2[0];

  /*  x(4): state Q1 */
  x->q_NS[1] = q2[1];

  /*  x(5): state Q2 */
  x->q_NS[2] = q2[2];

  /*  x(6): state Q3 */
  x->q_NS[3] = q2[3];

  /*  x(7): state Q4 */
  /*  Velocity */
  x->v_N[0] = 0.0F;

  /*  x(8): state N velocity */
  x->v_N[1] = 0.0F;

  /*  x(9): state E velocity */
  x->v_N[2] = 0.0F;

  /*  x(10): state D velocity      */
  /*  gyr bias */
  x->b_g[0] = 0.0F;
  x->b_g[1] = 0.0F;
  x->b_g[2] = 0.0F;

  /*  acc bias */
  x->b_a[0] = 0.0F;
  x->b_a[1] = 0.0F;
  x->b_a[2] = 0.0F;

  /*  QFF 1013.25 hPa */
  x->QFF = 101.325F;

  /*  wind */
  x->w[0] = 0.0F;
  x->w[1] = 0.0F;
  x->w[2] = 0.0F;

  /*  K */
  x->K = 0.0F;

  /*  % variances */
  /*  P=single(diag([(metric2GEOD([x.p(1,1),x.p(2,1),x.p(3,1),[25 25 100]])... */
  /*      .*[1e7 1e7 1]).^2,... */
  /*      0.001,0.001,0.001,... */
  /*      0.01,0.01,0.01,... */
  /*      0.001^2*[1 1 1],... */
  /*      configuration.sigma_a_d^2*[1,1,1],... */
  /*      1^2,... */
  /*      100,100,9,... */
  /*      0.001])); */
  /*  variances */
  /* %[50,50,100,... */
  norm1 = rt_powf_snf(configuration.sigma_a_d, 2.0F);
  v[0] = 24581.7227F;
  v[1] = 24581.7227F;
  v[2] = 10000.0F;
  v[3] = 0.001F;
  v[4] = 0.001F;
  v[5] = 0.001F;
  v[6] = 0.01F;
  v[7] = 0.01F;
  v[8] = 0.01F;
  for (i = 0; i < 3; i++) {
    v[i + 9] = 1.0E-6F;
  }

  for (i = 0; i < 3; i++) {
    v[i + 12] = norm1;
  }

  v[15] = 1.0F;
  v[16] = 100.0F;
  v[17] = 100.0F;
  v[18] = 9.0F;
  v[19] = 0.0025F;
  memset(&P[0], 0, 400U * sizeof(real32_T));
  for (i = 0; i < 20; i++) {
    P[i + 20 * i] = v[i];
  }

  /*  0.001])); */
}

/* End of code generation (initStates.c) */
