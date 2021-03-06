/*
 * autogen_TasUpdate_withAerodynamics.c
 *
 * Code generation for function 'autogen_TasUpdate_withAerodynamics'
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
#include "autogen_TasUpdate_withAerodynamics.h"
#include "HyEst_rtwutil.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void d_autogen_TasUpdate_withAerodyn(const real32_T in1[9], const real32_T in2[7],
  const real32_T in3[4], real32_T m, real32_T A, real32_T e[4], real32_T E_z[28],
  real32_T E_x_dash[36])
{
  real32_T t2;
  real32_T t3;
  real32_T t4;
  real32_T t5;
  real32_T t7;
  real32_T t13;
  real32_T t14;
  real32_T t15;
  real32_T t17;
  real32_T t18;
  real32_T t19;
  real32_T t20;
  real32_T t21;
  real32_T t23;
  real32_T x[28];
  real32_T b_x[36];

  /* AUTOGEN_TASUPDATE_WITHAERODYNAMICS */
  /*     [E,E_Z,E_X_DASH] = AUTOGEN_TASUPDATE_WITHAERODYNAMICS(IN1,IN2,IN3,M,A) */
  /*     This function was generated by the Symbolic Math Toolbox version 5.9. */
  /*     06-Jul-2014 12:18:30 */
  t2 = rt_powf_snf(in1[3], 2.0F);
  t3 = 1.0F / in2[5];
  t4 = rt_powf_snf(in1[1], 2.0F);
  t5 = t2 + t4;
  t7 = 1.0F / rt_powf_snf(in1[1], 3.0F);
  t13 = ((in3[0] * t2 * in1[3] * 5.832E+6F + 3.14159274F * in3[3] * t4 *
          9.86960411F * in1[1]) + 3.14159274F * in3[1] * t2 * in1[1] * 32400.0F)
    + in3[2] * t4 * 9.86960411F * in1[3] * 180.0F;
  t14 = (real32_T)sqrt(t5);
  t15 = 1.0F / rt_powf_snf(in2[5], 2.0F);
  t17 = (t2 + t4) + rt_powf_snf(in1[2], 2.0F);
  e[0] = -m * (in2[2] - in1[6]) - A * in2[3] * t3 * t5 * (((in3[3] +
    0.101321183F * in3[1] * t2 / rt_powf_snf(in1[1], 2.0F) * 32400.0F) + in3[2] *
    in1[3] * 180.0F / (3.14159274F * in1[1])) + in3[0] * t2 * 0.0322515331F * t7
    * in1[3] * 5.832E+6F) * 1.7418083F;
  e[1] = m * (in2[1] - in1[5]) + A * in1[8] * in2[3] * t3 * t14 * in1[2] *
    1.7418083F;
  e[2] = in2[4] * 1000.0F - in2[3] * t3 * t17 * 1.7418083F;
  e[3] = in2[3] * 10.0F - in1[7] / rt_powf_snf(t3 * (in2[5] + in1[0] * 0.00649F),
    5.26388407F) * 10.0F;
  t18 = rt_powf_snf(t2, 2.0F);
  t19 = rt_powf_snf(t4, 2.0F);
  t20 = 1.0F / (real32_T)sqrt(t5);
  t21 = in1[0] * t3 * 0.00649F;
  t23 = 1.0F / rt_powf_snf(t21 + 1.0F, 6.26388407F);
  x[0] = 0.0F;
  x[1] = 0.0F;
  x[2] = 0.0F;
  x[3] = 0.0F;
  x[4] = 0.0F;
  x[5] = m;
  x[6] = 0.0F;
  x[7] = 0.0F;
  x[8] = -m;
  x[9] = 0.0F;
  x[10] = 0.0F;
  x[11] = 0.0F;
  x[12] = A * t3 * t5 * 0.0322515331F * t7 * t13 * -1.7418083F;
  x[13] = A * in1[8] * t3 * t14 * in1[2] * 1.7418083F;
  x[14] = t3 * t17 * -1.7418083F;
  x[15] = 10.0F;
  x[16] = 0.0F;
  x[17] = 0.0F;
  x[18] = 1000.0F;
  x[19] = 0.0F;
  x[20] = A * in2[3] * t5 * 0.0322515331F * t7 * t13 * t15 * 1.7418083F;
  x[21] = A * in1[8] * in2[3] * t14 * t15 * in1[2] * -1.7418083F;
  x[22] = in2[3] * t15 * t17 * 1.7418083F;
  x[23] = in1[7] * in1[0] * t15 * t23 * -0.341626078F;
  x[24] = 0.0F;
  x[25] = 0.0F;
  x[26] = 0.0F;
  x[27] = 0.0F;
  memcpy(&E_z[0], &x[0], 28U * sizeof(real32_T));
  b_x[0] = 0.0F;
  b_x[1] = 0.0F;
  b_x[2] = 0.0F;
  b_x[3] = in1[7] * t3 * t23 * 0.341626078F;
  b_x[4] = A * in2[3] * t3 * 0.0322515331F / rt_powf_snf(in1[1], 4.0F) *
    (((((in3[0] * t18 * in1[3] * 8.748E+6F + in3[0] * t2 * t4 * in1[3] *
         2.916E+6F) - in3[2] * 9.86960411F * t19 * in1[3] * 90.0F) + 3.14159274F
       * in3[1] * t18 * in1[1] * 32400.0F) - 3.14159274F * in3[3] * 9.86960411F *
      t19 * in1[1]) + in3[2] * t2 * t4 * 9.86960411F * in1[3] * 90.0F) *
    3.48361659F;
  b_x[5] = A * in1[8] * in2[3] * t3 * t20 * in1[1] * in1[2] * 1.7418083F;
  b_x[6] = in2[3] * t3 * in1[1] * -3.48361659F;
  b_x[7] = 0.0F;
  b_x[8] = 0.0F;
  b_x[9] = A * in1[8] * in2[3] * t3 * t14 * 1.7418083F;
  b_x[10] = in2[3] * t3 * in1[2] * -3.48361659F;
  b_x[11] = 0.0F;
  b_x[12] = A * in2[3] * t3 * 0.0322515331F * t7 * ((((((in3[0] * t18 *
    1.458E+7F + in3[0] * t2 * t4 * 8.748E+6F) + in3[2] * 9.86960411F * t19 *
    90.0F) + in3[2] * t2 * t4 * 9.86960411F * 270.0F) + 3.14159274F * in3[1] *
    t2 * in1[1] * in1[3] * 64800.0F) + 3.14159274F * in3[1] * t4 * in1[1] * in1
    [3] * 32400.0F) + 3.14159274F * in3[3] * t4 * 9.86960411F * in1[1] * in1[3])
    * -3.48361659F;
  b_x[13] = A * in1[8] * in2[3] * t3 * t20 * in1[2] * in1[3] * 1.7418083F;
  b_x[14] = in2[3] * t3 * in1[3] * -3.48361659F;
  b_x[15] = 0.0F;
  b_x[16] = 0.0F;
  b_x[17] = 0.0F;
  b_x[18] = 0.0F;
  b_x[19] = 0.0F;
  b_x[20] = 0.0F;
  b_x[21] = -m;
  b_x[22] = 0.0F;
  b_x[23] = 0.0F;
  b_x[24] = m;
  b_x[25] = 0.0F;
  b_x[26] = 0.0F;
  b_x[27] = 0.0F;
  b_x[28] = 0.0F;
  b_x[29] = 0.0F;
  b_x[30] = 0.0F;
  b_x[31] = 1.0F / rt_powf_snf(t21 + 1.0F, 5.26388407F) * -10.0F;
  b_x[32] = 0.0F;
  b_x[33] = A * in2[3] * t3 * t14 * in1[2] * 1.7418083F;
  b_x[34] = 0.0F;
  b_x[35] = 0.0F;
  memcpy(&E_x_dash[0], &b_x[0], 36U * sizeof(real32_T));
}

/* End of code generation (autogen_TasUpdate_withAerodynamics.c) */
