/*
 * quat2rpy.c
 *
 * Code generation for function 'quat2rpy'
 *
 * C source code generated on: Fri Jul 11 14:42:14 2014
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
#include "HyEst_rtwutil.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void quat2rpy(const real32_T q[4], real32_T rpy[3])
{
  /*  set float type */
  /*  set float type */
  rpy[0] = rt_atan2f_snf(q[0] * q[3] * -2.0F + q[1] * q[2] * 2.0F, ((-q[0] * q[0]
    - q[1] * q[1]) + q[2] * q[2]) + q[3] * q[3]);
  rpy[1] = (real32_T)asin(-(q[0] * q[2] * 2.0F + q[1] * q[3] * 2.0F));
  rpy[2] = rt_atan2f_snf(q[0] * q[1] * 2.0F - q[2] * q[3] * 2.0F, ((q[0] * q[0]
    - q[1] * q[1]) - q[2] * q[2]) + q[3] * q[3]);
}

/* End of code generation (quat2rpy.c) */
