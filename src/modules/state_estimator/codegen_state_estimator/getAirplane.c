/*
 * getAirplane.c
 *
 * Code generation for function 'getAirplane'
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
#include "HyEst_data.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void getAirplane(int8_T idx, airplane_T *airplane, boolean_T *exists)
{
  /* GETAIRPLANE Summary of this function goes here */
  /*    Detailed explanation goes here */
  *exists = TRUE;
  switch (idx) {
   case 0:
    *airplane = senseSoarAirplane;
    break;

   case 1:
    *airplane = easyGliderAirplane;
    break;

   case 2:
    *airplane = AtlantikSolarAirplane;
    break;

   default:
    *exists = FALSE;
    *airplane = senseSoarAirplane;
    break;
  }
}

/* End of code generation (getAirplane.c) */
