/*
 * getAirplane.c
 *
 * Code generation for function 'getAirplane'
 *
 * C source code generated on: Wed May 06 13:59:17 2015
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

   case 3:
    *airplane = TechPodAirplane;
    break;

   default:
    *exists = FALSE;
    *airplane = senseSoarAirplane;
    break;
  }
}

/* End of code generation (getAirplane.c) */
