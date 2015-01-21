/*
 * HyEst_data.c
 *
 * Code generation for function 'HyEst_data'
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
#include "HyEst_data.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
struct_T configuration;
b_struct_T igrf11dataNext;
b_struct_T igrf11dataCurrent;
airplane_T senseSoarAirplane;
airplane_T easyGliderAirplane;
airplane_T AtlantikSolarAirplane;

/* Function Declarations */

/* Function Definitions */
/* End of code generation (HyEst_data.c) */
