/*
 * HyEst_initialize.c
 *
 * Code generation for function 'HyEst_initialize'
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
#include "HyEst_initialize.h"
#include "HyEst_data.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
static uint32_T configuration_dirty;
static uint32_T igrf11dataNext_dirty;
static uint32_T igrf11dataCurrent_dirty;
static states_T states;
static uint32_T states_dirty;
static uint32_T senseSoarAirplane_dirty;
static uint32_T easyGliderAirplane_dirty;
static uint32_T AtlantikSolarAirplane_dirty;

/* Function Declarations */

/* Function Definitions */
void HyEst_initialize(void)
{
  static b_struct_T r0 = { { 'I', 'G', 'R', 'F', '2', '0', '1', '0' }, 2010.0,
    13.0, 0.0, 0.0, 2010.0, 2015.0, -1000.0, 600000.0, { -29496.5, -1585.9,
      4945.1, -2396.6, 3026.0, -2707.7, 1668.6, -575.4, 1339.7, -2326.3, -160.5,
      1231.7, 251.7, 634.2, -536.8, 912.6, 809.0, 286.4, 166.6, -211.2, -357.1,
      164.4, 89.7, -309.2, -231.1, 357.2, 44.7, 200.3, 188.9, -141.2, -118.1,
      -163.1, 0.1, -7.7, 100.9, 72.8, 68.6, -20.8, 76.0, 44.2, -141.4, 61.5,
      -22.9, -66.3, 13.1, 3.1, -77.9, 54.9, 80.4, -75.0, -57.8, -4.7, -21.2,
      45.3, 6.6, 14.0, 24.9, 10.4, 7.0, 1.6, -27.7, 4.9, -3.4, 24.3, 8.2, 10.9,
      -14.5, -20.0, -5.7, 11.9, -19.3, -17.4, 11.6, 16.7, 10.9, 7.1, -14.1,
      -10.8, -3.7, 1.7, 5.4, 9.4, -20.5, 3.4, 11.6, -5.3, 12.8, 3.1, -7.2, -12.4,
      -7.4, -0.8, 8.0, 8.4, 2.2, -8.4, -6.1, -10.1, 7.0, -2.0, -6.3, 2.8, 0.9,
      -0.1, -1.1, 4.7, -0.2, 4.4, 2.5, -7.2, -0.3, -1.0, 2.2, -4.0, 3.1, -2.0,
      -1.0, -2.0, -2.8, -8.3, 3.0, -1.5, 0.1, -2.1, 1.7, 1.6, -0.6, -0.5, -1.8,
      0.5, 0.9, -0.8, -0.4, 0.4, -2.5, 1.8, -1.3, 0.2, -2.1, 0.8, -1.9, 3.8,
      -1.8, -2.1, -0.2, -0.8, 0.3, 0.3, 1.0, 2.2, -0.7, -2.5, 0.9, 0.5, -0.1,
      0.6, 0.5, 0.0, -0.4, 0.1, -0.4, 0.3, 0.2, -0.9, -0.8, -0.2, 0.0, 0.8, -0.2,
      -0.9, -0.8, 0.3, 0.3, 0.4, 1.7, -0.4, -0.6, 1.1, -1.2, -0.3, -0.1, 0.8,
      0.5, -0.2, 0.1, 0.4, 0.5, 0.0, 0.4, 0.4, -0.2, -0.3, -0.5, -0.3, -0.8 },
    8.0, { 11.4, 16.7, -28.8, -11.3, -3.9, -23.0, 2.7, -12.9, 1.3, -3.9, 8.6,
      -2.9, -2.9, -8.1, -2.1, -1.4, 2.0, 0.4, -8.9, 3.2, 4.4, 3.6, -2.3, -0.8,
      -0.5, 0.5, 0.5, -1.5, 1.5, -0.7, 0.9, 1.3, 3.7, 1.4, -0.6, -0.3, -0.3,
      -0.1, -0.3, -2.1, 1.9, -0.4, -1.6, -0.5, -0.2, 0.8, 1.8, 0.5, 0.2, -0.1,
      0.6, -0.6, 0.3, 1.4, -0.2, 0.3, -0.1, 0.1, -0.8, -0.8, -0.3, 0.4, 0.2,
      -0.1, 0.1, 0.0, -0.5, 0.2, 0.3, 0.5, -0.3, 0.4, 0.3, 0.1, 0.2, -0.1, -0.5,
      0.4, 0.2, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

  static states_T r1 = { { 0.0, 0.0, 0.0 }, { 0.0F, 0.0F, 0.0F, 0.0F }, { 0.0F,
      0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0.0F, { 0.0F,
      0.0F, 0.0F }, 0.0F };

  static airplane_T r2 = { { -0.00228700158F, 0.0971678272F, -1.5978148F }, { -
      0.000139884389F, -0.000835859217F, 0.0979335308F, 0.452988803F }, 0.3F,
    1.1F, { 5.0F, 5.0F }, 0.76F, { -0.000932878407F, 4.49220514F, -3.44509506F,
      9.7484436F, -4.33450747F }, -17.0974674F, 13.1138945F, -0.766627F,
    1.27805805F, 8.35339165F };

  static airplane_T r3 = { { -0.00228700158F, 0.0971678272F, -1.5978148F }, { -
      0.0003F, -0.0002F, 0.1005F, 0.6034F }, 0.3F, 1.1F, { 1.2F, 1.2F }, 0.316F,
    { -0.026575096F, 9.14059639F, -15.8700018F, 18.037878F, -7.13572407F },
    -10.4918032F, 10.0473595F, -0.126566947F, 1.28868723F, 4.07539797F };

  static airplane_T r4 = { { -0.00228700158F, 0.0971678272F, -1.5978148F }, { -
      0.0003F, 0.0006F, 0.1062F, 0.5732F }, 0.25F, 1.1F, { 6.0F, 6.0F }, 1.708F,
    { -0.0131047918F, 7.23460817F, -13.9426994F, 17.2970886F, -6.62192059F },
    -9.91655064F, 11.2498837F, -0.128382802F, 1.41673839F, 8.69128418F };

  igrf11dataCurrent = r0;
  states = r1;
  senseSoarAirplane = r2;
  easyGliderAirplane = r3;
  AtlantikSolarAirplane = r4;
  igrf11dataNext = igrf11dataCurrent;
  configuration.tau = 3600.0F;
  configuration.tau_w = 6.0F;
  configuration.sigma_a_c = 0.004F;
  configuration.sigma_a_d = 0.02F;
  configuration.sigma_aw_c = 5.5E-5F;
  configuration.sigma_g_c = 0.003F;
  configuration.sigma_gw_c = 2.0E-5F;
  configuration.sigma_pw_c = 0.00107582868F;
  configuration.sigma_w_c = 1.0F;
  configuration.sigma_psmd = 0.010625F;
  configuration.sigma_pdmd = 0.002F;
  configuration.sigma_Td = 1.0F;
  configuration.sigma_md = 5.0F;
  rt_InitInfAndNaN(8U);
  AtlantikSolarAirplane_dirty = 0U;
  easyGliderAirplane_dirty = 0U;
  senseSoarAirplane_dirty = 0U;
  states_dirty = 0U;
  igrf11dataCurrent_dirty = 0U;
  igrf11dataNext_dirty = 0U;
  configuration_dirty = 0U;
}

/* End of code generation (HyEst_initialize.c) */
