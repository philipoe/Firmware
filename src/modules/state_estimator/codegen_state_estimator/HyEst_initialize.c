/*
 * HyEst_initialize.c
 *
 * Code generation for function 'HyEst_initialize'
 *
 * C source code generated on: Wed May 06 13:59:16 2015
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
static uint32_T TechPodAirplane_dirty;

/* Function Declarations */

/* Function Definitions */
void HyEst_initialize(void)
{
  static b_struct_T r0 = { { 73.0F, 71.0F, 82.0F, 70.0F, 50.0F, 48.0F, 49.0F,
      48.0F }, 2010.0F, 13.0F, 0.0F, 0.0F, 2010.0F, 2015.0F, -1000.0F, 600000.0F,
    { -29496.5F, -1585.9F, 4945.1F, -2396.6F, 3026.0F, -2707.7F, 1668.6F,
      -575.4F, 1339.7F, -2326.3F, -160.5F, 1231.7F, 251.7F, 634.2F, -536.8F,
      912.6F, 809.0F, 286.4F, 166.6F, -211.2F, -357.1F, 164.4F, 89.7F, -309.2F,
      -231.1F, 357.2F, 44.7F, 200.3F, 188.9F, -141.2F, -118.1F, -163.1F, 0.1F,
      -7.7F, 100.9F, 72.8F, 68.6F, -20.8F, 76.0F, 44.2F, -141.4F, 61.5F, -22.9F,
      -66.3F, 13.1F, 3.1F, -77.9F, 54.9F, 80.4F, -75.0F, -57.8F, -4.7F, -21.2F,
      45.3F, 6.6F, 14.0F, 24.9F, 10.4F, 7.0F, 1.6F, -27.7F, 4.9F, -3.4F, 24.3F,
      8.2F, 10.9F, -14.5F, -20.0F, -5.7F, 11.9F, -19.3F, -17.4F, 11.6F, 16.7F,
      10.9F, 7.1F, -14.1F, -10.8F, -3.7F, 1.7F, 5.4F, 9.4F, -20.5F, 3.4F, 11.6F,
      -5.3F, 12.8F, 3.1F, -7.2F, -12.4F, -7.4F, -0.8F, 8.0F, 8.4F, 2.2F, -8.4F,
      -6.1F, -10.1F, 7.0F, -2.0F, -6.3F, 2.8F, 0.9F, -0.1F, -1.1F, 4.7F, -0.2F,
      4.4F, 2.5F, -7.2F, -0.3F, -1.0F, 2.2F, -4.0F, 3.1F, -2.0F, -1.0F, -2.0F,
      -2.8F, -8.3F, 3.0F, -1.5F, 0.1F, -2.1F, 1.7F, 1.6F, -0.6F, -0.5F, -1.8F,
      0.5F, 0.9F, -0.8F, -0.4F, 0.4F, -2.5F, 1.8F, -1.3F, 0.2F, -2.1F, 0.8F,
      -1.9F, 3.8F, -1.8F, -2.1F, -0.2F, -0.8F, 0.3F, 0.3F, 1.0F, 2.2F, -0.7F,
      -2.5F, 0.9F, 0.5F, -0.1F, 0.6F, 0.5F, 0.0F, -0.4F, 0.1F, -0.4F, 0.3F, 0.2F,
      -0.9F, -0.8F, -0.2F, 0.0F, 0.8F, -0.2F, -0.9F, -0.8F, 0.3F, 0.3F, 0.4F,
      1.7F, -0.4F, -0.6F, 1.1F, -1.2F, -0.3F, -0.1F, 0.8F, 0.5F, -0.2F, 0.1F,
      0.4F, 0.5F, 0.0F, 0.4F, 0.4F, -0.2F, -0.3F, -0.5F, -0.3F, -0.8F }, 8.0F, {
      11.4F, 16.7F, -28.8F, -11.3F, -3.9F, -23.0F, 2.7F, -12.9F, 1.3F, -3.9F,
      8.6F, -2.9F, -2.9F, -8.1F, -2.1F, -1.4F, 2.0F, 0.4F, -8.9F, 3.2F, 4.4F,
      3.6F, -2.3F, -0.8F, -0.5F, 0.5F, 0.5F, -1.5F, 1.5F, -0.7F, 0.9F, 1.3F,
      3.7F, 1.4F, -0.6F, -0.3F, -0.3F, -0.1F, -0.3F, -2.1F, 1.9F, -0.4F, -1.6F,
      -0.5F, -0.2F, 0.8F, 1.8F, 0.5F, 0.2F, -0.1F, 0.6F, -0.6F, 0.3F, 1.4F,
      -0.2F, 0.3F, -0.1F, 0.1F, -0.8F, -0.8F, -0.3F, 0.4F, 0.2F, -0.1F, 0.1F,
      0.0F, -0.5F, 0.2F, 0.3F, 0.5F, -0.3F, 0.4F, 0.3F, 0.1F, 0.2F, -0.1F, -0.5F,
      0.4F, 0.2F, 0.4F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } };

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

  static airplane_T r5 = { { -0.00228700158F, 0.0971678272F, -1.5978148F }, { -
      0.00037F, -0.00143F, 0.11257F, 0.35908F }, 0.3362F, 1.18F, { 2.5F, 2.5F },
    0.47F, { -0.0102834338F, 6.67184401F, -4.24546337F, 8.66733646F,
      -3.00241661F }, -11.14083F, 8.56425285F, -0.560904F, 0.985854387F,
    6.49339962F };

  igrf11dataCurrent = r0;
  states = r1;
  senseSoarAirplane = r2;
  easyGliderAirplane = r3;
  AtlantikSolarAirplane = r4;
  TechPodAirplane = r5;
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
  TechPodAirplane_dirty = 0U;
  AtlantikSolarAirplane_dirty = 0U;
  easyGliderAirplane_dirty = 0U;
  senseSoarAirplane_dirty = 0U;
  states_dirty = 0U;
  igrf11dataCurrent_dirty = 0U;
  igrf11dataNext_dirty = 0U;
  configuration_dirty = 0U;
}

/* End of code generation (HyEst_initialize.c) */
