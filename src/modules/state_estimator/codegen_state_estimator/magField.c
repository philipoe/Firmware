/*
 * magField.c
 *
 * Code generation for function 'magField'
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
#include "HyEst_data.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static real_T rt_powd_snf(real_T u0, real_T u1);

/* Function Definitions */
static real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T d0;
  real_T d1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d0 = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d0 == 1.0) {
        y = rtNaN;
      } else if (d0 > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

void magField(const real_T p[3], real_T year, real32_T b_N[3])
{
  real_T y;
  real_T factor;
  real_T gha[195];
  int32_T i;
  real_T sl[14];
  real_T cl[14];
  real_T b_p[119];
  real_T q[119];
  real_T X;
  real_T Y;
  real_T Z;
  int32_T n;
  int32_T m;
  real_T elevKM;
  real_T slat_gd;
  real_T clat_gd;
  real_T aa;
  real_T bb;
  real_T cc;
  real_T r;
  real_T cd;
  real_T sd;
  real_T slat;
  int32_T fn;
  int32_T k;
  int32_T ii;
  real_T b_X[3];

  /*  */
  /*  b_N = MAGFIELDEML(p,year) */
  /*  */
  /*  State propagation with indirect measurements */
  /*  */
  /*  Author: lestefan */
  /*  date:  02/2013 */
  /*  */
  /*  Inputs: */
  /*    p:            position p(lat,lon,h) [rad,rad,m] */
  /*    year:         decimal year */
  /*  */
  /*  Outputs: */
  /*    b_N:          (modeled) magnetic flux density in nav frame N [uT] */
  /*  */
  /*  See also UPDATEPOSITION, UPDATEVELNED, UPDATECOMPASS, UPDATEPRESSURES, */
  /*  UPDATEPRESSURES2, UPDATEPRESSURES3, PROPAGATE, GETINITIALQ */
  y = p[0] * 180.0 / 3.1415926535897931;

  /*  IGRF11MAGM Use 11th generation of International Geomagnetic Reference Field */
  /*   [XYZ, H, DEC, DIP, F, DXDYDZ, DH, DDEC, DDIP, DF] =  */
  /*                                      IGRF11MAGM( HEIGHT, LAT, LON, DYEAR ) */
  /*   calculates the Earth's magnetic field and the secular variation at a specific  */
  /*   location and time using the 11th generation of International Geomagnetic  */
  /*   Reference Field (IGRF-11).  */
  /*  */
  /*   Inputs required by IGRF-11 are: */
  /*    HEIGHT :a scalar value in meters.  */
  /*    LAT    :a scalar geodetic latitude in degrees where north latitude is  */
  /*           positive, and south latitude is negative. */
  /*    LON    :a scalar geodetic longitude in degrees where east longitude  */
  /*           is positive, west is negative. */
  /*    DYEAR  :a scalar decimal year.  Decimal year is the desired year in  */
  /*           a decimal format to include any fraction of the year that has  */
  /*           already passed. */
  /*  */
  /*   Output calculated for the Earth's magnetic field and secular variation include: */
  /*    XYZ    :magnetic field vector in nanotesla (nT). Z is vertical component (+ve down) */
  /*   These are currently not provided: */
  /*    H      :horizontal intensity in nanotesla (nT). */
  /*    DEC    :declination in degrees. (+ve east) */
  /*    DIP    :inclination in degrees. (+ve down) */
  /*    F      :total intensity in nanotesla (nT). */
  /*    DXDYDZ :secular variation in magnetic field vector in nT/year. Z is */
  /*           vertical component (+ve down)  */
  /*    DH     :secular variation in horizontal intensity in nT/year. */
  /*    DDEC   :secular variation in declination in minutes/year. (+ve east) */
  /*    DDIP   :secular variation in inclination in minutes/year. (+ve down) */
  /*    DF     :secular variation in total intensity in nT/year. */
  /*  */
  /*    Limitations: */
  /*  */
  /*    This function is valid between the heights of -1000 meters to 600000 */
  /*    meters.  */
  /*  */
  /*    This function is valid between the years of 1900 and 2015. */
  /*  */
  /*    This function has the limitations of the International Geomagnetic */
  /*    Reference Field (IGRF). For more information see the IGRF web site, */
  /*    http://www.ngdc.noaa.gov/IAGA/vmod/igrfhw.html.    */
  /*  */
  /*    Example: */
  /*  */
  /*    Calculate the magnetic model 1000 meters over Natick, Massachusetts on  */
  /*    July 4, 2005 using IGRF-11: */
  /*       [XYZ, H, DEC, DIP, F] = igrf11magm(1000, 42.283, -71.35, decyear(2005,7,4)) */
  /*  */
  /*    See also DECYEAR, WRLDMAGM. */
  /*     */
  /*    Interface taken from Aerospace Blockset Version, Mathworks Inc. */
  /*  */
  /*    Reference: */
  /*    [1] The IGRF-11 can be found on the web at  */
  /*        http://www.ngdc.noaa.gov/IAGA/vmod/igrf.html */
  /*    [2] Blakely, R. J., "Potential Theory in Gravity & Magnetic Applications",  */
  /*        Cambridge University Press, Cambridge UK, 1996.  */
  /*  Interpolate/Extrapolate coefficients */
  /* ========================================================================== */
  /*  Interpolate/Extrapolate coefficients if necessary */
  if (igrf11dataNext.minYear - igrf11dataCurrent.minYear == 0.0) {
    /*  Extrapolation */
    factor = year - igrf11dataCurrent.minYear;

    /*  Two orders are equal */
    for (i = 0; i < 195; i++) {
      gha[i] = igrf11dataCurrent.gh[i] + factor * igrf11dataCurrent.sv[i];
    }
  } else {
    /*  Interpolation */
    factor = (year - igrf11dataCurrent.minYear) / (igrf11dataNext.minYear -
      igrf11dataCurrent.minYear);

    /*  Two orders must be equal */
    for (i = 0; i < 195; i++) {
      gha[i] = igrf11dataCurrent.gh[i] + factor * (igrf11dataNext.gh[i] -
        igrf11dataCurrent.gh[i]);
    }
  }

  /*  Calculates field components from spherical harmonic  */
  /*  WGS84  */
  /*  preallocate vectors for calculations */
  for (i = 0; i < 14; i++) {
    sl[i] = 0.0;
    cl[i] = 0.0;
  }

  for (i = 0; i < 119; i++) {
    b_p[i] = 0.0;
    q[i] = 0.0;
  }

  /*  initialize outputs */
  X = 0.0;
  Y = 0.0;
  Z = 0.0;

  /*  initialize counters */
  i = 1;
  n = 0;
  m = 1;

  /*  convert to kilometers from meters */
  elevKM = p[2] * 0.001;
  slat_gd = sin(y / 180.0 * 3.1415926535897931);
  if (90.0 - y < 0.001) {
    /*   300 ft. from North pole */
    factor = 89.999;
  } else if (90.0 + y < 0.001) {
    /*   300 ft. from South pole */
    factor = -89.999;
  } else {
    factor = y;
  }

  clat_gd = cos(factor / 180.0 * 3.1415926535897931);
  factor = p[1] * 180.0 / 3.1415926535897931 / 180.0 * 3.1415926535897931;
  sl[0] = sin(factor);
  cl[0] = cos(factor);

  /*  convert from geodetic to geocentric coordinates */
  aa = 4.068063159E+7 * clat_gd * clat_gd;
  bb = 4.040829998E+7 * slat_gd * slat_gd;
  cc = aa + bb;
  factor = sqrt(cc);
  r = sqrt(elevKM * (elevKM + 2.0 * factor) + (4.068063159E+7 * aa +
            4.040829998E+7 * bb) / cc);
  cd = (elevKM + factor) / r;
  sd = 272331.61000000685 / factor * slat_gd * clat_gd / r;
  slat = slat_gd * cd - clat_gd * sd;
  clat_gd = clat_gd * cd + slat_gd * sd;
  factor = 6371.2 / r;
  b_p[0] = 2.0 * slat;
  b_p[1] = 2.0 * clat_gd;
  b_p[2] = 4.5 * slat * slat - 1.5;
  b_p[3] = 5.196152422706632 * clat_gd * slat;
  q[0] = -clat_gd;
  q[1] = slat;
  q[2] = -3.0 * clat_gd * slat;
  q[3] = 1.7320508075688772 * (slat * slat - clat_gd * clat_gd);
  fn = 0;
  r = 0.0;
  for (k = 0; k < 104; k++) {
    if (n < m) {
      m = 0;
      n++;
      r = rt_powd_snf(factor, (real_T)n + 2.0);
      fn = n;
    }

    if (1 + k >= 5) {
      if (m == n) {
        aa = sqrt(1.0 - 0.5 / (real_T)m);
        elevKM = ((real_T)(k - n) + 1.0) - 1.0;
        b_p[k] = (1.0 + 1.0 / (real_T)m) * aa * clat_gd * b_p[(int32_T)elevKM -
          1];
        q[k] = aa * (clat_gd * q[(int32_T)elevKM - 1] + slat / (real_T)m * b_p
                     [(int32_T)elevKM - 1]);
        sl[m - 1] = sl[m - 2] * cl[0] + cl[m - 2] * sl[0];
        cl[m - 1] = cl[m - 2] * cl[0] - sl[m - 2] * sl[0];
      } else {
        aa = sqrt((real_T)(fn * fn - m * m));
        bb = sqrt((real_T)((fn - 1) * (fn - 1) - m * m)) / aa;
        cc = (2.0 * (real_T)fn - 1.0) / aa;
        ii = (k - n) + 1;
        elevKM = ((1.0 + (real_T)k) - 2.0 * (real_T)n) + 1.0;
        b_p[k] = ((real_T)fn + 1.0) * (cc * slat / (real_T)fn * b_p[ii - 1] - bb
          / ((real_T)fn - 1.0) * b_p[(int32_T)elevKM - 1]);
        q[k] = cc * (slat * q[ii - 1] - clat_gd / (real_T)fn * b_p[ii - 1]) - bb
          * q[(int32_T)elevKM - 1];
      }
    }

    aa = r * gha[i - 1];
    if (m == 0) {
      X += aa * q[k];
      Z -= aa * b_p[k];
      i++;
    } else {
      bb = r * gha[i];
      cc = aa * cl[m - 1] + bb * sl[m - 1];
      X += cc * q[k];
      Z -= cc * b_p[k];
      if (clat_gd > 0.0) {
        Y += (aa * sl[m - 1] - bb * cl[m - 1]) * (real_T)m * b_p[k] / (((real_T)
          fn + 1.0) * clat_gd);
      } else {
        Y += (aa * sl[m - 1] - bb * cl[m - 1]) * q[k] * slat;
      }

      i += 2;
    }

    m++;
  }

  factor = X * cd + Z * sd;

  /* [X,Y,Z,XTEMP,YTEMP,ZTEMP] = locCalcFieldEml(lat, lon, height, nMax, gha, ghb); */
  /*  Calculate rest of Magnetic Field and Secular Variation */
  /* [H,F,DDEC,DDIP,DEC,DIP,DH,DX,DY,DZ,DF]=locCalcMagFieldAndSVEml(X,Y,Z,XTEMP,YTEMP,ZTEMP); */
  /*  Make corrections for geographic and magnetic poles */
  /* locCorrectAtPoles() */
  /* if (H < 100.0) */
  /*  at magnetic poles */
  /* DEC = NaN; */
  /* DDEC = NaN; */
  /* end */
  if (90.0 - fabs(y) <= 0.001) {
    /*  at geographic poles */
    factor = rtNaN;
    Y = rtNaN;

    /* DEC = NaN; */
    /* DX  = NaN; */
    /* DY  = NaN; */
    /* DDEC = NaN; */
  }

  /*  Vectorize Northward, Eastward and Downward components */
  /* DXDYDZ = [DX DY DZ]; */
  b_X[0] = factor;
  b_X[1] = Y;
  b_X[2] = Z * cd - X * sd;
  for (i = 0; i < 3; i++) {
    b_N[i] = (real32_T)(b_X[i] / 1000.0);
  }
}

/* End of code generation (magField.c) */
