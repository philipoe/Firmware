#ifndef HELPFUNCS_H_
#define HELPFUNCS_H_

#include <math.h>

//**********************************************************************
//*** Helper functions
//**********************************************************************
inline float sgn(float value) { return (value>0)-(value<0); }
inline float fabs(float value) { return sgn(value)*value;}	//float version of abs; has to be re-implemented here because nuttx only implements integer-version of abs
inline float min (float a, float b) {return ((a<b) ? a:b);}
inline float max (float a, float b) {return ((a>b) ? a:b);}

inline float limit1(float value, const float abs_max) {
	if(isnan(value)) return 0.0f;
	else if(fabs(value)>abs_max) return sgn(value)*abs_max;
	else return value;
}
inline float limit2(float value, float max, float min) {
	if(isnan(value)) return 0.0f;
	else if(value>max) return max;
	else if(value<min) return min;
	else return value;
}

inline int limit1_ret(float &value, const float abs_max) {
	if(isnan(value)) {
		value=0.0f;
		return -2;
	}
	else if(fabs(value)>abs_max) {
		value = sgn(value)*abs_max;
		return -1;
	}
	return 0;
}
inline int limit2_ret(float &value, const float max, const float min) {
	if(isnan(value)) {
		value=0.0f;
		return -2;
	}
	else if(value>max) {
		value = max;
		return -1;
	}
	else if(value<min) {
		value = min;
		return -1;
	}
	return 0;
}

const float Rair=287.1f; // [J/(kg*K)]
const float KelvinConstant=273.15f;
inline float CalcRhoAir(float pressure, float temperature)
{
	 return pressure*100.0f/Rair/(temperature+KelvinConstant);
}

inline float GainScheduler_linear(float error_cur, float error_max, float Kp_nom, float Kp_min)
//   Inputs:
//   error_cur   : the current value of the error
//   error_max   : the maximum value of the error for which we do
//                 gain_scheduling.
//   K_nom       : the nominal gain (for zero error -> the max gain we tune)
//   K_min       : the minimum gain (applied for error >= error_max)
//
//   Outputs:
//   Ksched      : the scheduled gain
{
	error_cur = fabs(error_cur); 										// absolute value of the error
	error_cur = min(error_cur,error_max); 								// max error value is saturated to error_max

	float Kp_sched = Kp_nom - (error_cur - 0)*(Kp_nom-Kp_min)/(error_max - 0); 	// linear interpolation
	return Kp_sched;
}

inline float interp1_lin(const float &x1, const float &x2, const float &x, const float &y1, const float &y2)
{
	//Standard linear interpolation
	return y1 + (x-x1)/(x2-x1)*(y2-y1);
}


inline float interp1_lin(const float &dx_rel,  const float &y1, const float &y2)
{
	// A bit faster linear interpolation, can be used when multiple datasets which have the same x-data are interpolated
	// after each others (this assumes that the dx_rel factor - which is the same for all these datasets - has been precomputed)
	return y1 + dx_rel*(y2-y1);
}

//Protection against division by zero
inline float SAFE_ZERO_DIV(const float &value) {
	return (fabs(value)>1E-5f?value:sgn(value)*1E-5f);
}

//inline float interp1_quadr(const float &x1, const float &x2, const float &x3, const float &x, const float &y1, const float &y2, const float &y3) {
//	SOURCE: http://isezen.com/2012/01/15/quadratic-interpolation-three-point/ (and tested in matlab)
//	a1=y1/((x1-x2)*(x1-x3));
//	a2=y2/((x2-x1)*(x2-x3));
//	a3=y3/((x3-x1)*(x3-x2));
//
//	y=a1*(x-x2)*(x-x3)+a2*(x-x1)*(x-x3)+a3*(x-x1)*(x-x2);
//	return y;
//}

#endif /*HELPFUNCS_H_*/
