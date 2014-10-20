/*
 * CAS_MPC.h : Wrapper-file for an mpc-based aircraft attitude controller
 *
 *  Created on: 04.10.2013
 *      Author: philipoe
 */

#include "helpers/params.h"
#include "helpers/subs.h"
#include "helpers/Filters.h"
#include "helpers/helpfuncs.h"
#include "helpers/consts.h"

//*****************************************************************
//*** CONSTANT PARAMETERS (not changeable via QGroundControl)
//*****************************************************************

//*****************************************************************
//*** CAS_MPC Class
//*****************************************************************
class CAS_MPC
{
public:
	CAS_MPC();
	CAS_MPC(parameters *params_arg, subscriptions *subs_arg);
	~CAS_MPC();
	void init(void);

	void CopyUpdatedParams(void);

	//int update(float &uAilCmd, float &uElevCmd, float &uRudCmd, const float &p, const float &q, const float &r);

public: //TODO: Change this to private, put all function calls to here in the update function
	int CASRollPitchControl_MPC(float& uAil, float& uEle, float const& RollAngleRef, float const& RollAngle, float const& p, float const& PitchAngleRef, float const& PitchAngle, float const&q, bool bModeChanged);

	inline float PitchControl_MPC(float const& PitchAngleRef, float const& PitchAngle, float const& q);
	inline float BankControl_MPC(float const& RollAngleRef, float const& RollAngle, float const& p);

	/*float GetDynamicPressureScaling(bool bModeChanged);*/
	int CalculateTrimOutputs(void);

private:
	parameters *params;	//parameters from PX4
	subscriptions *subs; //UORB subscriptions from PX4

	MovingAverage 	MA_Airspeed; 	// Airspeed Moving Average Filter
	LowPass 		LP_Airspeed;
	LowPass 		LP_AccZ;
	LowPass 		LP_PitchRate;			// PitchRate Low Pass Filter
	LowPass 		LP_RollRate;			// RollRate Low Pass Filter

	float uAilTrim;		// Trim commands
	float uEleTrim;
	float uRudTrim;

	double *xin_roll, *uout_roll;		//Matlab interface pointers
	double *xin_pitch, *uout_pitch;		//Matlab interface pointers
	unsigned long region;
};



