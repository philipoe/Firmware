/*
 * CAS.h
 *
 *  Created on: 04.10.2013
 *      Author: philipoe
 */

#include "helpers/subs.h"
#include "helpers/Filters.h"
#include "helpers/helpfuncs.h"
#include "helpers/consts.h"
#include "helpers/PI.h"

//*****************************************************************
//*** CONSTANT PARAMETERS (not changeable via QGroundControl)
//*****************************************************************

//*****************************************************************
//*** CAS Class
//*****************************************************************
class CAS
{
public:
	CAS();
	CAS(subscriptions *subs_arg);

	void UpdateFilters(float &h);
	void CopyUpdatedParams(void);

	//int update(float &uAilCmd, float &uElevCmd, float &uRudCmd, const float &p, const float &q, const float &r);

public: //TODO: Change this to private, put all function calls to here in the update function
	int CASRollPitchControl(float &pref, float &qref, float& rref, float const &RollAngleRef, float const &Roll, float const &PitchAngleRef, float const &Pitch, float const &accZ, aslctrl_data_s* ctrldata, bool bModeChanged);

	float PitchControl(float const& pitchRef, float& pitchRefCT, float const& pitch, float const& rollRef, float const& roll, float& PGain, float& uThrot, const int aslctrl_mode);
	float BankControl(float const &bankRef, float const &roll, float &PGain);
	float SideslipControl(float const& rollRef, float const& roll, float const& pitch, aslctrl_data_s* ctrldata);
	int CoordinatedTurnControl(const float& roll, const float& pitch, float& qRef, float& rRef, aslctrl_data_s* ctrldata);

	//Safety Functions

private:
	subscriptions *subs; //UORB subscriptions from PX4
	aslctrl_parameters_s *params;	// Pointer to aslctrl parameters in the UORB subscriptions

	PI_Ctrl PI_PitchAngle;
	PI_Ctrl PI_PitchTC;		//Pitch Angle turn compensation

	LowPass LP_Airspeed;
	LowPass LP_AccZ;
	LowPass LP_Yaw;
	LowPass LP_vZ;

	StallStatusDescriptor StallStatus;	//Stall Prevention System
};



