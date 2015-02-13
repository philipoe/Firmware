/*
 * SAS.h
 *
 *  Created on: 04.10.2013
 *      Author: philipoe
 */

#include "helpers/subs.h"
#include "helpers/Filters.h"

class SAS
{
public:
	SAS();
	SAS(subscriptions *subs_arg);

	//Pure Stability Augmentation
	int StabilityAugmentation(float &uAilCmd, float &uElevCmd, float &uRudCmd, float &fGainSchedQ, const float &p, const float &q, const float &r, bool bModeChanged);
	//Real Rate Controller, based on the gains set in params
	int RateControl(const float pRef, const float qRef, float& rRef, float &uAilCmd, float &uElevCmd, float &uRudCmd, float &fGainSchedQ, const float &p, const float &q, const float &r, const float& roll, const float& rollRef, bool bModeChanged);

	void CopyUpdatedParams();

private:
	void RollDamper(float &uAilCmd, float const &p, float AirspeedScaler1, float AirspeedScaler2);
	void PitchDamper(float &uEleCmd, float const &q, float AirspeedScaler1, float AirspeedScaler2);
	void YawDamper(float &uRudCmd, float const &r, float AirspeedScaler1, float AirspeedScaler2);
	void CoordinatedTurn_YawDamper(float &uRudCmd, float const &r, float& rRef, const float& roll,const float& RollRef, float AirspeedScaler1, float AirspeedScaler2);
	void CoordinatedTurn_RollYawDecoupling(float const &uAilCmd,float &uRudCmd, float AirspeedScaler);

	float GetDynamicPressureScaling(bool bModeChanged);
	int CalculateTrimOutputs(void);

public: //TODO Really just a temporary solution
	LowPass LP_Pitch;			// Pitch Low Pass Filter
	LowPass LP_Roll;			// Roll Low Pass Filter
	LowPass LP_Yaw;				// Yaw Low Pass Filter
	HighPass HP_Pitch;			// Pitch High Pass Filter - necessary to avoid damping in coordinated turns
	HighPass HP_Yaw;			// Yaw High Pass Filter - necessary to avoid damping in coordinated turns
	LowPass LP_Airspeed; 		// Airspeed Low Pass Filter
	MovingAverage MA_Airspeed; 	// Airspeed Moving Average Filter

private:
	subscriptions *subs; 			// UORB subscriptions from PX4
	aslctrl_parameters_s *params;	// Pointer to aslctrl parameters in the UORB subscriptions

	//Debug
	float uAilCtrlCmd; // The actual commands from the autopilot (i.e. not including manual-piloting-commands)
	float uEleCtrlCmd;
	float uRudCtrlCmd;

	float uAilTrim;		// Trim commands
	float uEleTrim;
	float uRudTrim;

	bool bOvSpdProt_Protecting;
};



