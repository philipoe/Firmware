/*
 * CAS.cpp
 *
 *  Created on: 04.10.2013
 *      Author: philipoe
 */

#include "CAS.h"
//#include "helpers/PI.h"
#include <math.h>
#include <stdio.h>
#include <drivers/drv_hrt.h>

//General paradigms
// - Function which sets a reference value imposes the limits on it

//*****************************************************************************************
//*** CONSTRUCTORS / DESTRUCTORS
//*****************************************************************************************

CAS::CAS() :
	LP_Airspeed(0,0), LP_AccZ(0,0), LP_Yaw(0,0), LP_vZ(0,0), StallStatus(NO_STALL)
{
}

CAS::CAS(subscriptions *subs_arg):
		LP_Airspeed(0,0), LP_AccZ(0,0), LP_Yaw(0,0), LP_vZ(0,0)
{
	subs=subs_arg;
	params=&(subs->aslctrl_params);
}

void CAS::CopyUpdatedParams(void)
{
	// When parameters have been updated, these can be accessed through the appropriate pointer and (if necessary, depending on the param)
	// are written to the objects requiring them.

	float CAS_tSample=params->SAS_tSample * params->CAS_fMult;

	PI_PitchAngle.SetParams(params->CAS_PitchPGain, params->CAS_PitchIGain, params->CAS_PitchRateLim, -params->CAS_PitchRateLim,params->CAS_PitchRateILim,-params->CAS_PitchRateILim, CAS_tSample);
	PI_PitchTC.SetParams(params->CAS_uElevTurnFF, params->CAS_PitchTCkI, params->CAS_PitchAngleLim, -params->CAS_PitchAngleLim,params->CAS_PitchTCILim,-params->CAS_PitchTCILim, CAS_tSample);
	LP_Airspeed.SetGains(CAS_tSample,12.57f);
	LP_AccZ.SetGains(CAS_tSample,12.57f);
	LP_Yaw.SetGains(CAS_tSample,params->CAS_YawLowPassOmega);
	LP_vZ.SetGains(CAS_tSample,12.57f);
}

//*****************************************************************************************
//*** LOWER-LEVEL CAS CONTROL (BANK ANGLE, PITCH ANGLE)
//*****************************************************************************************

float CAS::PitchControl(float const& pitchRef, float& pitchRefCT, float const& pitch, float const& rollRef, float const& roll, float& PGain, float& uThrot, const int aslctrl_mode)
{
	// PI-Controller with integrated elevator compensation for coordinated turns. P-Gain was scheduled before.
	// [1]: A minimalist control strategy for Small UAVs, Severin Leven et al., EPFL

	// Limit the roll angle at some reasonable value
	float roll_lim = limit1(roll, params->CAS_RollAngleLim*1.3f);
	// First order Taylor-series expansion (cp. [1]) of 1/cos(roll) to avoid steep increase/sensitivity to noise for 1/cos(roll) as roll -> 90°
	float inv_cosroll = 0.5*pow(roll_lim,2.0f) + 1.0;

	//Bank angle pitch compensation
	float pitchRef_ff=0.0f;
	if(params->ASLC_CoordTurn == 1) {
		// PitchReference FeedForward & Integrator
		//Compute and apply limit for maximum value of the integrator
		float inv_cosroll_max = 0.5*pow(params->CAS_RollAngleLim,2.0f) + 1.0;
		float cBankMax=limit2((inv_cosroll-1.0f)/SAFE_ZERO_DIV(inv_cosroll_max-1.0f),1.0f,0.0f);
		PI_PitchTC.SetParams(0.0f, params->CAS_PitchTCkI, cBankMax*params->CAS_PitchTCILim,-cBankMax*params->CAS_PitchTCILim);

		//Apply the pitch-angle-ref feedforward
		pitchRef_ff = PI_PitchTC.step(pitchRef-pitch);
		pitchRef_ff += params->CAS_uElevTurnFF*(inv_cosroll-1);

		if(params->ASLC_DEBUG==25) {
			printf("roll: %.2f cBankMax: %.3f m_int:%.2f PRef_org:%.2f PRef:%.2f P:%.2f \n",
				(double)roll, (double)cBankMax,(double)PI_PitchTC.m_int,(double)pitchRef, double(pitchRef+pitchRef_ff),(double)pitch);

		}
	}

	pitchRefCT=pitchRef+pitchRef_ff;

	//PI-Controller
	PI_PitchAngle.SetPGain(PGain);
	float qref = PI_PitchAngle.step(pitchRefCT-pitch); // This one is saturated within the PI controller

	// Throttle feed forward
	if((params->ASLC_CoordTurn==1) && (aslctrl_mode==MODE_CAS) && (fabs(roll) < 85.0f*DEG2RAD)) {
		// Throttle feed forward (necessary due to 1) higher stall speed and 2) higher drag in curve)
		// Note: This is taken from TECS, and is just a "hack"to have the necessary throttle feed-forward - with the
		// same behaviour as in TECS - also in a pure CAS mode.
		// Note2: To make this fully correct, one would need to increase the airspeed_reference in TECS as a function of the bank angle
		// reference, in order to guarantee that we are above the stall speed even in the turn.
		float cosPhi = cos(roll_lim);
		//Note: This heavily changes throttle even for low bank angles, although it is .
		// -> Adapt this to only change e.g. for bank>10°
		// -> Make this dependant of the roll_reference, not the roll, to avoid induced noise from the roll angle.
		float STEdot_dem = params->roll_throttle_compensation * (1.0f / limit2(cosPhi , 1.0f, 0.1f) - 1.0f);
		float _STEdot_max = params->max_climb_rate*g;
		float ff_throttle = STEdot_dem / _STEdot_max * (1.0f - params->throttle_cruise); //TODO Why 1.0f here? Why not THR_MAX?
		uThrot += ff_throttle;

		if(params->ASLC_DEBUG==24) printf("roll: %.2f STEdot:%.2f ff_throttle %.2f uThrot_new:%.2f \n",(double)roll_lim, (double)STEdot_dem,(double)ff_throttle,(double)uThrot);
	}

	// General issue: When banked too hard, pitch ref angle is hard to track because elevator action is
	// influencing roll motion more than pitch (assumption of decoupled long&lat motion breaks down). Here,
	// we need to scale the output ctrl. action by the angle.This might help in general, and might make the
	// FF-term less necesssary.

	qref=limit1(qref,params->CAS_PitchRateLim);
	return qref;
}

float CAS::BankControl(float const &bankRef, float const &roll, float & PGain)
{
	//P-Controller, P-Gain was scheduled before
	float pRef(PGain*(bankRef-roll));
	pRef=limit1(pRef,params->CAS_RollRateLim);
	return pRef;
}

//*****************************************************************************************
//*** HIGHER LEVEL CAS CONTROL (HEADING)
//*****************************************************************************************

int CAS::CASRollPitchControl(float &pref, float &qref, float& rref, float const &RollAngleRef, float const &Roll, float const &PitchAngleRef, float const &Pitch, float const &accZ, aslctrl_data_s *ctrldata, bool bModeChanged)
{
	//Only control Roll and Pitch Angles to reference values. Leave rest untouched.

	//If mode was changed, reinitialise applicable filters and variables
	if(bModeChanged) {
		LP_Airspeed.Set(subs->airspeed.true_airspeed_m_s);
		LP_AccZ.Set(accZ);
	}

	LP_Airspeed.update(subs->airspeed.true_airspeed_m_s);

	// Gain Scheduling w.r.t angle error - if activated
	if(params->ASLC_GainSch_E == 0) {
		ctrldata->P_kP_GainSch_E=params->CAS_PitchPGain;
		ctrldata->R_kP_GainSch_E=params->CAS_RollPGain;
	}
	else if(params->ASLC_GainSch_E == 1) {
		ctrldata->P_kP_GainSch_E = GainScheduler_linear(PitchAngleRef-Pitch, 0.75f*params->CAS_PitchAngleLim,params->CAS_PitchPGain,params->CAS_PitchPGainM);
		ctrldata->R_kP_GainSch_E = GainScheduler_linear(RollAngleRef-Roll, 0.75f*params->CAS_RollAngleLim,params->CAS_RollPGain,params->CAS_RollPGainM);
		if(params->ASLC_DEBUG==3) printf("Pitch: PGain, error (%7.4f, %7.4f). Roll: PGain, error (%7.4f, %7.4f). \n",
				(double)params->CAS_PitchPGain, double(PitchAngleRef-Pitch), (double)params->CAS_RollPGain, double(RollAngleRef-Roll));
	}

	//Controllers
	pref = BankControl(RollAngleRef,Roll, ctrldata->R_kP_GainSch_E);
	qref = PitchControl(PitchAngleRef,ctrldata->PitchAngleRefCT,Pitch,RollAngleRef,Roll,ctrldata->P_kP_GainSch_E,ctrldata->uThrot,ctrldata->aslctrl_mode);
	if(params->ASLC_CoordTurn == 1) {
		//Do coordinated turn control in SAS
	}

	// Stall protection
	//STALL PROTECTION: Normal acceleration penalizer
	if(params->ASLC_StallProt > 0) {
		//Old Stuff. Don't do anything right now.
	}

	if(params->ASLC_DEBUG==8) {
		printf(" Pitch_ref: %7.4f: Pitch%7.4f qref: %7.5f gain:%7.4f\n",(double)PitchAngleRef,(double)Pitch,(double)qref,(double)ctrldata->P_kP_GainSch_E);
		printf(" Roll_ref: %7.4f: Roll%7.4f pref: %7.5f gain:%7.4f\n",(double)RollAngleRef,(double)Roll,(double)pref,(double)ctrldata->R_kP_GainSch_E);
	}
	if(params->ASLC_DEBUG==15) {
		printf("PI-Ctrl: P-Gain:%7.4f I-Gain:%7.4f tS:%7.4f m_int:%7.5f \n",(double)PI_PitchAngle.m_PGain,(double)PI_PitchAngle.m_IGain,(double)PI_PitchAngle.m_tSample,(double)PI_PitchAngle.m_int);
	}

	//printf("[CAS-Ctrl] AccZ: %7.5f AccZBias: %7.5f\n",subs.sensors.accelerometer_m_s2[2], subs.ekf.x_b_a[2]);

	return 0;
}
