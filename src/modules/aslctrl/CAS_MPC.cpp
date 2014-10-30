/*
 * CAS.cpp
 *
 *  Created on: 04.10.2013
 *      Author: philipoe
 */

#include "CAS_MPC.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <drivers/drv_hrt.h>

//MPC includes
#include "MPC/fw_mpc_roll_v1.c"
#include "MPC/fw_mpc_pitch_v1.c"

//General paradigms
// - Function which sets a reference value imposes the limits on it

//*****************************************************************************************
//*** CONSTRUCTORS / DESTRUCTORS
//*****************************************************************************************

CAS_MPC::CAS_MPC() :
	LP_Airspeed(0,0), LP_AccZ(0,0),LP_RollRate(0,0), LP_PitchRate(0,0), MA_Airspeed(4,0.0f)
{
	init();
};

CAS_MPC::CAS_MPC(parameters *params_arg, subscriptions *subs_arg):
		LP_Airspeed(0,0), LP_AccZ(0,0),LP_RollRate(0,0), LP_PitchRate(0,0), MA_Airspeed(4,0.0f)
{
	init();
	params=params_arg;
	subs=subs_arg;
};
void CAS_MPC::init(void)
{
	xin_roll = (double*) malloc(2*sizeof(double));
	xin_pitch = (double*) malloc(2*sizeof(double));
	uout_roll= (double*) malloc(MPT_RANGE * sizeof(double));
	uout_pitch= (double*) malloc(MPT_RANGE_PITCH * sizeof(double));

	if (xin_roll == NULL || xin_pitch == NULL || uout_roll == NULL || uout_pitch == NULL) {
		printf("ASLCTRL_MPC failed: out of memory\n");
		//mavlink_log_info(mavlink_fd, "ASLCTRL_MPC failed: out of memory")
	}

	for (int i = 0; i<MPT_RANGE_PITCH; i++)
			uout_pitch[i] = 0;
	for (int i = 0; i<MPT_RANGE; i++)
			uout_roll[i] = 0;
	xin_roll[0]=0.0f;xin_roll[1]=0.0f;
	xin_pitch[0]=0.0f;xin_pitch[1]=0.0f;
}

CAS_MPC::~CAS_MPC(void)
{
	free(xin_roll);
	free(xin_pitch);
	free(uout_roll);
	free(uout_pitch);
}

/*void CAS_MPC::init(parameters *params_arg, subscriptions *subs_arg)
{
	params=params_arg;
	subs=subs_arg;
	LP_Airspeed.SetGains(0.0f,0.0f);
	LP_AccZ.SetGains(0.0f,0.0f);
}*/

void CAS_MPC::CopyUpdatedParams(void)
{
	// When parameters have been updated, these can be accessed through the appropriate pointer and (if necessary, depending on the param)
	// are written to the objects requiring them.

	float CAS_tSample=params->p.SAS_tSample*params->p.CAS_fMult;

	LP_Airspeed.SetGains(CAS_tSample,12.57f);
	LP_AccZ.SetGains(CAS_tSample,12.57f);
	LP_PitchRate.SetGains(params->p.SAS_tSample, params->p.SAS_PitchLowPassOmega);
	LP_RollRate.SetGains(params->p.SAS_tSample, params->p.SAS_RollLowPassOmega);
}

//*****************************************************************************************
//*** LOWER-LEVEL CAS CONTROL (BANK ANGLE, PITCH ANGLE)
//*****************************************************************************************

inline float CAS_MPC::PitchControl_MPC(float const& PitchAngleRef, float const& PitchAngle, float const& q)
{
	xin_pitch[0] = PitchAngleRef-PitchAngle;
	xin_pitch[1] = 0-q;
	for (int i = 0; i<MPT_RANGE_PITCH; i++)
		uout_pitch[i] = 0;

	region = fw_mpc_pitch_v1(xin_pitch, uout_pitch);

	return limit1((float)uout_pitch[0], params->p.SAS_PCtrlLim); // THIS IS THE CONTORL ACTION ACTUALLY APPLIED TO THE VEHICLE!
}

inline float CAS_MPC::BankControl_MPC(float const& RollAngleRef, float const& RollAngle, float const& p)
{
	xin_roll[0] = RollAngleRef-RollAngle;
	xin_roll[1] = 0-p;
	for (int i = 0; i<MPT_RANGE; i++)
		uout_roll[i] = 0;

	region = fw_mpc_roll_v1(xin_roll, uout_roll);

	return limit1((float)uout_roll[0], params->p.SAS_RCtrlLim); // THIS IS THE CONTORL ACTION ACTUALLY APPLIED TO THE VEHICLE!
}

//*****************************************************************************************
//*** HIGHER LEVEL CAS CONTROL (ALTITUDE, HEADING)
//*****************************************************************************************
int CAS_MPC::CASRollPitchControl_MPC(float& uAil, float& uEle, float const& RollAngleRef, float const& RollAngle, float const& p, float const& PitchAngleRef, float const& PitchAngle, float const&q, bool bModeChanged)
{
	//Model predictive control (MPC)
	//Only control Roll and Pitch Angles to reference values. Leave rest untouched.

	//If mode was changed, reinitialise applicable filters and variables
	if(bModeChanged) {
		//LP_AccZ.Set(accZ);
		LP_Airspeed.Set(subs->airspeed.true_airspeed_m_s);
		MA_Airspeed.Set(subs->airspeed.true_airspeed_m_s);
	}
	//LP_AccZ.update(accZ);
	MA_Airspeed.update(LP_Airspeed.update(subs->airspeed.true_airspeed_m_s));
	LP_RollRate.update(p);
	LP_PitchRate.update(q);

	// Controllers
	uAil = BankControl_MPC(RollAngleRef,RollAngle,LP_RollRate.Get());

	if(params->p.ASLC_CtrlType==MPC_STD) {
		uEle = PitchControl_MPC(PitchAngleRef,PitchAngle,LP_PitchRate.Get());
	}
	else {
		// TEMP ONLY FIX to allow disabling the PitchControlMPC
		// Use a crappy standard p-controller on angle and p on rate instead
		uEle = -params->p.CAS_PitchPGain*params->p.CAS_q2uPGain*params->p.SAS_PitchPGain* (PitchAngleRef-PitchAngle) + params->p.SAS_PitchPGain*(LP_PitchRate.Get());
	}

	// Dynamic Pressure Scaling
	// TODO

	// Trimming
	CalculateTrimOutputs();
	uAil += uAilTrim;
	uEle += uEleTrim;

	if(params->p.ASLC_DEBUG==8) {
		printf("MPC: Pitch_ref: %7.4f: Pitch %7.4f q: %7.5f u_elev:%7.4f\n",(double)PitchAngleRef,(double)PitchAngle,(double)q,(double)uEle);
		printf("MPC: Bank_ref:  %7.4f: Bank %7.4f  p: %7.5f u_Ail:%7.4f\n",(double)RollAngleRef,(double)RollAngle,(double)p,(double)uAil);
	}

	return 0;
}

int CAS_MPC::CalculateTrimOutputs(void)
{
	// Limit allowed airspeeds
	float Airspeed_Filt = limit2(MA_Airspeed.Get(), params->p.HL_Vel_vMax,params->p.HL_Vel_vMin);

	// Linear interpolation in Airspeed
	if(Airspeed_Filt < params->p.HL_Vel_vNom) {
		float dAirspeed_rel=(Airspeed_Filt-params->p.HL_Vel_vMin)/(params->p.HL_Vel_vNom-params->p.HL_Vel_vMin); //To speed up interpolation
		uAilTrim=interp1_lin(dAirspeed_rel, params->p.SAS_TrimAilvMin,params->p.SAS_TrimAilvNom);
		uEleTrim=interp1_lin(dAirspeed_rel, params->p.SAS_TrimElevMin,params->p.SAS_TrimElevNom);
		uRudTrim=interp1_lin(dAirspeed_rel, params->p.SAS_TrimRudvMin,params->p.SAS_TrimRudvNom);
	}
	else { //bigger or equal vNom
		float dAirspeed_rel=(Airspeed_Filt-params->p.HL_Vel_vNom)/(params->p.HL_Vel_vMax-params->p.HL_Vel_vNom); //To speed up interpolation
		uAilTrim=interp1_lin(dAirspeed_rel, params->p.SAS_TrimAilvNom,params->p.SAS_TrimAilvMax);
		uEleTrim=interp1_lin(dAirspeed_rel, params->p.SAS_TrimElevNom,params->p.SAS_TrimElevMax);
		uRudTrim=interp1_lin(dAirspeed_rel, params->p.SAS_TrimRudvNom,params->p.SAS_TrimRudvMax);
	}

	// Take into account air density changes
	// This is still TODO .

	if(params->p.ASLC_DEBUG == 7) printf("Trim Ail=%7.4f Elev=%7.4f Rud=%7.4f\n",(double)uAilTrim,(double)uEleTrim,(double)uRudTrim);

	return 0;
}

/*float SAS::GetDynamicPressureScaling(bool bModeChanged)
{
	if(params->p.ASLC_GainSch_Q <= 0) return 1.0f; //Dynamic Pressure Scaling disabled

	//Calculation of airspeed scaling factor
	float MA_Airspeed_lim=limit2(MA_Airspeed.Get(),params->p.HL_Vel_vMax*params->p.SAS_vScaleLimF,params->p.HL_Vel_vMin/params->p.SAS_vScaleLimF);
	float fAirspeed=pow(params->p.HL_Vel_vNom/MA_Airspeed_lim, 2.0f);
	//Calculation of air-density scaling factor
	//Assume constant for now.
	float fRho=1.0f;

	if(params->p.ASLC_DEBUG==5) printf("v-Scale: v=%7.4f v_f=%7.4f f_Sc:%7.4f ",subs->sensors.dbaro_velo_ms, MA_Airspeed_lim, fAirspeed*fRho);

	return fAirspeed*fRho;
}*/


