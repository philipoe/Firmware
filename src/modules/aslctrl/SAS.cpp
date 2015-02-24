/*
 * SAS.cpp
 *
 *  Created on: 04.10.2013
 *      Author: philipoe
 */

#include <stdio.h>
#include <math.h>

#include "SAS.h"
#include "helpers/helpfuncs.h"
#include "helpers/consts.h"
#include <systemlib/param/param.h>

SAS::SAS() :
	LP_Pitch(0,0), LP_Roll(0,0), LP_Yaw(0,0), HP_Pitch(0,0), HP_Yaw(0,0), LP_Airspeed(0,0), MA_Airspeed(4,0.0f)
{
};

SAS::SAS(subscriptions *subs_arg) :
		LP_Pitch(0,0), LP_Roll(0,0), LP_Yaw(0,0), HP_Pitch(0,0), HP_Yaw(0,0), LP_Airspeed(0,0), MA_Airspeed(4,0.0f)
{
	subs=subs_arg;
	params=&(subs->aslctrl_params);

	bOvSpdProt_Protecting=false;
};

void SAS::CopyUpdatedParams(void)
{
	// When parameters have been updated, these can be accessed through the appropriate pointer and (if necessary, depending on the param)
	// are written to the objects requiring them.
	LP_Pitch.SetGains(params->SAS_tSample, params->SAS_PitchLowPassOmega);
	LP_Roll.SetGains(params->SAS_tSample, params->SAS_RollLowPassOmega);
	LP_Yaw.SetGains(params->SAS_tSample, params->SAS_RollLowPassOmega); //Set same as roll low-pass dynamics
	HP_Pitch.SetGain(params->SAS_tSample, params->SAS_YawHighPassOmega); //Set same as yaw high-pass dynamics
	HP_Yaw.SetGain(params->SAS_tSample, params->SAS_YawHighPassOmega);
	LP_Airspeed.SetGains(params->SAS_tSample,3.14159f);
}

void SAS::RollDamper(float &uAilCmd, float const &p, float AirspeedScaler1, float AirspeedScaler2)
{
	//LP-filter
	float pFilt = LP_Roll.update(p);

	//SAS action
	uAilCtrlCmd=params->SAS_RollPDir*params->SAS_RollPGain*pFilt;
	uAilCtrlCmd=limit1(uAilCtrlCmd,params->SAS_RCtrlLim);

	//Combination of CAS&SAS to output (with scaling)
	uAilCmd = AirspeedScaler1*uAilCmd + AirspeedScaler2*uAilCtrlCmd + uAilTrim;

	//debug
	if(params->ASLC_DEBUG == 17)  printf("p: %7.5f, pFilt: %7.5f, uAilCtrlCmd: %7.4f, uAilCmd: %.4f\n", (double)p, (double)pFilt, (double)uAilCtrlCmd, (double)uAilCmd);

	return;
}

void SAS::PitchDamper(float &uEleCmd, float const &q, float AirspeedScaler1, float AirspeedScaler2)
{
	//LP filter to smooth outputs to servos, HP-filter to avoid damping in coordinated turns
	float qFilt = LP_Pitch.update(q);
	qFilt = HP_Pitch.update(qFilt);

	//SAS action
	uEleCtrlCmd=params->SAS_PitchPDir*params->SAS_PitchPGain*qFilt;
	uEleCtrlCmd=limit1(uEleCtrlCmd, params->SAS_PCtrlLim);

	//Combination of CAS&SAS to output (with scaling)
	uEleCmd = AirspeedScaler1*uEleCmd + AirspeedScaler2*uEleCtrlCmd + uEleTrim;

	//debug
	if(params->ASLC_DEBUG == 18) printf("q: %7.5f, qFilt: %7.5f, uElevCtrlCmd: %7.4f, uEleCmd:%.4f\n", (double)q, (double)qFilt,(double)uEleCtrlCmd,(double)uEleCmd);

	return;
}

void SAS::YawDamper(float &uRudCmd, float const &r, float AirspeedScaler1, float AirspeedScaler2)
{
	//LP filter to smooth outputs to servos, HP-filter to avoid damping in coordinated turns
	float rFilt = LP_Yaw.update(r);
	rFilt = HP_Yaw.update(rFilt);

	//SAS action
	uRudCtrlCmd = params->SAS_YawPDir * params->SAS_YawPGain*rFilt;
	uRudCtrlCmd=limit1(uRudCtrlCmd, params->SAS_YCtrlLim);

	//Combination of CAS&SAS to output (with scaling)
	uRudCmd = AirspeedScaler1*uRudCmd + AirspeedScaler2*uRudCtrlCmd + uRudTrim;

	//debug
	if(params->ASLC_DEBUG == 19) printf("r: %7.5f, rFilt: %7.5f, HPmgain: %7.4f, uRudCtrlCmd: %7.4f uRudCmd: %.4f\n", (double)r, (double)rFilt,(double)HP_Yaw.m_gain, (double)uRudCtrlCmd, (double)uRudCmd);

	return;
}

void SAS::CoordinatedTurn_YawDamper(float &uRudCmd, float const &r, float& rRef, const float& roll, const float& RollRef, float AirspeedScaler1, float AirspeedScaler2)
{
	//LP filter to smooth outputs to servos, but do not HP filter because we'll compare this to rRef in any case.
	float rFilt = LP_Yaw.update(r);

	if(LP_Airspeed.Get()>1.0f /*bAirspeedValid*/) {
		// Recalculate reference body-yawrate (i.e. r-rate) necessary for a coordinated turn
		rRef = g * sinf(roll) / LP_Airspeed.Get();

		// Yawrate reference. Calculated from coordinated turn constraint, assuming constant pitch (theta_dot=0)
		//ctrldata.Yawdot_ref = g * tan(roll) / LP_Airspeed.Get();
		// Yawrate. Calculated from body velocities
		//ctrldata.Yawdot = sin(roll)/cos(pitch)*subs->att.pitchspeed + cos(roll)/cos(pitch)*subs->att.yawspeed;
	}

	//SAS action
	uRudCtrlCmd = subs->manual_sp.r;
	uRudCtrlCmd += params->SAS_YawPDir * params->SAS_YawCTkP*(rFilt-rRef);
	uRudCtrlCmd -= params->SAS_YawPDir * params->SAS_YawCTFF*sinf(RollRef); 			//real feed forward
	uRudCtrlCmd=limit1(uRudCtrlCmd, params->SAS_YCtrlLim);

	//Combination of CAS&SAS to output (with scaling)
	uRudCmd = AirspeedScaler1*uRudCmd + AirspeedScaler2*uRudCtrlCmd + uRudTrim;

	//debug
	if(params->ASLC_DEBUG == 19) printf("r: %.3f, rFilt: %.3f, rRef: %.3f roll:%.3f rollref: %.3f uRudCtrlCmd: %7.4f uRudCmd: %.4f\n", (double)r, (double)rFilt,(double)rRef,(double)roll, (double)RollRef,(double)uRudCtrlCmd, (double)uRudCmd);

	return;
}

void SAS::CoordinatedTurn_RollYawDecoupling(float const &uAilCmd,float &uRudCmd, float AirspeedScaler)
{
	// This function tries to decouple roll and yaw motions in the sense that when the ailerons are
	// deflected, enough rudder is mixed in in order to eliminate the resulting yaw motion.
	// Note: uAilCmd is already airspeed-gain-scheduled, so we don't need to do that again here.

	uRudCmd += limit1(params->SAS_RollYawDecoupleKari*uAilCmd,params->SAS_YCtrlLim);

	if(params->ASLC_DEBUG == 20)  printf("uRudCmd_New: %7.5f\n", (double)uRudCmd);
}

int SAS::StabilityAugmentation(float &uAilCmd, float &uElevCmd, float &uRudCmd,float &fGainSchedQ,float const &p, float const &q, float const &r, bool bModeChanged)
{
	float temp1=uAilCmd;
	float temp2=uElevCmd;
	float temp3=uRudCmd;

	//Filtering (LowPass & Moving Average)
	if(bModeChanged) {
		LP_Airspeed.Set(subs->airspeed.true_airspeed_m_s);
		MA_Airspeed.Set(subs->airspeed.true_airspeed_m_s);
	}
	MA_Airspeed.update(LP_Airspeed.update(subs->airspeed.true_airspeed_m_s));

	//Airspeed scaling
	fGainSchedQ=GetDynamicPressureScaling(bModeChanged);
	//Trim
	CalculateTrimOutputs();

	//Apply dampers
	RollDamper(uAilCmd, p, 1.0f, fGainSchedQ);
	PitchDamper(uElevCmd, q, 1.0f, fGainSchedQ);
	YawDamper(uRudCmd, r, 1.0f, fGainSchedQ);
	CoordinatedTurn_RollYawDecoupling(uAilCmd, uRudCmd, fGainSchedQ);
	//YawRollDecoupling(uAilCmd,r);

	if(params->ASLC_DEBUG==5) printf("uAil=(%7.4f/%7.4f) uElev=(%7.4f/%7.4f) uRud=(%7.4f/%7.4f)\n", (double)temp1,(double)uAilCmd,(double)temp2,(double)uElevCmd,(double)temp3,(double)uRudCmd);

	return 0;
}

int SAS::RateControl(const float pRef, const float qRef, float& rRef, float &uAilCmd, float &uElevCmd, float &uRudCmd, float &fGainSchedQ, const float &p, const float &q, const float &r, const float& roll, const float& rollRef, bool bModeChanged)
{
	//ATTENTION: Really only controls p&q rates correctly!

	float temp1=uAilCmd;
	float temp2=uElevCmd;
	float temp3=uRudCmd;

	//Filtering (LowPass & Moving Average)
	if(bModeChanged) {
		LP_Airspeed.Set(subs->airspeed.true_airspeed_m_s);
		MA_Airspeed.Set(subs->airspeed.true_airspeed_m_s);
	}
	MA_Airspeed.update(LP_Airspeed.update(subs->airspeed.true_airspeed_m_s));

	//Airspeed scaling
	fGainSchedQ=GetDynamicPressureScaling(bModeChanged);
	//Trim
	CalculateTrimOutputs();

	//Convert reference rates to u commands
	switch(params->ASLC_CtrlType) {
		case PID_GAINSDECOUPLED:
		default:
			uAilCmd = -pRef*params->SAS_RollPDir;
			uElevCmd = -qRef*params->SAS_PitchPDir;
			uRudCmd = -rRef*params->SAS_YawPDir;
			break;
	}

	//Apply dampers
	RollDamper(uAilCmd, p, fGainSchedQ, fGainSchedQ);
	PitchDamper(uElevCmd, q, fGainSchedQ, fGainSchedQ);
	if(params->ASLC_CoordTurn == 1) CoordinatedTurn_YawDamper(uRudCmd, r, rRef, roll, rollRef, fGainSchedQ, fGainSchedQ);
	else YawDamper(uRudCmd, r, 1.0f, fGainSchedQ);

	//Decoupling
	CoordinatedTurn_RollYawDecoupling(uAilCmd, uRudCmd, fGainSchedQ);

	if(params->ASLC_DEBUG==5) printf(" uAil=(%7.4f/%7.4f) uElev=(%7.4f/%7.4f) uRud=(%7.4f/%7.4f)\n",
			(double)temp1,(double)uAilCmd,(double)temp2,(double)uElevCmd,(double)temp3,(double)uRudCmd);

	// Return codes
	if(fabs(params->SAS_RollPGain) <1.0E-5f || fabs(params->SAS_PitchPGain) <1.0E-5f) {
		return -1; //Let the user know that the gains were accidentally set to zero, so there is no control authority!
	}
	else if(bOvSpdProt_Protecting) return -2;
	else return 0;
}

int SAS::CalculateTrimOutputs(void)
{
	// Limit allowed airspeeds
	float Airspeed_Filt = limit2(MA_Airspeed.Get(), params->HL_Vel_vMax,params->HL_Vel_vMin);

	// Bi-linear interpolation when airspeed =[vMin...vMax]. If outside of that range, limit airspeed (see above)
	if(Airspeed_Filt < params->HL_Vel_vNom) {
		float dAirspeed_rel=(Airspeed_Filt-params->HL_Vel_vMin)/(params->HL_Vel_vNom-params->HL_Vel_vMin); //To speed up interpolation
		uAilTrim=interp1_lin(dAirspeed_rel, params->SAS_TrimAilvMin,params->SAS_TrimAilvNom);
		uEleTrim=interp1_lin(dAirspeed_rel, params->SAS_TrimElevMin,params->SAS_TrimElevNom);
		uRudTrim=interp1_lin(dAirspeed_rel, params->SAS_TrimRudvMin,params->SAS_TrimRudvNom);
	}
	else { //bigger or equal vNom
		float dAirspeed_rel=(Airspeed_Filt-params->HL_Vel_vNom)/(params->HL_Vel_vMax-params->HL_Vel_vNom); //To speed up interpolation
		uAilTrim=interp1_lin(dAirspeed_rel, params->SAS_TrimAilvNom,params->SAS_TrimAilvMax);
		uEleTrim=interp1_lin(dAirspeed_rel, params->SAS_TrimElevNom,params->SAS_TrimElevMax);
		uRudTrim=interp1_lin(dAirspeed_rel, params->SAS_TrimRudvNom,params->SAS_TrimRudvMax);
	}

	// Overspeed Protection - for elevator only, and only if v_airspeed > f_tresh1 * vMax & pitch negative.
	// TODO: Improve. While this method works, it a) tends to oscillate after recovery b) only works on TRIM (and should
	// maybe better work on the pitch-ref angle combined with decreasing the effect of q-scaling). Alternatives would be:
	// a) uElev += kP*(vel-vel_max).
	// b) Multiply by pitch angle.
	const float f_tresh1 = 1.15f;
	const float f_tresh2 = 1.5f;
	if(params->ASLC_OvSpdProt > 0)
		//Implements hysteresis
		if(!bOvSpdProt_Protecting && (MA_Airspeed.Get() > f_tresh1 * params->HL_Vel_vMax)) {bOvSpdProt_Protecting=true;}
		if(bOvSpdProt_Protecting && (MA_Airspeed.Get() < f_tresh1 * params->HL_Vel_vMax)) {bOvSpdProt_Protecting=false;}

		//Protect
		if(bOvSpdProt_Protecting) {
		float dAirspeed_rel = ((MA_Airspeed.Get()-f_tresh1*params->HL_Vel_vMax)/((f_tresh2-f_tresh1)*params->HL_Vel_vMax));
		float kP_theta = limit2(-2.0f * subs->att.pitch/params->CAS_PitchAngleLim,1.0f,0.0f) ;
		uEleTrim += kP_theta * interp1_lin(dAirspeed_rel, 0.0f,2.0f*(params->SAS_TrimElevMin-params->SAS_TrimElevMax));

		if(params->ASLC_DEBUG == 13) printf("OVSPD_Prot: v=%.2f, dAirsped_rel=%.2f, kP_theta=%.3f, uEleTrim=%.2f\n",(double)MA_Airspeed.Get(),(double)dAirspeed_rel,(double)kP_theta,(double)uEleTrim);
	}

	// Take into account air density changes
	// This is still TODO .

	if(params->ASLC_DEBUG == 7) printf("Trim Ail=%7.4f Elev=%7.4f Rud=%7.4f\n",(double)uAilTrim,(double)uEleTrim,(double)uRudTrim);

	return 0;
}

float SAS::GetDynamicPressureScaling(bool bModeChanged)
{
	if(params->ASLC_GainSch_Q <= 0) return 1.0f; //Dynamic Pressure Scaling disabled

	//Calculation of airspeed scaling factor
	float MA_Airspeed_lim=limit2(MA_Airspeed.Get(),params->HL_Vel_vMax*params->SAS_vScaleLimF,params->HL_Vel_vMin/params->SAS_vScaleLimF);
	float fAirspeed=pow(params->HL_Vel_vNom/MA_Airspeed_lim, params->SAS_vScaleExp);

	//Calculation of air-density scaling factor
	//Assume constant for now.
	float fRho=1.0f;

	if(params->ASLC_DEBUG==5) printf("v-Scale: v=%.4f v_f=%.4f f_Sc:%.4f vScaleExp:%.2f",(double)subs->airspeed.true_airspeed_m_s, (double)MA_Airspeed_lim, double(fAirspeed*fRho),(double)params->SAS_vScaleExp);

	return fAirspeed*fRho;
}



