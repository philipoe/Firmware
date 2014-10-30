/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "params.h"
#include "consts.h"

//**********************************************************************
//*** Constructors / Destructors
//**********************************************************************
parameters::parameters():
	bInitialized(false)
{
	init();
}
parameters::~parameters()
{

}

//**********************************************************************
//*** Functions
//**********************************************************************
int parameters::init()
{
	//ASLCTRL-GENERAL
	h.ASLC_DEBUG=param_find("ASLC_DEBUG");
	h.ASLC_CtrlType=param_find("ASLC_CtrlType");
	h.ASLC_GainSch_E=param_find("ASLC_GainSch_E");
	h.ASLC_GainSch_Q=param_find("ASLC_GainSch_Q");
	h.ASLC_StallProt=param_find("ASLC_StallProt");
	h.ASLC_VelCtrl=param_find("ASLC_VelCtrl");
	h.ASLC_OnRCLoss=param_find("ASLC_OnRCLoss");
	h.ASLC_OvSpdProt=param_find("ASLC_OvSpdProt");
	h.ASLC_CoordTurn=param_find("ASLC_CoordTurn");

	//DEMIX
	h.DEMIX_Enabled=param_find("DEMIX_Enabled");
	h.DEMIX_F_CH2t4=param_find("DEMIX_F_CH2t4");
	h.DEMIX_F_CH6t1=param_find("DEMIX_F_CH6t1");
	h.DEMIX_CH2t4=param_find("DEMIX_CH2t4");
	h.DEMIX_CH6t1=param_find("DEMIX_CH6t1");

	//SAS
	h.SAS_tSample=param_find("SAS_tSample");
	h.SAS_RollPGain=param_find("SAS_RollPGain");
	h.SAS_PitchPGain=param_find("SAS_PitchPGain");
	h.SAS_YawPGain=param_find("SAS_YawPGain");
	h.SAS_RollPDir=param_find("SAS_RollPDir");
	h.SAS_PitchPDir=param_find("SAS_PitchPDir");
	h.SAS_YawPDir=param_find("SAS_YawPDir");
	h.SAS_RollYawDecoupleKari=param_find("SAS_RYDecK");
	h.SAS_YawCTkP=param_find("SAS_YawCTkP");
	h.SAS_YawCTFF=param_find("SAS_YawCTFF");
	h.SAS_RCtrlLim=param_find("SAS_RCtrlLim");
	h.SAS_PCtrlLim=param_find("SAS_PCtrlLim");
	h.SAS_YCtrlLim=param_find("SAS_YCtrlLim");
	h.SAS_vScaleLimF=param_find("SAS_vScaleLimF");
	h.SAS_vScaleExp=param_find("SAS_vScaleExp");
	h.SAS_YawHighPassOmega=param_find("SAS_YawHPw");
	h.SAS_PitchLowPassOmega=param_find("SAS_PitchLPw");
	h.SAS_RollLowPassOmega=param_find("SAS_RollLPw");
	h.SAS_TrimAilvNom=param_find("SAS_TrimAilvNom");
	h.SAS_TrimAilvMin=param_find("SAS_TrimAilvMin");
	h.SAS_TrimAilvMax=param_find("SAS_TrimAilvMax");
	h.SAS_TrimElevNom=param_find("SAS_TrimElevNom");
	h.SAS_TrimElevMin=param_find("SAS_TrimElevMin");
	h.SAS_TrimElevMax=param_find("SAS_TrimElevMax");
	h.SAS_TrimRudvNom=param_find("SAS_TrimRudvNom");
	h.SAS_TrimRudvMin=param_find("SAS_TrimRudvMin");
	h.SAS_TrimRudvMax=param_find("SAS_TrimRudvMax");

	//CAS
	h.CAS_fMult=param_find("CAS_fMult");
	h.CAS_PitchPGain=param_find("CAS_PitchPGain");
	h.CAS_PitchPGainM=param_find("CAS_PitchPGainM");
	h.CAS_PitchIGain=param_find("CAS_PitchIGain");
	h.CAS_RollPGain=param_find("CAS_RollPGain");
	h.CAS_RollPGainM=param_find("CAS_RollPGainM");
	h.CAS_HeadPGain=param_find("CAS_HeadPGain");
	h.CAS_q2uPGain=param_find("CAS_q2uPGain");
	h.CAS_p2uPGain=param_find("CAS_p2uPGain");
	h.CAS_PitchRateLim=param_find("CAS_PRateLim");
	h.CAS_PitchRateILim=param_find("CAS_PRateILim");
	h.CAS_PitchTCkI=param_find("CAS_PitchTCkI");
	h.CAS_PitchTCILim=param_find("CAS_PitchTCILim");
	h.CAS_RollRateLim=param_find("CAS_RRateLim");
	h.CAS_YawRateLim=param_find("CAS_YRateLim");
	h.CAS_PitchAngleLim=param_find("CAS_PAngleLim");
	h.CAS_RollAngleLim=param_find("CAS_RAngleLim");
	h.CAS_YawLowPassOmega=param_find("CAS_YawLPw");
	h.CAS_uElevTurnFF= param_find("CAS_uElevTurnFF");

	//HL
	h.HL_fMult=param_find("HL_fMult");
	h.HL_WPL1_Damping=param_find("HL_WPL1_Damping");
	h.HL_WPL1_P_vMin=param_find("HL_WPL1_P_vMin");
	h.HL_WPL1_P_vNom=param_find("HL_WPL1_P_vNom");
	h.HL_WPL1_P_vMax=param_find("HL_WPL1_P_vMax");
	h.HL_Vel_vNom=param_find("HL_Vel_vNom");
	h.HL_Vel_vMin=param_find("HL_Vel_vMin");
	h.HL_Vel_vMax=param_find("HL_Vel_vMax");
	//Old altitude controller
	h.HL_AltLowPassOmega=param_find("HL_AltLPw");
	h.HL_AlthMax=param_find("HL_AlthMax");
	h.HL_AlthMin=param_find("HL_AlthMin");
	h.HL_vZClimb= param_find("HL_vZClimb");
	h.HL_vZSink= param_find("HL_vZSink");

	//TECS
	h.airspeed_min = param_find("HL_Vel_vMin");
	h.airspeed_trim = param_find("HL_Vel_vNom");
	h.airspeed_max = param_find("HL_Vel_vMax");
	h.pitch_limit_min = param_find("FW_P_LIM_MIN");
	h.pitch_limit_max = param_find("FW_P_LIM_MAX");
	h.throttle_min = param_find("FW_THR_MIN");
	h.throttle_max = param_find("FW_THR_MAX");
	h.throttle_cruise = param_find("FW_THR_CRUISE");
	h.time_const = 	param_find("FW_T_TIME_CONST");
	h.time_const_throt = param_find("FW_T_TC_THROT");
	h.min_sink_rate = param_find("FW_T_SINK_MIN");
	h.max_sink_rate = param_find("FW_T_SINK_MAX");
	h.max_climb_rate = param_find("FW_T_CLMB_MAX");
	h.throttle_damp = param_find("FW_T_THR_DAMP");
	h.integrator_gain =	param_find("FW_T_INTEG_GAIN");
	h.vertical_accel_limit = param_find("FW_T_VERT_ACC");
	h.height_comp_filter_omega = param_find("FW_T_HGT_OMEGA");
	h.speed_comp_filter_omega =	param_find("FW_T_SPD_OMEGA");
	h.roll_throttle_compensation = param_find("FW_T_RLL2THR");
	h.speed_weight = param_find("FW_T_SPDWEIGHT");
	h.pitch_damping = param_find("FW_T_PTCH_DAMP");
	h.heightrate_p = param_find("FW_T_HRATE_P");
	h.heightrate_ff = param_find("FW_T_HRATE_FF");
	h.speedrate_p =	param_find("FW_T_SRATE_P");
	h.throttle_slewrate = param_find("FW_T_THRSLEW");
	h.throttle_ILim = param_find("FW_T_ThrILim");

	//General
	bInitialized=true;
	bSanityChecksEnabled=true;

	return 0;
}

int parameters::update()
{
	if(!bInitialized) { init(); }

	if(bSanityChecksEnabled)
	{
		if(Sanitize(h.SAS_RollPGain,0.0f, 10.0f) != 0) return -1;
		if(Sanitize(h.SAS_PitchPGain,0.0f, 10.0f) != 0) return -1;
		if(Sanitize(h.SAS_YawPGain,0.0f, 10.0f) != 0) return -1;
		if(Sanitize(h.SAS_TrimAilvNom, -0.3f, 0.3f) != 0) return -1;
		if(Sanitize(h.SAS_TrimAilvMin, -0.3f, 0.3f) != 0) return -1;
		if(Sanitize(h.SAS_TrimAilvMax, -0.3f, 0.3f) != 0) return -1;
		if(Sanitize(h.SAS_TrimElevNom, -0.5f, 0.5f) != 0) return -1;
		if(Sanitize(h.SAS_TrimElevMin, -0.5f, 0.5f) != 0) return -1;
		if(Sanitize(h.SAS_TrimElevMax, -0.5f, 0.5f) != 0) return -1;
		if(Sanitize(h.SAS_TrimRudvNom, -0.3f, 0.3f) != 0) return -1;
		if(Sanitize(h.SAS_TrimRudvMin, -0.3f, 0.3f) != 0) return -1;
		if(Sanitize(h.SAS_TrimRudvMax, -0.3f, 0.3f) != 0) return -1;

		if(Sanitize(h.CAS_PitchPGain,0.0f, 10.0f) != 0) return -1;
		if(Sanitize(h.CAS_PitchIGain,0.0f, 3.0f) != 0) return -1;
		if(Sanitize(h.CAS_RollPGain,0.0f, 10.0f) != 0) return -1;
		if(Sanitize(h.CAS_PitchAngleLim,0.0f, 45.0f * DEG2RAD) != 0) return -1;
		if(Sanitize(h.CAS_RollAngleLim,0.0f, 60.0f * DEG2RAD) != 0) return -1;

		if(Sanitize(h.CAS_HeadPGain,0.0f, 10.0f) != 0) return -1;
		if(Sanitize(h.HL_vZClimb,0.0f, 5.0f) != 0) return -1;
		if(Sanitize(h.HL_vZSink,0.0f, 5.0f) != 0) return -1;
		if(Sanitize(h.HL_Vel_vNom, 0.0f, 15.0f) != 0) return -1;

		if(Sanitize(h.pitch_limit_min, -60.0f, 0.0f) != 0) return -1;
		if(Sanitize(h.pitch_limit_max, 0.0f, 60.0f) != 0) return -1;
	}

	//ASLCTRL-GENERAL
	param_get(h.ASLC_DEBUG, &(p.ASLC_DEBUG));
	param_get(h.ASLC_CtrlType, &(p.ASLC_CtrlType));
	param_get(h.ASLC_GainSch_E, &(p.ASLC_GainSch_E));
	param_get(h.ASLC_GainSch_Q, &(p.ASLC_GainSch_Q));
	param_get(h.ASLC_StallProt, &(p.ASLC_StallProt));
	param_get(h.ASLC_VelCtrl, &(p.ASLC_VelCtrl));
	param_get(h.ASLC_OnRCLoss, &(p.ASLC_OnRCLoss));
	param_get(h.ASLC_OvSpdProt, &(p.ASLC_OvSpdProt));
	param_get(h.ASLC_CoordTurn, &(p.ASLC_CoordTurn));

	//DEMIX
	param_get(h.DEMIX_Enabled, &(p.DEMIX_Enabled));
	param_get(h.DEMIX_F_CH2t4, &(p.DEMIX_F_CH2t4));
	param_get(h.DEMIX_F_CH6t1, &(p.DEMIX_F_CH6t1));
	param_get(h.DEMIX_CH2t4, &(p.DEMIX_CH2t4));
	param_get(h.DEMIX_CH6t1, &(p.DEMIX_CH6t1));

	//SAS
	param_get(h.SAS_tSample, &(p.SAS_tSample));
	param_get(h.SAS_RollPGain, &(p.SAS_RollPGain));
	param_get(h.SAS_PitchPGain, &(p.SAS_PitchPGain));
	param_get(h.SAS_YawPGain, &(p.SAS_YawPGain));
	param_get(h.SAS_RollPDir, &(p.SAS_RollPDir));
	param_get(h.SAS_PitchPDir, &(p.SAS_PitchPDir));
	param_get(h.SAS_YawPDir, &(p.SAS_YawPDir));
	param_get(h.SAS_RollYawDecoupleKari, &(p.SAS_RollYawDecoupleKari));
	param_get(h.SAS_YawCTkP, &(p.SAS_YawCTkP));
	param_get(h.SAS_YawCTFF, &(p.SAS_YawCTFF));
	param_get(h.SAS_RCtrlLim, &(p.SAS_RCtrlLim));
	param_get(h.SAS_PCtrlLim, &(p.SAS_PCtrlLim));
	param_get(h.SAS_YCtrlLim, &(p.SAS_YCtrlLim));
	param_get(h.SAS_vScaleLimF, &(p.SAS_vScaleLimF));
	param_get(h.SAS_vScaleExp, &(p.SAS_vScaleExp));
	param_get(h.SAS_YawHighPassOmega, &(p.SAS_YawHighPassOmega));
	param_get(h.SAS_PitchLowPassOmega, &(p.SAS_PitchLowPassOmega));
	param_get(h.SAS_RollLowPassOmega, &(p.SAS_RollLowPassOmega));
	param_get(h.SAS_TrimAilvNom, &(p.SAS_TrimAilvNom));
	param_get(h.SAS_TrimAilvMin, &(p.SAS_TrimAilvMin));
	param_get(h.SAS_TrimAilvMax, &(p.SAS_TrimAilvMax));
	param_get(h.SAS_TrimElevNom, &(p.SAS_TrimElevNom));
	param_get(h.SAS_TrimElevMin, &(p.SAS_TrimElevMin));
	param_get(h.SAS_TrimElevMax, &(p.SAS_TrimElevMax));
	param_get(h.SAS_TrimRudvNom, &(p.SAS_TrimRudvNom));
	param_get(h.SAS_TrimRudvMin, &(p.SAS_TrimRudvMin));
	param_get(h.SAS_TrimRudvMax, &(p.SAS_TrimRudvMax));

	//CAS
	param_get(h.CAS_fMult, &(p.CAS_fMult));
	param_get(h.CAS_PitchPGain, &(p.CAS_PitchPGain));
	param_get(h.CAS_PitchPGainM, &(p.CAS_PitchPGainM));
	param_get(h.CAS_PitchIGain, &(p.CAS_PitchIGain));
	param_get(h.CAS_RollPGain, &(p.CAS_RollPGain));
	param_get(h.CAS_RollPGainM, &(p.CAS_RollPGainM));
	param_get(h.CAS_HeadPGain, &(p.CAS_HeadPGain));
	param_get(h.CAS_q2uPGain, &(p.CAS_q2uPGain));
	param_get(h.CAS_p2uPGain, &(p.CAS_p2uPGain));
	param_get(h.CAS_PitchRateLim, &(p.CAS_PitchRateLim));
	param_get(h.CAS_PitchRateILim, &(p.CAS_PitchRateILim));
	param_get(h.CAS_PitchTCkI, &(p.CAS_PitchTCkI));
	param_get(h.CAS_PitchTCILim, &(p.CAS_PitchTCILim));
	param_get(h.CAS_RollRateLim, &(p.CAS_RollRateLim));
	param_get(h.CAS_YawRateLim, &(p.CAS_YawRateLim));
	param_get(h.CAS_PitchAngleLim, &(p.CAS_PitchAngleLim));
	param_get(h.CAS_RollAngleLim, &(p.CAS_RollAngleLim));
	param_get(h.CAS_YawLowPassOmega, &(p.CAS_YawLowPassOmega));
	param_get(h.CAS_uElevTurnFF, &(p.CAS_uElevTurnFF));

	//HL
	param_get(h.HL_fMult, &(p.HL_fMult));
	param_get(h.HL_WPL1_Damping, &(p.HL_WPL1_Damping));
	param_get(h.HL_WPL1_P_vMin, &(p.HL_WPL1_P_vMin));
	param_get(h.HL_WPL1_P_vNom, &(p.HL_WPL1_P_vNom));
	param_get(h.HL_WPL1_P_vMax, &(p.HL_WPL1_P_vMax));
	param_get(h.HL_Vel_vNom, &(p.HL_Vel_vNom));
	param_get(h.HL_Vel_vMin, &(p.HL_Vel_vMin));
	param_get(h.HL_Vel_vMax, &(p.HL_Vel_vMax));
	//Old altitude controller
	param_get(h.HL_vZClimb, &(p.HL_vZClimb));
	param_get(h.HL_vZSink, &(p.HL_vZSink));
	param_get(h.HL_AlthMax, &(p.HL_AlthMax));
	param_get(h.HL_AlthMin, &(p.HL_AlthMin));
	param_get(h.HL_AltLowPassOmega, &(p.HL_AltLowPassOmega));

	//TECS
	param_get(h.airspeed_min, &(p.airspeed_min));
	param_get(h.airspeed_trim, &(p.airspeed_trim));
	param_get(h.airspeed_max, &(p.airspeed_max));
	param_get(h.pitch_limit_min, &(p.pitch_limit_min));
	param_get(h.pitch_limit_max, &(p.pitch_limit_max));
	param_get(h.throttle_min, &(p.throttle_min));
	param_get(h.throttle_max, &(p.throttle_max));
	param_get(h.throttle_cruise, &(p.throttle_cruise));
	param_get(h.time_const, &(p.time_const));
	param_get(h.time_const_throt, &(p.time_const_throt));
	param_get(h.min_sink_rate, &(p.min_sink_rate));
	param_get(h.max_sink_rate, &(p.max_sink_rate));
	param_get(h.throttle_damp, &(p.throttle_damp));
	param_get(h.integrator_gain, &(p.integrator_gain));
	param_get(h.vertical_accel_limit, &(p.vertical_accel_limit));
	param_get(h.height_comp_filter_omega, &(p.height_comp_filter_omega));
	param_get(h.speed_comp_filter_omega, &(p.speed_comp_filter_omega));
	param_get(h.roll_throttle_compensation, &(p.roll_throttle_compensation));
	param_get(h.speed_weight, &(p.speed_weight));
	param_get(h.pitch_damping, &(p.pitch_damping));
	param_get(h.max_climb_rate, &(p.max_climb_rate));
	param_get(h.heightrate_p, &(p.heightrate_p));
	param_get(h.heightrate_ff, &(p.heightrate_ff));
	param_get(h.speedrate_p, &(p.speedrate_p));
	param_get(h.throttle_slewrate, &(p.throttle_slewrate));
	param_get(h.throttle_ILim, &(p.throttle_ILim));

	return 0;
}

int parameters::Sanitize(param_t & parameter_handle, float minval, float maxval)
{
	float temp_param;
	param_get(parameter_handle, &(temp_param));

	if(temp_param > maxval) {
		param_set(parameter_handle,&maxval);
		return -1;
	}
	else if (temp_param < minval) {
		param_set(parameter_handle,&minval);
		return -1;
	}
	else return 0;
}


