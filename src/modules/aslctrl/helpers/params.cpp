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
	//General
	bInitialized=true;
	bSanityChecksEnabled=true;

	return 0;
}

int parameters::update()
{
	if(!bInitialized) { init(); }

	param_t handle;

	//ASLCTRL-GENERAL
	handle=param_find("ASLC_DEBUG");
	param_get(handle, &(p.ASLC_DEBUG));
	handle=param_find("ASLC_CtrlType");
	param_get(handle, &(p.ASLC_CtrlType));
	handle=param_find("ASLC_GainSch_E");
	param_get(handle, &(p.ASLC_GainSch_E));
	handle=param_find("ASLC_GainSch_Q");
	param_get(handle, &(p.ASLC_GainSch_Q));
	handle=param_find("ASLC_StallProt");
	param_get(handle, &(p.ASLC_StallProt));
	handle=param_find("ASLC_VelCtrl");
	param_get(handle, &(p.ASLC_VelCtrl));
	handle=param_find("ASLC_OnRCLoss");
	param_get(handle, &(p.ASLC_OnRCLoss));
	handle=param_find("ASLC_OvSpdProt");
	param_get(handle, &(p.ASLC_OvSpdProt));
	handle=param_find("ASLC_CoordTurn");
	param_get(handle, &(p.ASLC_CoordTurn));

	//DEMIX
	handle=param_find("DEMIX_Enabled");
	param_get(handle, &(p.DEMIX_Enabled));
	handle=param_find("DEMIX_F_CH2t4");
	param_get(handle, &(p.DEMIX_F_CH2t4));
	handle=param_find("DEMIX_F_CH6t1");
	param_get(handle, &(p.DEMIX_F_CH6t1));
	handle=param_find("DEMIX_CH2t4");
	param_get(handle, &(p.DEMIX_CH2t4));
	handle=param_find("DEMIX_CH6t1");
	param_get(handle, &(p.DEMIX_CH6t1));

	//SAS
	handle=param_find("SAS_tSample");
	param_get(handle, &(p.SAS_tSample));
	handle=param_find("SAS_RollPGain");
	if(bSanityChecksEnabled && Sanitize(handle,0.0f, 10.0f) != 0) return -1;
	param_get(handle, &(p.SAS_RollPGain));
	handle=param_find("SAS_PitchPGain");
	if(bSanityChecksEnabled && Sanitize(handle,0.0f, 10.0f) != 0) return -1;
	param_get(handle, &(p.SAS_PitchPGain));
	handle=param_find("SAS_YawPGain");
	if(bSanityChecksEnabled && Sanitize(handle,0.0f, 10.0f) != 0) return -1;
	param_get(handle, &(p.SAS_YawPGain));
	handle=param_find("SAS_RollPDir");
	param_get(handle, &(p.SAS_RollPDir));
	handle=param_find("SAS_PitchPDir");
	param_get(handle, &(p.SAS_PitchPDir));
	handle=param_find("SAS_YawPDir");
	param_get(handle, &(p.SAS_YawPDir));
	handle=param_find("SAS_RYDecK");
	param_get(handle, &(p.SAS_RollYawDecoupleKari));
	handle=param_find("SAS_YawCTkP");
	param_get(handle, &(p.SAS_YawCTkP));
	handle=param_find("SAS_YawCTFF");
	param_get(handle, &(p.SAS_YawCTFF));
	handle=param_find("SAS_RCtrlLim");
	param_get(handle, &(p.SAS_RCtrlLim));
	handle=param_find("SAS_PCtrlLim");
	param_get(handle, &(p.SAS_PCtrlLim));
	handle=param_find("SAS_YCtrlLim");
	param_get(handle, &(p.SAS_YCtrlLim));
	handle=param_find("SAS_vScaleLimF");
	param_get(handle, &(p.SAS_vScaleLimF));
	handle=param_find("SAS_vScaleExp");
	param_get(handle, &(p.SAS_vScaleExp));
	handle=param_find("SAS_YawHPw");
	param_get(handle, &(p.SAS_YawHighPassOmega));
	handle=param_find("SAS_PitchLPw");
	param_get(handle, &(p.SAS_PitchLowPassOmega));
	handle=param_find("SAS_RollLPw");
	param_get(handle, &(p.SAS_RollLowPassOmega));
	handle=param_find("SAS_TrimAilvNom");
	if(bSanityChecksEnabled && Sanitize(handle, -0.3f, 0.3f) != 0) return -1;
	param_get(handle, &(p.SAS_TrimAilvNom));
	handle=param_find("SAS_TrimAilvMin");
	if(bSanityChecksEnabled && Sanitize(handle, -0.3f, 0.3f) != 0) return -1;
	param_get(handle, &(p.SAS_TrimAilvMin));
	handle=param_find("SAS_TrimAilvMax");
	if(bSanityChecksEnabled && Sanitize(handle, -0.3f, 0.3f) != 0) return -1;
	param_get(handle, &(p.SAS_TrimAilvMax));
	handle=param_find("SAS_TrimElevNom");
	if(bSanityChecksEnabled && Sanitize(handle, -0.5f, 0.5f) != 0) return -1;
	param_get(handle, &(p.SAS_TrimElevNom));
	handle=param_find("SAS_TrimElevMin");
	if(bSanityChecksEnabled && Sanitize(handle, -0.5f, 0.5f) != 0) return -1;
	param_get(handle, &(p.SAS_TrimElevMin));
	handle=param_find("SAS_TrimElevMax");
	if(bSanityChecksEnabled && Sanitize(handle, -0.5f, 0.5f) != 0) return -1;
	param_get(handle, &(p.SAS_TrimElevMax));
	handle=param_find("SAS_TrimRudvNom");
	if(bSanityChecksEnabled && Sanitize(handle, -0.3f, 0.3f) != 0) return -1;
	param_get(handle, &(p.SAS_TrimRudvNom));
	handle=param_find("SAS_TrimRudvMin");
	if(bSanityChecksEnabled && Sanitize(handle, -0.3f, 0.3f) != 0) return -1;
	param_get(handle, &(p.SAS_TrimRudvMin));
	handle=param_find("SAS_TrimRudvMax");
	if(bSanityChecksEnabled && Sanitize(handle, -0.3f, 0.3f) != 0) return -1;
	param_get(handle, &(p.SAS_TrimRudvMax));

	//CAS
	handle=param_find("CAS_fMult");
	param_get(handle, &(p.CAS_fMult));
	handle=param_find("CAS_PitchPGain");
	if(bSanityChecksEnabled && Sanitize(handle,0.0f, 10.0f) != 0) return -1;
	param_get(handle, &(p.CAS_PitchPGain));
	handle=param_find("CAS_PitchPGainM");
	param_get(handle, &(p.CAS_PitchPGainM));
	handle=param_find("CAS_PitchIGain");
	if(bSanityChecksEnabled && Sanitize(handle,0.0f, 3.0f) != 0) return -1;
	param_get(handle, &(p.CAS_PitchIGain));
	handle=param_find("CAS_RollPGain");
	if(bSanityChecksEnabled && Sanitize(handle,0.0f, 10.0f) != 0) return -1;
	param_get(handle, &(p.CAS_RollPGain));
	handle=param_find("CAS_RollPGainM");
	param_get(handle, &(p.CAS_RollPGainM));
	handle=param_find("CAS_HeadPGain");
	if(bSanityChecksEnabled && Sanitize(handle,0.0f, 10.0f) != 0) return -1;
	param_get(handle, &(p.CAS_HeadPGain));
	handle=param_find("CAS_q2uPGain");
	param_get(handle, &(p.CAS_q2uPGain));
	handle=param_find("CAS_p2uPGain");
	param_get(handle, &(p.CAS_p2uPGain));
	handle=param_find("CAS_PRateLim");
	param_get(handle, &(p.CAS_PitchRateLim));
	handle=param_find("CAS_PRateILim");
	param_get(handle, &(p.CAS_PitchRateILim));
	handle=param_find("CAS_PitchTCkI");
	param_get(handle, &(p.CAS_PitchTCkI));
	handle=param_find("CAS_PitchTCILim");
	param_get(handle, &(p.CAS_PitchTCILim));
	handle=param_find("CAS_RRateLim");
	param_get(handle, &(p.CAS_RollRateLim));
	handle=param_find("CAS_YRateLim");
	param_get(handle, &(p.CAS_YawRateLim));
	handle=param_find("CAS_PAngleLim");
	if(bSanityChecksEnabled && Sanitize(handle,0.0f, 45.0f * DEG2RAD) != 0) return -1;
	param_get(handle, &(p.CAS_PitchAngleLim));
	handle=param_find("CAS_RAngleLim");
	if(bSanityChecksEnabled && Sanitize(handle,0.0f, 60.0f * DEG2RAD) != 0) return -1;
	param_get(handle, &(p.CAS_RollAngleLim));
	handle=param_find("CAS_YawLPw");
	param_get(handle, &(p.CAS_YawLowPassOmega));
	handle= param_find("CAS_uElevTurnFF");
	param_get(handle, &(p.CAS_uElevTurnFF));

	//HL
	handle=param_find("HL_fMult");
	param_get(handle, &(p.HL_fMult));
	handle=param_find("HL_WPL1_Damping");
	param_get(handle, &(p.HL_WPL1_Damping));
	handle=param_find("HL_WPL1_P_vMin");
	param_get(handle, &(p.HL_WPL1_P_vMin));
	handle=param_find("HL_WPL1_P_vNom");
	param_get(handle, &(p.HL_WPL1_P_vNom));
	handle=param_find("HL_WPL1_P_vMax");
	param_get(handle, &(p.HL_WPL1_P_vMax));
	handle=param_find("HL_Vel_vNom");
	if(bSanityChecksEnabled && Sanitize(handle, 0.0f, 15.0f) != 0) return -1;
	param_get(handle, &(p.HL_Vel_vNom));
	handle=param_find("HL_Vel_vMin");
	param_get(handle, &(p.HL_Vel_vMin));
	handle=param_find("HL_Vel_vMax");
	param_get(handle, &(p.HL_Vel_vMax));
	//Old altitude controller
	handle=param_find("HL_AltLPw");
	param_get(handle, &(p.HL_AltLowPassOmega));
	handle=param_find("HL_AlthMax");
	param_get(handle, &(p.HL_AlthMax));
	handle=param_find("HL_AlthMin");
	param_get(handle, &(p.HL_AlthMin));
	handle= param_find("HL_vZClimb");
	if(bSanityChecksEnabled && Sanitize(handle,0.0f, 5.0f) != 0) return -1;
	param_get(handle, &(p.HL_vZClimb));
	handle= param_find("HL_vZSink");
	if(bSanityChecksEnabled && Sanitize(handle,0.0f, 5.0f) != 0) return -1;
	param_get(handle, &(p.HL_vZSink));

	//TECS & HL
	handle = param_find("HL_Vel_vMin");
	param_get(handle, &(p.airspeed_min));
	handle = param_find("HL_Vel_vNom");
	param_get(handle, &(p.airspeed_trim));
	handle = param_find("HL_Vel_vMax");
	param_get(handle, &(p.airspeed_max));
	handle = param_find("FW_P_LIM_MIN");
	if(bSanityChecksEnabled && Sanitize(handle, -60.0f, 0.0f) != 0) return -1;
	param_get(handle, &(p.pitch_limit_min));
	handle = param_find("FW_P_LIM_MAX");
	if(bSanityChecksEnabled && Sanitize(handle, 0.0f, 60.0f) != 0) return -1;
	param_get(handle, &(p.pitch_limit_max));
	handle = param_find("FW_THR_MIN");
	param_get(handle, &(p.throttle_min));
	handle = param_find("FW_THR_MAX");
	param_get(handle, &(p.throttle_max));
	handle = param_find("FW_THR_CRUISE");
	param_get(handle, &(p.throttle_cruise));
	handle = 	param_find("FW_T_TIME_CONST");
	param_get(handle, &(p.time_const));
	handle = param_find("FW_T_TC_THROT");
	param_get(handle, &(p.time_const_throt));
	handle = param_find("FW_T_SINK_MIN");
	param_get(handle, &(p.min_sink_rate));
	handle = param_find("FW_T_SINK_MAX");
	param_get(handle, &(p.max_sink_rate));
	handle = param_find("FW_T_CLMB_MAX");
	param_get(handle, &(p.max_climb_rate));
	handle = param_find("FW_T_THR_DAMP");
	param_get(handle, &(p.throttle_damp));
	handle =	param_find("FW_T_INTEG_GAIN");
	param_get(handle, &(p.integrator_gain));
	handle = param_find("FW_T_VERT_ACC");
	param_get(handle, &(p.vertical_accel_limit));
	handle = param_find("FW_T_HGT_OMEGA");
	param_get(handle, &(p.height_comp_filter_omega));
	handle =	param_find("FW_T_SPD_OMEGA");
	param_get(handle, &(p.speed_comp_filter_omega));
	handle = param_find("FW_T_RLL2THR");
	param_get(handle, &(p.roll_throttle_compensation));
	handle = param_find("FW_T_SPDWEIGHT");
	param_get(handle, &(p.speed_weight));
	handle = param_find("FW_T_PTCH_DAMP");
	param_get(handle, &(p.pitch_damping));
	handle = param_find("FW_T_HRATE_P");
	param_get(handle, &(p.heightrate_p));
	handle = param_find("FW_T_HRATE_FF");
	param_get(handle, &(p.heightrate_ff));
	handle =	param_find("FW_T_SRATE_P");
	param_get(handle, &(p.speedrate_p));
	handle = param_find("FW_T_THRSLEW");
	param_get(handle, &(p.throttle_slewrate));
	handle = param_find("FW_T_ThrILim");
	param_get(handle, &(p.throttle_ILim));

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


