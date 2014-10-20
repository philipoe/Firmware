/*
 * HL.cpp
 *
 *  Created on: 04.10.2013
 *      Author: philipoe
 */

#include <stdio.h>
#include <mathlib/mathlib.h>
#include "HL.h"
#include "helpers/helpfuncs.h"
#include "helpers/consts.h"

HL::HL() : LP_Airspeed(0,0),MA_Airspeed(4,0.0f),LP_vZ(0,0),LP_h(0,0)
{
	bSpoilerAltExceeded=false;
	bhMinExceeded=false;
};

HL::HL(parameters *params_arg, subscriptions *subs_arg) : LP_Airspeed(0,0),MA_Airspeed(4,0.0f),LP_vZ(0,0),LP_h(0,0)
{
	params=params_arg;
	subs=subs_arg;
	t_old=0;
	bSpoilerAltExceeded=false;
	bhMinExceeded=false;
};

void HL::CopyUpdatedParams(void)
{
	// When parameters have been updated, these can be accessed through the appropriate pointer and (if necessary, depending on the param)
	// are written to the objects requiring them.
	float HL_tSample=params->p.SAS_tSample*params->p.HL_fMult;

	L1Ctrl.set_l1_damping(params->p.HL_WPL1_Damping);
	//L1Ctrl.set_l1_period(params->p.HL_WPL1_Period);
	LP_Airspeed.SetGains(HL_tSample,1.0f); //1.0 rad/s
	LP_vZ.SetGains(HL_tSample,1.0f); //1.0 rad/s
	LP_h.SetGains(HL_tSample,params->p.HL_AltLowPassOmega);

	tecs.enable_airspeed(true);
	tecs.set_time_const(params->p.time_const);
	tecs.set_time_const_throt(params->p.time_const_throt);
	tecs.set_min_sink_rate(params->p.min_sink_rate);
	tecs.set_max_sink_rate(params->p.max_sink_rate);
	tecs.set_throttle_damp(params->p.throttle_damp);
	tecs.set_integrator_gain(params->p.integrator_gain);
	tecs.set_throtILim(params->p.throttle_ILim);
	tecs.set_vertical_accel_limit(params->p.vertical_accel_limit);
	tecs.set_height_comp_filter_omega(params->p.height_comp_filter_omega);
	tecs.set_speed_comp_filter_omega(params->p.speed_comp_filter_omega);
	tecs.set_roll_throttle_compensation(math::radians(params->p.roll_throttle_compensation));
	tecs.set_speed_weight(params->p.speed_weight);
	tecs.set_pitch_damping(params->p.pitch_damping);
	tecs.set_indicated_airspeed_min(params->p.airspeed_min);
	tecs.set_indicated_airspeed_max(params->p.airspeed_max);
	tecs.set_max_climb_rate(params->p.max_climb_rate);
	tecs.set_heightrate_p(params->p.heightrate_p);
	tecs.set_speedrate_p(params->p.speedrate_p);
	tecs.set_throttle_slewrate(params->p.throttle_slewrate);	//Added
}

int HL::WaypointControl_L1(float &RollAngleRef)
{
	math::Vector2f ground_speed(subs->global_pos.vx, subs->global_pos.vy);
	math::Vector2f cur_pos(float(subs->global_pos.lat)/1.0E7f, float(subs->global_pos.lon)/1.0E7f);

	// *********************************************************************
	// *** L1-PERIOD GAIN SCHEDULING
	// *********************************************************************
	float airspeed_lim = limit2(LP_Airspeed.update(subs->sensors.dbaro_velo_ms),params->p.HL_Vel_vMax,params->p.HL_Vel_vMin);
	float L1_P_GS = params->p.HL_WPL1_P_vNom;

	if(airspeed_lim > 1.0f) {
		// Bi-linear interpolation when airspeed =[vMin...vMax].
		if(airspeed_lim < params->p.HL_Vel_vNom) {
			float dAirspeed_rel=(airspeed_lim-params->p.HL_Vel_vMin)/(params->p.HL_Vel_vNom-params->p.HL_Vel_vMin); //To speed up interpolation
			L1_P_GS = interp1_lin(dAirspeed_rel, params->p.HL_WPL1_P_vMin,params->p.HL_WPL1_P_vNom);
		}
		else { //bigger or equal vNom
			float dAirspeed_rel=(airspeed_lim-params->p.HL_Vel_vNom)/(params->p.HL_Vel_vMax-params->p.HL_Vel_vNom); //To speed up interpolation
			L1_P_GS = interp1_lin(dAirspeed_rel, params->p.HL_WPL1_P_vNom,params->p.HL_WPL1_P_vMax);
		}
	}
	L1Ctrl.set_l1_period(L1_P_GS);

	if(params->p.ASLC_DEBUG == 22) printf("L1_GS: vAir:%.2f L1_P_GS:%.2f\n",LP_Airspeed.Get(),L1_P_GS);

	// *********************************************************************
	// *** STD-WAYPOINT CONTROL
	// *********************************************************************
	if (subs->global_pos_set_triplet.current.nav_cmd == NAV_CMD_WAYPOINT) {
		/* waypoint is a plain navigation waypoint */
		if(params->p.ASLC_DEBUG==10) printf("M5.1: ");

		math::Vector2f next_wp(float(subs->global_pos_set_triplet.current.lat)/1.0E7f,float(subs->global_pos_set_triplet.current.lon)/1.0E7f);
		math::Vector2f prev_wp(float(subs->global_pos_set_triplet.previous.lat)/1.0E7f,float(subs->global_pos_set_triplet.previous.lon)/1.0E7f);
		if (!subs->global_pos_set_triplet.previous_valid) {
			// No valid next waypoint, go for heading hold. This is automatically handled by the L1 library.
			prev_wp.setX(subs->global_pos_set_triplet.current.lat / 1e7f);
			prev_wp.setY(subs->global_pos_set_triplet.current.lon / 1e7f);
		}
		L1Ctrl.navigate_waypoints(prev_wp, next_wp, cur_pos, ground_speed);

		if(params->p.ASLC_DEBUG==10) printf("Previous WP: (%7.5f,%7.5f) Next WP (x/y): (%7.5f,%7.5f) | ", prev_wp.getX(), prev_wp.getY(), next_wp.getX(), next_wp.getY());
	}
	else if (subs->global_pos_set_triplet.current.nav_cmd == NAV_CMD_LOITER_TURN_COUNT ||
			subs->global_pos_set_triplet.current.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
			subs->global_pos_set_triplet.current.nav_cmd == NAV_CMD_LOITER_UNLIMITED) {
		/* waypoint is a loiter waypoint */
		math::Vector2f next_wp(float(subs->global_pos_set_triplet.current.lat)/1.0E7f,float(subs->global_pos_set_triplet.current.lon)/1.0E7f);
		if(params->p.ASLC_DEBUG==10) printf("M5.2: Next WP (x/y): (%7.5f,%7.5f) | ", next_wp.getX(), next_wp.getY());

		if(params->p.ASLC_DEBUG <27) {
			L1Ctrl.navigate_loiter(next_wp, cur_pos, subs->global_pos_set_triplet.current.loiter_radius,
					subs->global_pos_set_triplet.current.loiter_direction, ground_speed);
		}
		else {
			L1Ctrl.navigate_loiter_adapt(next_wp, cur_pos, subs->global_pos_set_triplet.current.loiter_radius,
								subs->global_pos_set_triplet.current.loiter_direction, ground_speed,subs->att.yaw,subs->global_pos_set_triplet.current.altitude, subs->global_pos.alt);
		}

	}
	else {
		/* RETURN TO LAUNCH (RTL) */
		if(params->p.ASLC_DEBUG==10) printf("M5.3: Home=(%7.5f,%7.5f,%.3f)\n",float(subs->home_pos.lat)/1.0E7f,float(subs->home_pos.lon)/1.0E7f,float(subs->home_pos.alt));
		math::Vector2f rtl_pos(float(subs->home_pos.lat)/1.0E7f, float(subs->home_pos.lon)/1.0E7f);

		L1Ctrl.navigate_waypoints(rtl_pos, rtl_pos, cur_pos, ground_speed);
	}

	RollAngleRef = limit1(L1Ctrl.nav_roll(), params->p.CAS_RollAngleLim);

	if(params->p.ASLC_DEBUG==10) printf("Roll Angle Ref(unlim):%7.4f \n",L1Ctrl.nav_roll());

	return 0;
}

int HL::TECS_AltAirspeedControl(float &PitchAngleRef, float& uThrot, float& AirspeedRef, float &hRef, float const &h, float const h_home, float &hRef_t, uint8_t & AltitudeStatus, bool& bEngageSpoilers, const bool bUseRamp, const bool bUseThermalHighEtaMode, const bool bModeChanged)
{
	//Absolute altitude controller
	//printf("hRef: %7.4f h:%7.4f h_home: %7.4f velw: %7.4f\n", hRef, h, h_home,velw);

	if(params->p.ASLC_DEBUG==11) printf("AltCtrl: bModeChanged:%u, bUseRamp:%u, h:%.2f, hRef:%.2f, hRef_t:%.2f\n",bModeChanged,bUseRamp,h,hRef,hRef_t);

	int RET = 0;
	//Safety checks. Either
	//a) Errors (return error code directly) or
	if(h_home<0.0f || h_home>5000.0f) 				{return -1;}						// First, check whether home altitude is OK
	if(params->p.HL_AlthMax < h_home+50.0f)			{return -2;}						// To catch a wrong user input
	//b) Warnings (return RET after execution)
	if(hRef<h_home+50.0f || hRef>h_home+5000.0f) 	{RET=-3; hRef=h_home+150.0;} 		// Relative altitude setpoint can be between these values
	if(h<h_home-100.0f || h>h_home+1000.0f) 		{RET=-4;} 							// Relative altitude can be between these values
	//if(fabs(hRef-h)>300.0f)						{return -5;}  						// Maximum altitude change
	if(hRef>params->p.HL_AlthMax)					{RET=-6; hRef=params->p.HL_AlthMax;} // Altitude-ref may not exceed max alt limit
	if(params->p.HL_AlthMax<params->p.HL_AlthMin)	{RET=-7; params->p.HL_AlthMin=params->p.HL_AlthMax;}
	if(hRef<params->p.HL_AlthMin)					{RET=-8; hRef=params->p.HL_AlthMin;}

	//Using Ramp for altitude? Then calculate new altitude reference (ramp-function)
	if(bUseRamp) CalcAltitudeRamp(hRef_t, hRef, h, bModeChanged);
	else hRef_t = hRef;

	//If using the relaxed-altitude-tracking (thermal compliance) mode, the following function makes the necessary parameter mods
	CalcThermalModeModifications(subs->global_pos.alt, hRef_t, bEngageSpoilers, bUseThermalHighEtaMode);

	// Try to protect against bad descents caused by a) thermal downdrafts or b) unreachable velocity references (with simple hysteresis)
	if(subs->global_pos.alt < params->p.HL_AlthMin) bhMinExceeded=true;
	if(subs->global_pos.alt > params->p.HL_AlthMin+20.0f) bhMinExceeded=false;
	float maxThrot=params->p.throttle_max;
	if(bhMinExceeded)
	{
		if(maxThrot<1.0f) maxThrot=1.0f;	// This protects mainly against a) thermal downdrafts. Compromise maxThrottle constraint here for max safety, i.e. allow temporary uThrot>uThrotMax
		AirspeedRef=params->p.HL_Vel_vNom;	// This protects mainly against b) bad descents caused by an unreachable (i.e. too high) airspeed reference.
		RET=-9;								// This warns the user to take further measures in case the above does not suffice as a protection
	}

	//TECS altitude & airspeed controller
	tecs.update_pitch_throttle(R_nb, subs->att.pitch, subs->global_pos.alt, hRef_t,
						AirspeedRef,subs->sensors.dbaro_velo_ms, 1.0f, false, math::radians(params->p.pitch_limit_min),
						params->p.throttle_min, maxThrot, params->p.throttle_cruise, math::radians(params->p.pitch_limit_min),
						math::radians(params->p.pitch_limit_max));

	PitchAngleRef = tecs.get_pitch_demand();
	uThrot = tecs.get_throttle_demand();

	//Limiters. TODO: Add additional checks here.
	PitchAngleRef=limit1(PitchAngleRef,params->p.CAS_PitchAngleLim);

	// Set ALTITUDE_STATUS
	if(hRef_t == hRef) AltitudeStatus = ALTITUDE_OK_LEVELING;
	else if(hRef_t < hRef) AltitudeStatus = ALTITUDE_OK_CLIMBING;
	else if(hRef_t > hRef) AltitudeStatus = ALTITUDE_OK_DESCENDING;
	if(h < hRef_t - 20.0f && PitchAngleRef>params->p.CAS_PitchAngleLim) {
		// Check for any large altitude undershoot. We need too increase throttle if we are descending too much!
		AltitudeStatus = ALTITUDE_WARNING_ALTLOW;
		RET=-10;
	}

	return RET;
}

int HL::TECS_Update50Hz(void)
{
	//This function updates the TECS data fields. It should be run at >50Hz.
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
		R_nb(i, j) = subs->att.R[i][j];
	math::Vector3 accel_body(subs->sensors.accelerometer_m_s2[0], subs->sensors.accelerometer_m_s2[1], subs->sensors.accelerometer_m_s2[2]);
	math::Vector3 accel_earth = R_nb.transpose() * accel_body;

	tecs.update_50hz(subs->global_pos.alt, subs->sensors.dbaro_velo_ms, R_nb, accel_body, accel_earth);

	return 0;
}

int HL::CalcAltitudeRamp(float& hRef_t, const float& hRef, const float& h, const bool& bModeChanged)
{
	// This function Calculates an altitude ramp (hRef_t)

	float dt = (hrt_absolute_time()-t_old)/1.0E6;

	if(dt > 1.0 /*|| bModeChanged*/) {
		//Reinit (because of ModeChange or first init), but don't run for now!
		hRef_t = h;
		t_old = hrt_absolute_time();
		return -1;
	}

	if((hRef_t < hRef+dt*params->p.HL_vZSink) && (hRef_t > hRef-dt*params->p.HL_vZClimb)) {
		// hRef_t is so close to hRef that it would exceed it in the next step -> Set it to hRef
		// to avoid this small overshoot.
		hRef_t=hRef;
	}
	else {
		//Do the standard ramp
		if(hRef_t < hRef) hRef_t += dt*params->p.HL_vZClimb;
		else if(hRef_t > hRef) hRef_t -= dt*params->p.HL_vZSink;
	}

	t_old = hrt_absolute_time();

	//DEBUG
	if(params->p.ASLC_DEBUG==11) printf("AltRAMP: bModeChanged:%u, h:%.2f, hRef:%.2f, hRef_t:%.2f\n",bModeChanged,h,hRef,hRef_t);

	return 0;
}

const float dh_offset_spoilers = 50.0f;
const float dh_offset_spoilers_hysteresis = 10.0f;
const float dh_offset_spdweight_hmax = 20.0f;
const float dh_offset_spdweight_href = 5.0f;

int HL::CalcThermalModeModifications(const float& h, const float& hRef_t, bool& bEngageSpoilers, bool bUseThermalHighEtaMode)
{
	//Set default values if not using thermal mode or if wrong values for AtlhMax were set.
	if(!bUseThermalHighEtaMode || params->p.HL_AlthMax<0.1f) {
		bEngageSpoilers=false;
		tecs.set_speed_weight(params->p.speed_weight);
		tecs.set_roll_throttle_compensation(params->p.roll_throttle_compensation);
		tecs.set_heightrate_p(params->p.heightrate_p);
	}

	//Set adapted values for altitude control if necessary
	if(bUseThermalHighEtaMode && params->p.HL_AlthMax>0.1f) {

		float dh_rel=0.0f, adapted_spd_weight=0.0f;
		//if(h>params->p.HL_AlthMax) { //Speed-Weight switching to re-enable the pitch-down at h>hMax.
			dh_rel = limit2((h-params->p.HL_AlthMax)/dh_offset_spdweight_hmax,1.0f,0.0f);
			adapted_spd_weight = interp1_lin(dh_rel, 2.0f,params->p.speed_weight);
		//}
		//else { //Speed-Weight switching to allow some pitch corrections to better keep the altitude around href
		//	dh_rel = limit2((h-hRef_t)/dh_offset_spdweight_href,1.0f,0.0f);
		//	adapted_spd_weight = interp1_lin(dh_rel, params->p.speed_weight,2.0f);
		//}

		tecs.set_speed_weight(adapted_spd_weight);

		//Also set all other parameters for efficient/low-disturbance throttle actuation
		tecs.set_roll_throttle_compensation(0.0f);
		tecs.set_heightrate_p(0.0f);

		// Spoiler activation (With some simple hysteresis)
		if(h>params->p.HL_AlthMax+dh_offset_spoilers && bSpoilerAltExceeded==false) bSpoilerAltExceeded=true;
		if(h<params->p.HL_AlthMax+dh_offset_spoilers-dh_offset_spoilers_hysteresis && bSpoilerAltExceeded==true) bSpoilerAltExceeded=false;
		bEngageSpoilers=bSpoilerAltExceeded;
	}

	if(params->p.ASLC_DEBUG == 29) printf("h:%.1f hMax:%.1f sw:%.2f r2t:%.2f hrp:%.2f bSpoil:%d\n",h,params->p.HL_AlthMax,tecs.get_speed_weight(),tecs.get_roll_throttle_compensation(), tecs.get_heightrate_p(),bEngageSpoilers);

	return 0;
}
