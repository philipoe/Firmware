
#include "subs.h"
#include <string.h>
#include <systemlib/err.h> //Debug only
#include <math.h>
#include <drivers/drv_hrt.h>
#include <poll.h>

//**********************************************************************
//*** Constructors / Destructors
//**********************************************************************
subscriptions::subscriptions(void)
{
	init();
}
subscriptions::~subscriptions(void)
{

}

//**********************************************************************
//*** Functions
//**********************************************************************

int subscriptions::init(void)
{
	memset(&vstatus, 0, sizeof(vstatus));
	memset(&att, 0, sizeof(att));
	//memset(&att_sp, 0, sizeof(att_sp));
	//memset(&rates_sp, 0, sizeof(rates_sp));
	memset(&global_pos, 0, sizeof(global_pos));
	memset(&position_setpoint_triplet, 0, sizeof(position_setpoint_triplet));
	memset(&manual_sp, 0, sizeof(manual_sp));
	memset(&actuators, 0, sizeof(actuators));
	memset(&param_update, 0, sizeof(param_update));
	memset(&sensors, 0, sizeof(sensors));
	memset(&ekf, 0, sizeof(ekf));
	memset(&home_pos, 0, sizeof(home_pos));
	memset(&airspeed, 0, sizeof(airspeed));

	//Inputs
	/* subscribe */
	att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	//att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	position_setpoint_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	//vcontrol_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	param_update_sub = orb_subscribe(ORB_ID(parameter_update));
	sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
	ekf_sub = orb_subscribe(ORB_ID(state_estimator_EKF_parameters));
	home_pos_sub = orb_subscribe(ORB_ID(home_position));
	airspeed_sub = orb_subscribe(ORB_ID(airspeed));

	//Outputs
	/* publish actuator controls */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}
	actuators_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
	aslctrl_params_pub = orb_advertise(ORB_ID(aslctrl_parameters), &aslctrl_params);
	ctrl_data_pub = orb_advertise(ORB_ID(aslctrl_data), &ctrl_data);

	//Set some default values
	fds.fd=att_sub;
	fds.events=POLLIN;
	return 0;
}

int subscriptions::get_inputs(void)
{
	// Wait for attitude updates from PX4.
	// ATTENTION: this limits the ASLCTRL execution frequency to this attitude-update-frequency!
	// TODO IMPORTANT: Recheck this, because this is highly uneven! Have to use orb_set_interval before for sure!!
	if(poll(&(fds), 1, 5000) <= 0) {
		return -1;
	}

	orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
	//orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub, &att_sp);
	orb_copy(ORB_ID(vehicle_global_position), global_pos_sub, &global_pos);
	orb_copy(ORB_ID(position_setpoint_triplet), position_setpoint_triplet_sub, &position_setpoint_triplet);
	orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, &manual_sp);
	//orb_copy(ORB_ID(vehicle_control_mode), vcontrol_sub, &vcontrol);
	orb_copy(ORB_ID(sensor_combined), sensors_sub, &sensors);
	orb_copy(ORB_ID(state_estimator_EKF_parameters), ekf_sub, &ekf);
	orb_copy(ORB_ID(home_position), home_pos_sub, &home_pos);
	orb_copy(ORB_ID(airspeed), airspeed_sub, &airspeed);

	vehicle_status_s temp=vstatus;
	orb_copy(ORB_ID(vehicle_status), vstatus_sub, &vstatus);
	if((vstatus.main_state != temp.main_state) && !vstatus.rc_signal_lost)
	{
		warnx("State and/or Mode changed!");
		warnx("MainState/NavState: (%d/%d). Before:(%d/%d)",vstatus.main_state, vstatus.nav_state,temp.main_state, temp.nav_state);
		//warnx("Enabled control flags : %d/%d/%d/%d (pos/att/rate/man)\n",vcontrol.flag_control_position_enabled,
		//		vcontrol.flag_control_attitude_enabled,vcontrol.flag_control_rates_enabled,vcontrol.flag_control_manual_enabled);
	}

	return 0;
}

bool subscriptions::check_aslctrl_params_updated(void)
{
	// This actually checks whether ANY of the params has been updated, not only aslctrl parameters
	// However, this is still better than stupidly updating all parameters every loop.

	bool ParamsUpdated=false;
	orb_check(param_update_sub, &ParamsUpdated);

	if(ParamsUpdated>0) {
		orb_copy(ORB_ID(parameter_update), param_update_sub, &param_update);
		return true;
	}
	else return false;
}

int subscriptions::publish_actuator_outputs(void)
{
	/* sanity check and publish actuator outputs */
	if (isfinite(actuators.control[0]) &&
		isfinite(actuators.control[1]) &&
		isfinite(actuators.control[2]) &&
		isfinite(actuators.control[3]) &&
		isfinite(actuators.control[4]) &&
		isfinite(actuators.control[5]))
	{
		orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuators_pub, &actuators);
		return 0;
	}
	else return -1;
}

int subscriptions::publish_aslctrl_params(aslctrl_parameters_s * params)
{
	if(params==NULL) return -1;

	//TODO: Change this, we don't need to store the parameters twice.
	params->timestamp = hrt_absolute_time();
	aslctrl_params = *params;

	orb_publish(ORB_ID(aslctrl_parameters), aslctrl_params_pub, &aslctrl_params);
	return 0;
}

int subscriptions::publish_aslctrl_data(aslctrl_data_s * data)
{
	if(data==NULL) return -1;

	//TODO: Change this, we don't need to store the data twice.
	data->timestamp=hrt_absolute_time();
	ctrl_data = *data;

	orb_publish(ORB_ID(aslctrl_data), ctrl_data_pub, &ctrl_data);
	return 0;
}

