#ifndef SUBS_H
#define SUBS_H

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include "params.h"
#include <uORB/topics/aslctrl_data.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/state_estimator_EKF_parameters.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/airspeed.h>


#include <poll.h>
#include "params.h"

//**********************************************************************
//*** Class definition for subscriptions
//**********************************************************************

class subscriptions
{
public:
	subscriptions(void);
	~subscriptions(void);

	int init(void);
	int get_inputs(void);																			// get subscriptions
	int publish_actuator_outputs(void);																// publish actuator outputs
	bool check_aslctrl_params_updated(void);														// check (&get) updated parameters
	int publish_aslctrl_params(aslctrl_parameters_s * params);		// publish aslctrl parameters
	int publish_aslctrl_data();

public:
	//Input
	struct vehicle_attitude_s att;
//	struct vehicle_attitude_setpoint_s att_sp;
//	struct vehicle_rates_setpoint_s rates_sp;
	struct vehicle_global_position_s global_pos;
	struct position_setpoint_triplet_s position_setpoint_triplet;
	struct manual_control_setpoint_s manual_sp;
	struct vehicle_status_s vstatus;
//	struct vehicle_control_mode_s vcontrol;
	struct parameter_update_s param_update;
	struct sensor_combined_s sensors;
	struct state_estimator_EKF_parameters_s ekf;
	struct home_position_s home_pos;
	struct airspeed_s airspeed;

	//Output
	struct actuator_controls_s actuators;
	struct aslctrl_parameters_s aslctrl_params;
	struct aslctrl_data_s aslctrl_data;

private:
	//Input
	int att_sub;
//	int att_sp_sub;
//	int rates_sp_sub;
	int global_pos_sub;
	int position_setpoint_triplet_sub;
	int manual_sp_sub;
	int vstatus_sub;
//	int vcontrol_sub;
	int param_update_sub;
	int sensors_sub;
	int ekf_sub;
	int home_pos_sub;
	int airspeed_sub;

	//Output
	orb_advert_t actuators_pub, aslctrl_params_pub, aslctrl_data_pub;

	struct pollfd fds;
};

#endif /*SUBS_H*/
