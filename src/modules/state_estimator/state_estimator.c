/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author:
 *           Lorenz Meier <lm@inf.ethz.ch>
 *   		 Stefan Leutenegger <stefan.leutenegger@mavt.ethz.ch>
 *           Amir Melzer <amir.melzer@mavt.ethz.ch>
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

/*
 * @file state_estimator.c
 *
 * State space Estimation.
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/state_estimator_EKF_parameters.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/home_position.h>
#include <drivers/drv_hrt.h>

#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include "state_estimator_params.h"

#include "codegen_state_estimator/HyEst_initialize.h"
#include "codegen_state_estimator/propagate.h"
#include "codegen_state_estimator/updateCompass2.h"
#include "codegen_state_estimator/updatePosition.h"
#include "codegen_state_estimator/updateVelNed.h"
#include "codegen_state_estimator/magField.h"
#include "codegen_state_estimator/updatePressures_all.h"
#include "codegen_state_estimator/quat2rpy.h"
#include "codegen_state_estimator/getAirplane.h"
#include "codegen_state_estimator/initStates.h"

#include "codegen_state_estimator/HyEst_rtwutil.h"
#include "codegen_state_estimator/HyEst_types.h"


#define CONV_CELSIUS_KELVIN 273.15f
#define DEG2RAD 0.01745329251994329577l
#define RAD2DEG 57.2957795130823208767l
#define UERE_H 1.5f 									/* User Equivalent Range Error (horizontal) */
#define UERE_V 2.0f 									/* User Equivalent Range Error (vertical) 	*/


__EXPORT int state_estimator_main(int argc, char *argv[]);

static unsigned int loop_interval_alarm = 20000;	/* loop interval in microseconds */

static float dt = 0.01f;

/* state vector x has the following states:  [p(lat*1e7,lon*1e7,h)||q_NS||Vn,Ve,Vd||b_gx,b_gy,b_gz||b_ax,b_ay,b_az||QFF||Wn,We,Wd||K] */

/* Covariance matrix */
static float P_apost[400];

/* output Euler angles */
static float euler[3] 			= {0.0f, 0.0f, 0.0f};

/* init: identity matrix */
static float Rot_matrix[9] 		= {1.0f,  0.0f,  0.0f,
								   0.0f,  1.0f,  0.0f,
								   0.0f,  0.0f,  1.0f};

static float gyro_b[3]		  	= {0.0f,0.0f,0.0f};					/* Gyroscope measurements in body frame					*/
static float accel_b[3]		  	= {0.0f,0.0f,0.0f};					/* Accelerometer measurements in body frame 			*/
static float mag_b[3]			= {0.0f,0.0f,0.0f};					/* Magnetometer measurements in body frame 				*/
static float baro_b			  	= 0.0f;								/* Static barometer measurements  						*/
static float dbaro_b		  	= 0.0f;								/* Differential barometer measurements 					*/
static float amb_temp_b		  	= 0.0f;								/* Ambient temperature measurements						*/

static float geoid_separation 	= 48.6836f;							/* offset between WGS-84 and MSL [m] 					*/

static boolean_T p_stat_valid 	= false;
static boolean_T on_ground 		= false;
static boolean_T aero_valid 	= false;

double gps_pos[3];									     			/* GPS position: Lat [1E7 deg], Lon [1E7 deg], Alt (above MSL) */
static float gps_vel[3]			= {0.0f,0.0f,0.0f};					/* GPS ground velocity [m/s]: north, east, down			*/
static float GPS_H_acc 			= 1.0f;								/* GPS horizontal accuracy 								*/
static float GPS_V_acc 			= 100.0f;							/* GPS vertical accuracy 								*/
static bool  GPS_outage 		= false; 							/* GPS outage flag (use true on GPS out)				*/

static float sigma_h_sq 		= 1.0f;								/* GPS horizontal accuracy 								*/
static float sigma_v_sq 		= 100.0f;							/* GPS vertical accuracy 								*/
																	/* GPS position measurement covariance matrix [m^2]		*/
																	/* GPS horizontal 5 sigma[m], vertical sigma 10[m]		*/
static float R_m[9] 			= {5.0f,  0.0f, 0.0f,
								   0.0f,  5.0f, 0.0f,
								   0.0f,  0.0f, 100.0f};
																	/* GPS velocity measurement covariance matrix [m^2/s^2]	*/
																	/* GPS velocity accuracy 0.5 sigma [m/sec]				*/
																	/* (R_mvel increased to 0.5 sigma (from 0.2 sigma))     */
static float R_mvel[9]			= {0.25f, 0.0f,  0.0f,
							   	   0.0f,  0.25f, 0.0f,
							   	   0.0f,  0.0f,  0.25f};

static float mag_date 			= 2015.6f;							/* Decimal date for calculating the earth's magnetic field */
static float TAS[3] 		  	= {0.0f,0.0f,0.0f};					/* True air speed */
static float B_N[3] 		  	= {0.0f,0.0f,0.0f};					/* Earth's magnetic field vector [nT] */

static float wind_NE[2]			= {0.0f,0.0f};						/* Wind (north, east) used for propagation				*/

static int8_t PlaneIndex 		= 0;								/* PlaneIndex (set by parameters )  */
static boolean_T airplaneExists = false;							/* PlaneIndex Exists				*/

static bool thread_should_exit 	= false;							/**< Deamon exit flag */
static bool thread_running 		= false;							/**< Deamon status flag */
static int state_estimator_task;									/**< Handle of deamon task / thread */

static float _home_alt 			= 0.0f;								/* Altitude of the home position    */

static uint64_t offset_integration_time = 10000000LL;		        /* initialization time for the sensors in [usec] */

/* Mainloop of attitude_estimator */
int state_estimator_thread_main(int argc, char *argv[]);

/* Print the correct usage */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: state_estimator {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

int state_estimator_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("state_estimator already running\n");
			exit(0);
		}

		thread_should_exit = false;
		state_estimator_task = task_spawn_cmd("state_estimator",
												SCHED_DEFAULT,
												SCHED_PRIORITY_MAX - 5,
												7200,
												state_estimator_thread_main,
												(argv) ? (char *const * )&argv[2] : (char *const * )NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("state_estimator app is running\n");
		} else {
			printf("state_estimator app not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}


/*
 * State estimator main function.
 *
 * @param argc number of commandline arguments (plus command name)
 * @param argv strings containing the arguments
 */
int state_estimator_thread_main(int argc, char *argv[])
{

	/* get some state estimator configuration parameters from the start script */
	int ch;
	while ((ch = getopt(argc, argv, "d:")) != EOF) {
		switch (ch) {
		case 'd':
			{
				unsigned s = strtoul(optarg, NULL, 10);
				if (s > 3){
					errx(1, "Error wrong value for the dynamic model (%d), out of range (0..2)...exiting.\n ", s);
				}
				else
					PlaneIndex = (int8_t) s;
			}
			break;
		default:
			usage("unrecognized flag");
			errx(1, "exiting.");
		}
	}

	/* print text */
	printf("State estimator using Extended Kalman Filter initializing...\n");
	fflush(stdout);

	uint8_t overloadcounter = 0;

	/* Initialize filter */
	HyEst_initialize();

	/* store start time to guard against too slow update rates */
	uint64_t last_run = hrt_absolute_time();

	struct sensor_combined_s *raw;
	raw = (struct sensor_combined_s *) calloc(1,sizeof(struct sensor_combined_s));
	//memset(&raw, 0, sizeof(raw));
	struct vehicle_attitude_s *att;
	att = (struct vehicle_attitude_s *) calloc(1,sizeof(struct vehicle_attitude_s));
	//memset(&att, 0, sizeof(att));
	struct vehicle_control_mode_s *control_mode;
	control_mode = (struct vehicle_control_mode_s *) calloc(1,sizeof(struct vehicle_control_mode_s));
	//memset(&control_mode, 0, sizeof(control_mode));
	struct vehicle_global_position_s *vehicle_global_pos;
	vehicle_global_pos = (struct vehicle_global_position_s *) calloc(1,sizeof(struct vehicle_global_position_s));
	//memset(&vehicle_global_pos, 0, sizeof(vehicle_global_pos));
	struct state_estimator_EKF_parameters_s *ekf_param;
	ekf_param = (struct state_estimator_EKF_parameters_s *) calloc(1,sizeof(struct state_estimator_EKF_parameters_s));
	//memset(&ekf_param, 0, sizeof(ekf_param));

	/* initialize parameter handles */
	struct state_estimator_params ekf_params;
	struct state_estimator_param_handles ekf_param_handles;

	parameters_init(&ekf_param_handles);
	parameters_update(&ekf_param_handles, &ekf_params);									/* Update EKF parameters for the parameter file */

	mag_date   = (float)ekf_params.mag_date_year + ((float)ekf_params.mag_date_month)/10;

	uint64_t last_measurement = 0;


	/* subscribe to home position */
	int _home_sub = orb_subscribe(ORB_ID(home_position));

	/* subscribe to raw data */
	int sub_raw = orb_subscribe(ORB_ID(sensor_combined));

	/* rate-limit raw data updates to 100Hz */
	//orb_set_interval(sub_raw, 8);
	orb_set_interval(sub_raw, 4);

	/* subscribe to param changes */
	int sub_params = orb_subscribe(ORB_ID(parameter_update));

	/* subscribe to system state*/
	int sub_control_mode = orb_subscribe(ORB_ID(vehicle_control_mode));

	/* subscribe state space */
	states_T *x_state;
	x_state = (states_T *) malloc(sizeof(states_T));

	x_state->p[0] 	= 0.0l;
	x_state->p[1] 	= 0.0l;
	x_state->p[2] 	= 0.0l;
	x_state->q_NS[0] = 0.0f;
	x_state->q_NS[1] = 0.0f;
	x_state->q_NS[2] = 0.0f;
	x_state->q_NS[3] = 0.0f;
	x_state->v_N[0] 	= 0.0f;
	x_state->v_N[1]	= 0.0f;
	x_state->v_N[2]	= 0.0f;
	x_state->b_g[0] 	= 0.0f;
	x_state->b_g[1] 	= 0.0f;
	x_state->b_g[2]  = 0.0f;
	x_state->b_a[0] 	= 0.0f;
	x_state->b_a[1] 	= 0.0f;
	x_state->b_a[2]	= 0.0f;
	x_state->QFF 	= 0.0f;
	x_state->w[0] 	= 0.0f;
	x_state->w[1] 	= 0.0f;
	x_state->w[2] 	= 0.0f;
	x_state->K 		= 0.0f;

	/* subscribe plane type */
	airplane_T airplaneUsed;

	getAirplane(PlaneIndex, &airplaneUsed, &airplaneExists);
	if (airplaneExists){
		 switch (PlaneIndex) {
		   case 0:
			   printf("State estimator uses SenseSoar airplane dynamic model\n\n");
			   break;
		   case 1:
			   printf("State estimator uses EasyGlider airplane dynamic model\n\n");
			   break;
		   case 2:
			   printf("State estimator uses AtlantikSolar airplane dynamic model\n\n");
			   break;
		   case 3:
			   printf("State estimator uses TechPod airplane dynamic model\n\n");
			   break;
		   default:
			   printf("Wrong airplane dynamic model given, State estimator uses SenseSoar airplane dynamic model as default\n\n");
			   break;
		 }
	}
	 else
		   printf("No airplane dynamic model given, State estimator uses SenseSoar airplane dynamic model as default\n\n");


	/* GPS initialization: */

	/* subscribe to vehicle GPS */
	struct vehicle_gps_position_s gps;

	gps.fix_type = 0;

	GPS_outage = true;

	int vehicle_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));

	/* wait until GPS signal turns valid, if not succeed deliver the last valid position  */
	static uint32_t GPS_disappointment_max = 10;												/// <--- increase the GPS_disappointment_max to larger value after debugging !!!!!!
	static uint32_t GPS_disappointment = 0;

	while (gps.fix_type < 3) {
		struct pollfd fds[1] = { {.fd = vehicle_gps_sub, .events = POLLIN} };
		GPS_disappointment++;

		if (poll(fds, 1, 5000)) {
			usleep(5000);													 /* Wait for the GPS update */
			orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);   /* check for valid GPS data */
			GPS_outage = !(gps.fix_type > 2);
			if (GPS_disappointment > GPS_disappointment_max){

				/* get last valid GPS positions and velocity data		*/
				gps_pos[0] = (double)(gps.lat);
				gps_pos[1] = (double)(gps.lon);
				gps_pos[2] = (double)(gps.alt * 1e-3f);

				gps_vel[0]	= (float)(gps.vel_n_m_s);
				gps_vel[1]	= (float)(gps.vel_e_m_s);
				gps_vel[2]	= (float)(gps.vel_d_m_s);

				warnx("WARNING, No valid GPS initialization data, set position to last valid coordinates: %3.6f, %3.6f, %4.6f", (double)(gps_pos[0]*1e-7l), (double)(gps_pos[1]*1e-7l), (double)(gps_pos[2]));
				warnx("WARNING, No valid GPS initialization data, set ground speed to last valid value of: %3.6f, %3.6f, %3.6f", (double)(gps_vel[0]), (double)(gps_vel[1]), (double)(gps_vel[2]));
				warnx("WARNING, Check state estimator condition after first valid GPS data\n");
				break;
			}
		}

		else{
			/* set a default GPS positions and velocity data		*/
			gps_pos[0] = (double)(471909750l);
			gps_pos[1] = (double)(89530328l);
			gps_pos[2] = (double)(407.304l);

			gps_vel[0]	= (float)(0.0f);
			gps_vel[1]	= (float)(0.0f);
			gps_vel[2]	= (float)(0.0f);

			warnx("FATAL ERROR, No GPS receiver installed, set position to a default coordinates: %3.6f, %3.6f, %4.6f", (double)(gps_pos[0]*1e-7l), (double)(gps_pos[1]*1e-7l), (double)(gps_pos[2]));
			warnx("FATAL ERROR, No GPS receiver installed, set ground speed to a default value of: %3.6f, %3.6f, %3.6f", (double)(gps_vel[0]), (double)(gps_vel[1]), (double)(gps_vel[2]));
			break;
		}
	}

	if (!GPS_outage){
		/* get GPS value for first initialization */
		orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
		gps_pos[0] = (double)(gps.lat);
		gps_pos[1] = (double)(gps.lon);
		gps_pos[2] = (double)(gps.alt * 1e-3f);
	}

	/* advertise attitude */
	orb_advert_t pub_att = orb_advertise(ORB_ID(vehicle_attitude), att);

	/* advertise vehicle global position */
	orb_advert_t pub_vehicle_global_pos = orb_advertise(ORB_ID(vehicle_global_position), vehicle_global_pos);

	/* advertise EKF parameters */
	orb_advert_t pub_ekf_param = orb_advertise(ORB_ID(state_estimator_EKF_parameters), ekf_param);

	int loopcounter = 0;

	thread_running = true;

	float sensor_update_hz[8] = {0, 0, 0, 0, 0, 0, 0, 0};			/* update hz fields: 	  [gyro||accel||mag||baro||dbaro||amb_temp||gps_pos||gps_vel] */

	/* check for sensor updates */
	uint64_t sensor_last_timestamp[8] = {0, 0, 0, 0, 0, 0, 0, 0};	/* last timestamp fields: [gyro||accel||mag||baro||dbaro||amb_temp||gps_pos||gps_vel] */

	uint64_t start_time = hrt_absolute_time();
	bool initialized = false;

	volatile float gyro_offsets[3] = {0.0f, 0.0f, 0.0f};
	volatile unsigned offset_count = 0;

	static float mag_initial[3]  = {0.0f, 0.0f, 0.0f};
	volatile uint64_t magnetometer_timestamp_last = 0;
	volatile unsigned mag_initial_count = 0;

	static float dbaro_initial = 0.0f;
	volatile uint64_t dbaro_timestamp_last = 0;
	volatile unsigned dbaro_initial_count = 0;

	/* mag averaging */
	static float mag_average[3]			= {0.0f,0.0f,0.0f};
	volatile unsigned mag_average_count = 0;
	volatile unsigned mag_average_count_limit = 4;

	/* register the perf counter */
	perf_counter_t ekf_loop_perf = perf_alloc(PC_ELAPSED, "state_estimator");

	/* reset update vector fields: [gyro||accel||mag||baro||dbaro||amb_temp||gps_pos||gps_vel] */
	uint8_t update_vect[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	/* Checking HIL mode: */
	orb_copy(ORB_ID(vehicle_control_mode), sub_control_mode, control_mode);
	if (control_mode->flag_system_hil_enabled)
		printf("[state estimator] Info: State Estimator works in HIL mode\n");

	/* Main loop*/
	while (!thread_should_exit) {

		struct pollfd fds[3] = {
				{ .fd = sub_raw,   		.events = POLLIN },										/* Sensors update 	*/
				{ .fd = sub_params, 	.events = POLLIN },										/* Parameter update */
				{ .fd = vehicle_gps_sub,.events = POLLIN }										/* GPS update 		*/
		};

		int ret = poll(fds, 3, 1000);

		if (ret < 0) {
			/* seriously bad - an emergency case*/
		} else if (ret == 0) {
			/* check if we're in HIL - not getting sensor data is fine then */
			orb_copy(ORB_ID(vehicle_control_mode), sub_control_mode, control_mode);
			if (!control_mode->flag_system_hil_enabled) {
				fprintf(stderr, "[state estimator] WARNING: Not getting sensors data - is sensors application running?\n");
			}
		} else {

			/* only update parameters if they changed */
			if (fds[1].revents & POLLIN) {
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), sub_params, &update);

				/* update parameters */
				parameters_update(&ekf_param_handles, &ekf_params);
			}

			/* update GPS measurements if new valid GPS data is available:			 */

			if (fds[2].revents & POLLIN) {
				/* get latest GPS data  */
				orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);

				/* get GPS positions					*/
				gps_pos[0] = (double)(gps.lat);
				gps_pos[1] = (double)(gps.lon);
				gps_pos[2] = (double)(gps.alt * 1e-3f);

				/* get GPS ground velocities 			*/
				gps_vel[0]	= (float)(gps.vel_n_m_s);
				gps_vel[1]	= (float)(gps.vel_e_m_s);
				gps_vel[2]	= (float)(gps.vel_d_m_s);

				/* get GPS dilution of precision (DOP)	*/
				GPS_H_acc	= ((float)(gps.eph)) * UERE_H;			  	 /* GPS HDOP horizontal DOP in [m] */
				GPS_V_acc	= ((float)(gps.epv)) * UERE_V;				 /* GPS VDOP vertical DOP in [m]   */

				GPS_H_acc   = ((GPS_H_acc*GPS_H_acc) < 1.0f) ?  1.0f : GPS_H_acc;     /* protection against GPS_H_acc^2 values lower than 1 */
				sigma_h_sq 	= GPS_H_acc*GPS_H_acc;

				GPS_V_acc   = ((GPS_V_acc*GPS_V_acc) < 1.0f) ? 1.0f : GPS_V_acc;  	 /* protection against GPS_V_acc^2 values lower than 1 */
				sigma_v_sq 	= GPS_V_acc *GPS_V_acc;

				R_m[0]  = sigma_h_sq;
				R_m[4]  = sigma_h_sq;
				R_m[8]  = sigma_v_sq;

				sensor_update_hz[6] = (gps.timestamp_position > 0) ? (1e6f / (gps.timestamp_position - sensor_last_timestamp[6])) : 0.0f;
				sensor_last_timestamp[6] = gps.timestamp_position;

				sensor_update_hz[7] = (gps.timestamp_velocity > 0) ? (1e6f / (gps.timestamp_velocity - sensor_last_timestamp[7])) : 0.0f;
				sensor_last_timestamp[7] = gps.timestamp_velocity;

				/* Always read the GPS data and make an update only if there is a 3d fix (in this way the last position and velocity values are kept)*/
				if (gps.fix_type > 2) {
					/* reset GPS outage flag				*/
					GPS_outage = !(gps.fix_type > 2);
					geoid_separation = ((float)(gps.alt_ellipsoid - gps.alt)) * 1e-3f;
					/*update the GPS position and velocity update vector */
					update_vect[6] = 1;
					update_vect[7] = gps.vel_ned_valid;
				}

				/* if latest GPS data is not valid set GPS outage flag*/
				else{
					GPS_outage = !(gps.fix_type > 2);
				}
			}

			/* check if there is any new sensor data */
			if ((fds[0].revents & POLLIN)) {

				/* get latest measurements */
				orb_copy(ORB_ID(sensor_combined), sub_raw, raw);

				/* Initialization of the state estimator */
				if (!initialized) {
					gyro_offsets[0] += raw->gyro_rad_s[0];
					gyro_offsets[1] += raw->gyro_rad_s[1];
					gyro_offsets[2] += raw->gyro_rad_s[2];

					/* average magnetometer measurements */
					if (raw->magnetometer_timestamp > magnetometer_timestamp_last){
						mag_initial[0] += (raw->magnetometer_ga[0])*100.0f;			/*convert magnetometer measurements from [gauss] -> [uT] */
						mag_initial[1] += (raw->magnetometer_ga[1])*100.0f;			/*convert magnetometer measurements from [gauss] -> [uT] */
						mag_initial[2] += (raw->magnetometer_ga[2])*100.0f;			/*convert magnetometer measurements from [gauss] -> [uT] */
						magnetometer_timestamp_last = raw->magnetometer_timestamp;
						mag_initial_count ++;
					}

					/* average dbaro measurements */
					if (raw->differential_pressure_timestamp > dbaro_timestamp_last){
						dbaro_initial += (raw->differential_pressure_pa);
						dbaro_timestamp_last = raw->differential_pressure_timestamp;
						dbaro_initial_count ++;
					}

					offset_count++;

					if (hrt_absolute_time() - start_time > offset_integration_time) {
						initialized = true;
						gyro_offsets[0] /= offset_count;
						gyro_offsets[1] /= offset_count;
						gyro_offsets[2] /= offset_count;

						/* average magnetometer measurements */
						mag_initial[0] /= mag_initial_count;
						mag_initial[1] /= mag_initial_count;
						mag_initial[2] /= mag_initial_count;

						/* average dbaro measurements */
						dbaro_initial /= dbaro_initial_count;
					}

				/* Normal loop of the state estimator */
				} else {

					perf_begin(ekf_loop_perf);

					/* Protect against large dt after initialization */
					(last_measurement == 0) ? (last_measurement = raw->timestamp - 10000) : 0;

					/* Calculate data time difference in seconds */
					dt = (float) (raw->timestamp - last_measurement) / 1000000.0f;
					last_measurement = raw->timestamp;

					/* check for new gyroscope measurements */
					if (sensor_last_timestamp[0] != raw->timestamp) {
						update_vect[0] = 1;
						sensor_update_hz[0] = (raw->timestamp > 0) ? (1e6f / (float) (raw->timestamp - sensor_last_timestamp[0])) : 0.0f;
						sensor_last_timestamp[0] = raw->timestamp;
					}
					gyro_b[0] =  raw->gyro_rad_s[0];
					gyro_b[1] =  raw->gyro_rad_s[1];
					gyro_b[2] =  raw->gyro_rad_s[2];

					/* check for new accelerometer measurements */
					if (sensor_last_timestamp[1] != raw->accelerometer_timestamp) {
						update_vect[1] = 1;
						sensor_update_hz[1] = (raw->accelerometer_timestamp > 0) ? (1e6f / (float) (raw->accelerometer_timestamp - sensor_last_timestamp[1])) : 0.0f;
						sensor_last_timestamp[1] = raw->accelerometer_timestamp;
					}
					accel_b[0] = raw->accelerometer_m_s2[0];
					accel_b[1] = raw->accelerometer_m_s2[1];
					accel_b[2] = raw->accelerometer_m_s2[2];

					/* check for new magnetometer measurements */
					if (sensor_last_timestamp[2] != raw->magnetometer_timestamp) {
						/* average magnetometer measurements */
						mag_average[0] += (raw->magnetometer_ga[0]);
						mag_average[1] += (raw->magnetometer_ga[1]);
						mag_average[2] += (raw->magnetometer_ga[2]);
						mag_average_count ++;

						if (mag_average_count > mag_average_count_limit){
							update_vect[2] = 1;
							sensor_update_hz[2] = (raw->magnetometer_timestamp > 0) ? (1e6f / (float) (raw->magnetometer_timestamp - sensor_last_timestamp[2]) / (float) mag_average_count) : 0.0f;

							mag_b[0] = (mag_average[0] / mag_average_count)*100.0f;        /*convert magnetometer measurements from [gauss] -> [uT] */
							mag_b[1] = (mag_average[1] / mag_average_count)*100.0f;        /*convert magnetometer measurements from [gauss] -> [uT] */
							mag_b[2] = (mag_average[2] / mag_average_count)*100.0f;        /*convert magnetometer measurements from [gauss] -> [uT] */

							mag_average[0] = 0.0f;
							mag_average[1] = 0.0f;
							mag_average[2] = 0.0f;
							mag_average_count = 0;
						}
						sensor_last_timestamp[2] = raw->magnetometer_timestamp;

					}

					/* check for new baro measurements */
					if (sensor_last_timestamp[3] != raw->baro_timestamp) {
						update_vect[3] = 1;
						sensor_update_hz[3] = (raw->baro_timestamp > 0) ? (1e6f / (float) (raw->baro_timestamp - sensor_last_timestamp[3])) : 0.0f;
						sensor_last_timestamp[3] = raw->baro_timestamp;
					}
					baro_b = raw->baro_pres_mbar;

					/* check for new dbaro measurements */
					if (sensor_last_timestamp[4] != raw->differential_pressure_timestamp) {
						update_vect[4] = 1;
						sensor_update_hz[4] = (raw->differential_pressure_timestamp > 0) ? (1e6f / (float) (raw->differential_pressure_timestamp - sensor_last_timestamp[4])) : 0.0f;
						sensor_last_timestamp[4] = raw->differential_pressure_timestamp;
					}
					dbaro_b = (raw->differential_pressure_pa>0) ? raw->differential_pressure_pa : 0.0f;				/* take only the positive values of the dbaro 			 */
					//dbaro_b = raw->differential_pressure_pa - dbaro_initial;										/* removed! (remove dbaro constant measurement bias) 	 */
					//dbaro_b = (dbaro_b>0) ? dbaro_b : 0.0f;														/* removed! (take only the positive values of the dbaro) */

					/* check for new amb_temp measurements */
					if (sensor_last_timestamp[5] != raw->amb_temp_timestamp) {
						update_vect[5] = 1;
						sensor_update_hz[5] = (raw->amb_temp_timestamp > 0) ? (1e6f / (float) (raw->amb_temp_timestamp - sensor_last_timestamp[5])) : 0.0f;
						sensor_last_timestamp[5] = raw->amb_temp_timestamp;
					}
					amb_temp_b = raw->amb_temp_celcius + CONV_CELSIUS_KELVIN;

					uint64_t now = hrt_absolute_time();
					unsigned int time_elapsed = now - last_run;
					last_run = now;

					if (time_elapsed > loop_interval_alarm && !control_mode->flag_system_hil_enabled) {
						//TODO: add warning, cpu overload here
						 if (overloadcounter > 50) {
						 	warnx("CPU OVERLOAD DETECTED IN STATE ESTIMATOR (%u > %u)", time_elapsed, loop_interval_alarm);
						 	overloadcounter = 0;
						 }
						overloadcounter++;
					}

					static bool const_initialized = false;

					/* initialize with good values once there are reasonable dt values */
					/* runs only once at the beginning */
					if (!const_initialized && dt < 0.05f && dt > 0.005f)
					{
						/*State initialization*/

						/* magField vector */
					    double pos_vector[3];
						pos_vector[0] = gps_pos[0]*1e-7l*DEG2RAD;
						pos_vector[1] = gps_pos[1]*1e-7l*DEG2RAD;
						pos_vector[2] = gps_pos[2];

#if 1					/* Temporary commented out. Constant values of B_N were taken (for the position: 47.1909750, 8.9530328, 407.304) */

						/* calculates the Earth's magnetic field and the secular variation */
						magField(pos_vector, mag_date,B_N);
#else
						B_N[0] = 21.571220f;						// B_N of Zurich
						B_N[1] = 0.70266867f;						// B_N of Zurich
						B_N[2] = 42.614651f;						// B_N of Zurich
#endif
						initStates(pos_vector, accel_b, gyro_b,TAS, mag_initial, B_N, x_state, P_apost); 			/* Init states and variances*/
						x_state->K  = airplaneUsed.K;

						/* initialization of gyro offset states (if initialization is made in the air the initial state should be zero - fix it!!!!!!!) */
						x_state->b_g[0]  = gyro_offsets[0];
						x_state->b_g[1]  = gyro_offsets[1];
						x_state->b_g[2]  = gyro_offsets[2];

						/* Display initial quaternion (in Euler angle representation ) */
						quat2rpy(x_state->q_NS,euler);
						printf("Initial Euler rotation angles (r,p,y): %3.6f, %3.6f, %3.6f \n", (double) euler[0], (double) euler[1], (double) euler[2]);

						const_initialized = true;
					}

					/* Execute the state estimator only after initialized */
					if (!const_initialized) {
						continue;
					}

					/* perform a state propagation */
					if ((update_vect[0] == 1) && (update_vect[1] == 1)) {
						float propagate_vector[6];							  							/* propagation vector */
						propagate_vector[0] = gyro_b[0];
						propagate_vector[1] = gyro_b[1];
						propagate_vector[2] = gyro_b[2];
						propagate_vector[3] = accel_b[0];
						propagate_vector[4] = accel_b[1];
						propagate_vector[5] = accel_b[2];
						propagate(x_state, P_apost, propagate_vector, dt, GPS_outage, wind_NE);
					}

					/* Kalman filter updates:  */

					/* pressure sensors update*/
					if (update_vect[4] == 1) {
						updatePressures_all(x_state, P_apost, baro_b/10.0f, dbaro_b/1000.0f, amb_temp_b, geoid_separation, accel_b, airplaneUsed, GPS_outage, GPS_outage, true, &p_stat_valid, &on_ground, &aero_valid);
					}

					/* Compass update*/
					if (update_vect[2] == 1) {
						updateCompass2(x_state, P_apost, mag_b, B_N, accel_b, GPS_outage, GPS_outage, true);
					}

					/* GPS position update*/
					if (update_vect[6] == 1) {
						gps_pos[0]*=1e-7l*DEG2RAD;														/*Convert to GPS LAT position to rad */
						gps_pos[1]*=1e-7l*DEG2RAD;														/*Convert to GPS LON position to rad */
						wind_NE[0] = 0.99f*wind_NE[0] + 0.01f*x_state->w[0];								/*Correction for the wind propagation*/
						wind_NE[1] = 0.99f*wind_NE[1] + 0.01f*x_state->w[1];								/*Correction for the wind propagation*/
						updatePosition(x_state, P_apost, gps_pos, R_m, GPS_outage, GPS_outage, true);
					}

					/* GPS velocity update*/
					if (update_vect[7] == 1) {
						updateVelNed(x_state, P_apost, gps_vel, R_mvel, GPS_outage, GPS_outage, true);
					}


					/* converts quaternion to euler angles representation*/
					quat2rpy(x_state->q_NS,euler);

					/* swap values for next iteration, check for fatal inputs */
					if (isfinite(euler[0]) && isfinite(euler[1]) && isfinite(euler[2])) {

					} else {
						/* There is an error on the input data or numerical failure (the output is invalid), skip this iteration */
						continue;
					}

					//if (last_data > 0 && raw->timestamp - last_data > 12000) printf("[state estimator] sensor data missed! (%llu)\n", raw->timestamp - last_data);    // temporary comment
					//last_data = raw->timestamp;

					/* send estimator data data out */
					att->timestamp = raw->timestamp;
					att->roll  = euler[0];
					att->pitch = euler[1];
					att->yaw   = euler[2];

#if 0
					printf("State estimator Euler angles (dt,r,p,y):%llu, %3.6f, %3.6f, %3.6f  |", att->timestamp, (double) att->roll, (double) att->pitch, (double) att->yaw );
					printf("Update: ");
					(update_vect[2]== 1)? (printf("M")):(printf("_")) ;
					(update_vect[4]== 1)? (printf("D")):(printf("_")) ;
					(update_vect[6]== 1)? (printf("P")):(printf("_")) ;
					(update_vect[7]== 1)? (printf("V")):(printf("_")) ;

					printf("|States:[ %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f  ]|",
								(double)x_state->p[0],(double)x_state->p[1],(double)x_state->p[2],
								(double)x_state->q_NS[0],(double)x_state->q_NS[1],(double)x_state->q_NS[2],(double)x_state->q_NS[3],
								(double)x_state->v_N[0],(double)x_state->v_N[1],(double)x_state->v_N[2],
								(double)x_state->b_a[0],(double)x_state->b_a[1],(double)x_state->b_a[2],
								(double)x_state->b_g[0],(double)x_state->b_g[1],(double)x_state->b_g[2],
								(double)x_state->QFF,
								(double)x_state->w[0],(double)x_state->w[1],(double)x_state->w[2],
								(double)x_state->K
							);

					//printf("|Variance:[ %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f ]|",
					//							P_apost[0], P_apost[21], P_apost[42], P_apost[63], P_apost[84], P_apost[105], P_apost[126], P_apost[147], P_apost[168], P_apost[189],
					//							P_apost[210], P_apost[231], P_apost[252], P_apost[273], P_apost[294], P_apost[315], P_apost[336], P_apost[357], P_apost[378], P_apost[399]);

					printf("\n");

					//printf("Update rate: %3.6f| %3.6f| %3.6f| %3.6f| %3.6f| %3.6f| %3.6f| %3.6f| , dt = %3.6f |\n",(double) sensor_update_hz[0],(double) sensor_update_hz[1],(double) sensor_update_hz[2],(double) sensor_update_hz[3],(double) sensor_update_hz[4],(double) sensor_update_hz[5],(double) sensor_update_hz[6],(double) sensor_update_hz[7],(double) dt);  /// delete!!!!!!!!!!

					//printf("dt = %3.6f |\n",(double) dt);  /// delete!!!!!!!!!!
					printf("dt = %3.6f | %3.6f\n",(double) sensor_update_hz[6], (double) update_vect[6]);  /// delete!!!!!!!!!!

					printf("%d %d %d %d %d %d %d %d\n",update_vect[0],update_vect[1],update_vect[2],update_vect[3],update_vect[4],update_vect[5],update_vect[6],update_vect[7]);  /// delete!!!!!!!!!!
#endif
					att->rollspeed  = raw->gyro_rad_s[0] - x_state->b_g[0];			/* x angular velocity with gyro bias deduction */
					att->pitchspeed = raw->gyro_rad_s[1] - x_state->b_g[1];			/* y angular velocity with gyro bias deduction */
					att->yawspeed   = raw->gyro_rad_s[2] - x_state->b_g[2];			/* z angular velocity with gyro bias deduction */

					att->rollacc    = raw->accelerometer_m_s2[0] - x_state->b_a[0];	/* x acceleration with bias deduction */    // <<-- check it - not an angular acceleration!
					att->pitchacc   = raw->accelerometer_m_s2[1] - x_state->b_a[1];	/* y acceleration with bias deduction */    // <<-- check it - not an angular acceleration!
					att->yawacc     = raw->accelerometer_m_s2[2] - x_state->b_a[2];	/* z acceleration with bias deduction */    // <<-- check it - not an angular acceleration!

					/* copy offsets */
					memcpy(&(att->rate_offsets), &(x_state->b_g[3]), sizeof(att->rate_offsets));

					/* copy rotation matrix */
					memcpy(&(att->R), Rot_matrix, sizeof(Rot_matrix));
					att->R_valid = true;

					if (isfinite(att->roll) && isfinite(att->pitch) && isfinite(att->yaw)) {
						orb_publish(ORB_ID(vehicle_attitude), pub_att, att);
					} else {
						warnx("NaN in roll/pitch/yaw estimate!");
					}


					/* publish EKF parameters */

					ekf_param->timestamp =  raw->timestamp;

					if (isfinite(x_state->p[0]) && isfinite(x_state->p[1]) && isfinite(x_state->p[2])) {
						memcpy(&(ekf_param->x_p), x_state->p, sizeof(x_state->p));
					} else {
						warnx("NaN in position estimate!");
						continue;
					}

					if (isfinite(x_state->q_NS[0]) && isfinite(x_state->q_NS[1]) && isfinite(x_state->q_NS[2]) && isfinite(x_state->q_NS[3])) {
						memcpy(&(ekf_param->x_q_NS), x_state->q_NS, sizeof(x_state->q_NS));
					} else {
						warnx("NaN in angular estimate!");
						continue;
					}

					if (isfinite(x_state->v_N[0]) && isfinite(x_state->v_N[1]) && isfinite(x_state->v_N[2])) {
						memcpy(&(ekf_param->x_v_N), x_state->v_N, sizeof(x_state->v_N));
					} else {
						warnx("NaN in velocity estimate!");
						continue;
					}

					if (isfinite(x_state->b_g[0]) && isfinite(x_state->b_g[1]) && isfinite(x_state->b_g[2])) {
						memcpy(&(ekf_param->x_b_g), x_state->b_g, sizeof(x_state->b_g));
					} else {
						warnx("NaN in gyroscope bias estimate!");
						continue;
					}

					if (isfinite(x_state->b_a[0]) && isfinite(x_state->b_a[1]) && isfinite(x_state->b_a[2])) {
						memcpy(&(ekf_param->x_b_a), x_state->b_a, sizeof(x_state->b_a));
					} else {
						warnx("NaN in accelerometer bias estimate!");
						continue;
					}

					if (isfinite(x_state->w[0]) && isfinite(x_state->w[1]) && isfinite(x_state->w[2])) {
						memcpy(&(ekf_param->x_w), x_state->w, sizeof(x_state->w));
					} else {
						warnx("NaN in wind estimate!");
						continue;
					}

					if (isfinite(x_state->QFF)) {
						ekf_param->x_QFF = x_state->QFF;
					} else {
						warnx("NaN in QFF estimate!");
						continue;
					}

					if (isfinite(x_state->K)) {
						ekf_param->x_K = x_state->K;
					} else {
						warnx("NaN in K estimate!");
						continue;
					}

					for (uint8_t i=0; i<20;i++){
						if (isfinite(P_apost[i+20*i])) {
							ekf_param->P_var_vect[i] = P_apost[i+20*i];
						} else {
							warnx("NaN in variance estimate!");
							continue;
						}
					}

					memcpy(&(ekf_param->update_vect), update_vect, sizeof(update_vect));

					orb_publish(ORB_ID(state_estimator_EKF_parameters), pub_ekf_param, ekf_param);

					/* Get home position report */
					bool home_pos_updated;
					orb_check(_home_sub, &home_pos_updated);
					if (home_pos_updated) {
						/* get a local copy of the home position data */
						struct home_position_s home;
						memset(&home, 0, sizeof(home));
						orb_copy(ORB_ID(home_position), _home_sub, &home);

						_home_alt = ((float)(home.alt))*1e-3f;
					}

					/* vehicle global position publication */
					vehicle_global_pos->timestamp = hrt_absolute_time();
					vehicle_global_pos->time_utc_usec = gps.timestamp_position;

					vehicle_global_pos->lat = x_state->p[0]*1e-7l*RAD2DEG;
					vehicle_global_pos->lon = x_state->p[1]*1e-7l*RAD2DEG;

					vehicle_global_pos->alt = (float) x_state->p[2];

					vehicle_global_pos->vel_n = x_state->v_N[0];
					vehicle_global_pos->vel_e = x_state->v_N[1];
					vehicle_global_pos->vel_d = x_state->v_N[2];

					vehicle_global_pos->yaw = att->yaw;

					if (isfinite(x_state->p[0]) && isfinite(x_state->p[1]) && isfinite(x_state->p[2]) && isfinite(x_state->v_N[0]) && isfinite(x_state->v_N[1]) && isfinite(x_state->v_N[2]) && isfinite(att->yaw)) {
						orb_publish(ORB_ID(vehicle_global_position), pub_vehicle_global_pos, vehicle_global_pos);
					} else {
						warnx("NaN in vehicle global position estimate!");
					}

					/* Reset sensor update rate vector */
					memset(sensor_update_hz, 0, sizeof(sensor_update_hz));

					/* Reset update vector */
					memset(update_vect, 0, sizeof(update_vect));

					perf_end(ekf_loop_perf);
				}
			}
		}

		loopcounter++;
	}

	free(raw);
	free(att);
	free(control_mode);
	free(vehicle_global_pos);
	free(ekf_param);
	free(x_state);

	thread_running = false;

	return 0;
}
