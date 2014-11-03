/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: @author Thomas Gubler <thomasgubler@student.ethz.ch>
 *           @author Julian Oes <joes@student.ethz.ch>
 *           @author Lorenz Meier <lm@inf.ethz.ch>
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

/**
 * @file state_estimator_EKF_parameters.h
 * Definition of the attitude uORB topic.
 */

#ifndef STATE_ESTIMATOR_EKF_PARAMETERS_H_
#define STATE_ESTIMATOR_EKF_PARAMETERS_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 */

struct state_estimator_EKF_parameters_s {

	/*
	 * Actual data, this is specific to the type of data which is stored in this struct
	 * A line containing L0GME will be added by the Python logging code generator to the
	 * logged dataset.
	 */

	uint64_t timestamp;      /**< in microseconds since system start          */

	/**< EKF states:  [p(lat*1e7,lon*1e7,h)||q_NS||Vn,Ve,Vd||b_gx,b_gy,b_gz||b_ax,b_ay,b_az||QFF||Wn,We,Wd||K] */
	double x_p[3];
	float x_q_NS[4];
	float x_v_N[3];
	float x_b_g[3];
	float x_b_a[3];
	float x_QFF;
	float x_w[3];
	float x_K;

	float alpha;		// Estimated angle of attack [rad]
	float beta;			// Estimated sideslip angle [rad]
	float airspeed;		// Filtered/estimated airspeed [m/s]. Safer than using raw airspeed-sensor data.

	/**< EKF states covariance matrix*/
	float P_var_vect[20];

	/**< EKF update vector: [gyro||accel||mag||baro||dbaro||amb_temp||gps_pos||gps_vel] */
	uint8_t update_vect[8];

};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(state_estimator_EKF_parameters);

#endif
