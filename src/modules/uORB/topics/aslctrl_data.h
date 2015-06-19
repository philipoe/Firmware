/****************************************************************************
 *
 *   Copyright (C) 2013 Autonomous Systems Lab, ETH Zurich. All rights reserved.
 *   Author: @author
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
 * @file aslctrl_data.h
 * Definition of the aslctrl-data uORB topic.
 */

#ifndef ASLCTRL_DATA_H_
#define ASLCTRL_DATA_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 */

struct aslctrl_data_s {

	/*
	 * Actual data, this is specific to the type of data which is stored in this struct
	 * A line containing L0GME will be added by the Python logging code generator to the
	 * logged dataset.
	 */

	uint64_t timestamp;      	/**< in microseconds since system start          */
	uint32_t dt;				// time since last autopilot main-loop run
	uint8_t aslctrl_mode; 		// The control mode (manual, assisted, auto...)
	uint8_t aslctrl_last_mode;	// The control mode before the last mode switch

	//Longitudinal loop
	float h;
	float hRef;
	float hRef_t;
	float PitchAngle;
	float PitchAngleRef;
	float PitchAngleRefCT;
	float q;
	float qRef;
	float uElev;
	float uThrot;
	float uThrot2;
	float AirspeedRef;
	bool bEngageSpoilers;

	//Lateral loops
	float YawAngle;
	float YawAngleRef;
	float RollAngle;
	float RollAngleRef;
	float p;
	float pRef;
	float r;
	float rRef;
	float uAil;
	float uRud;
	float Yawdot_ref; 		// for coordinated turns
	float Yawdot;			// for coordinated turns

	//General
	float f_GainSch_Q;
	float P_kP_GainSch_E;
	float R_kP_GainSch_E;
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(aslctrl_data);

#endif
