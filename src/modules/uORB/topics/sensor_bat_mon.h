/****************************************************************************
 *
 *   Copyright (C) 2008-2014 ASL Development Team. All rights reserved.
 *   Author: @author Philipp Oettershagen <philipp.oettershagen@mavt.ethz.ch>
 *           @author Amir Melzer <amir.melzer@mavt.ethz.ch>
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
 * @file sensor_bat_mon.h
 * Definition of the sensor_power uORB topic. This is for battery monitoring/
 * sensors that extend the PX4-onboard sensors.
 */

#ifndef SENSOR_BAT_MON_H_
#define SENSOR_BAT_MON_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

/**
 * Sensor readings in raw and SI-unit form.
 *
 * These values are read from the sensors. Raw values are in sensor-specific units,
 * the scaled values are in SI-units, as visible from the ending of the variable
 * or the comments. The use of the SI fields is in general advised, as these fields
 * are scaled and offset-compensated where possible and do not change with board
 * revisions and sensor updates.
 *
 */
struct sensor_bat_mon_s {

	/*
	 * Actual data, this is specific to the type of data which is stored in this struct
	 * A line containing L0GME will be added by the Python logging code generator to the
	 * logged dataset.
	 */

	uint64_t timestamp;					/**< Timestamp in microseconds since boot, of bat monitor 	*/

	uint16_t temperature;				/**< battery monitor sensor temperature report in [0.1 K] 	*/
	uint16_t voltage;					/**< battery monitor sensor voltage report in [mV] 			*/
	uint16_t current;					/**< battery monitor sensor current report in [mA] 			*/
	uint16_t batterystatus;				/**< battery monitor sensor battery status report in Hex 	*/
	uint16_t serialnumber;				/**< battery monitor sensor serial number report in Hex 	*/
	uint16_t hostfetcontrol;			/**< battery monitor sensor host FET control report in Hex 	*/
	uint16_t cellvoltage1;				/**< battery monitor sensor cell 1 voltage  report in [mV] 	*/
	uint16_t cellvoltage2;				/**< battery monitor sensor cell 2 voltage report in [mV] 	*/
	uint16_t cellvoltage3;				/**< battery monitor sensor cell 3 voltage report in [mV] 	*/
	uint16_t cellvoltage4;				/**< battery monitor sensor cell 4 voltage report in [mV] 	*/
	uint16_t cellvoltage5;				/**< battery monitor sensor cell 5 voltage report in [mV] 	*/
	uint16_t cellvoltage6;				/**< battery monitor sensor cell 6 voltage report in [mV] 	*/

};

/**
 * @}
 */

/* register this as object request broker structure */
#define MAX_NUM_BAT_MON_SENSORS 3

ORB_DECLARE(sensor_bat_mon_0);
ORB_DECLARE(sensor_bat_mon_1);
ORB_DECLARE(sensor_bat_mon_2);

static const struct orb_metadata *sensor_bat_mon_orb_id[MAX_NUM_BAT_MON_SENSORS] = {
	ORB_ID(sensor_bat_mon_0),
	ORB_ID(sensor_bat_mon_1),
	ORB_ID(sensor_bat_mon_2),
};

// This is a hack to quiet an unused-variable warning for when telemetry_status.h is
// included but telemetry_status_orb_id is not referenced. The inline works if you
// choose to use it, but you can continue to just directly index into the array as well.
// If you don't use the inline this ends up being a no-op with no additional code emitted.
extern inline const struct orb_metadata *sensor_bat_mon_orb_id_lookup(size_t index);
extern inline const struct orb_metadata *sensor_bat_mon_orb_id_lookup(size_t index)
{
	return sensor_bat_mon_orb_id[index];
}

#endif
