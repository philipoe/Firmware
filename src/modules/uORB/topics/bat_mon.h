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
 * @file bat_mon.h
 * Definition of the sensor_power uORB topic. This is for battery monitoring/
 * sensors that extend the PX4-onboard sensors.
 */

#ifndef BAT_MON_H_
#define BAT_MON_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

/**
 * Battery monitor sensor topics
 */

struct bat_mon_s {

	uint64_t timestamp;					/**< Timestamp in microseconds since boot, of bat monitor 	*/
	uint16_t temperature;				/**< battery monitor sensor temperature report in [0.1 K] 	*/
	uint16_t voltage;					/**< battery monitor sensor voltage report in [mV] 			*/
	int16_t  current;					/**< battery monitor sensor current report in [mA] 			*/
	uint8_t  stateofcharge;				/**< battery monitor sensor state of charge in [%] 			*/
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


ORB_DECLARE(bat_mon);

#endif
