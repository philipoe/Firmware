/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file current sensor driver interface.
 */

#ifndef _DRV_VOLTAGE_CURRENT_H
#define _DRV_VOLTAGE_CURRENT_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define ADC121_VSPB_DEVICE_PATH	"/dev/adc121/adc121_vspb"
#define ADC121_CSPB_DEVICE_PATH	"/dev/adc121/adc121_cspb"
#define ADC121_CS1_DEVICE_PATH	"/dev/adc121/adc121_cs1"
#define ADC121_CS2_DEVICE_PATH	"/dev/adc121/adc121_cs2"


/**
 * voltage and current sensors report structure. Reads from the device must be in multiples of this structure.
 */
struct adc121_vspb_report {
	float voltage;
	uint64_t timestamp;
};

struct adc121_cspb_report {
	float current;
	uint64_t timestamp;
};

struct adc121_cs1_report {
	float current;
	uint64_t timestamp;
};

struct adc121_cs2_report {
	float current;
	uint64_t timestamp;
};
/*
 * ObjDev tag for raw voltage and current data.
 */
ORB_DECLARE(sensor_adc121_vspb);
ORB_DECLARE(sensor_adc121_cspb);
ORB_DECLARE(sensor_adc121_cs1);
ORB_DECLARE(sensor_adc121_cs2);

/*
 * ioctl() definitions
 */

#define _VOLTAGECURRENTIOCBASE		(0x2d00)						// check the VOLTAGECURRENTIOCBASE value of 0x2d00
#define _CURRENTIOC(_n)		(_IOC(_VOLTAGECURRENTIOCBASE, _n))


#endif /* _DRV_VOLTAGE_CURRENT_H */
