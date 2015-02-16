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
 * @file sdp600.cpp
 * Driver for the SDP600 differential barometric pressure sensor connected via I2C.
 *
 * Author: Amir Melzer  <amir.melzer@mavt.ethz.ch>
 * 	       Lorenz Meier <lm@inf.ethz.ch>
 *
 */

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <board_config.h>

#include <drivers/drv_hrt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include <drivers/drv_airspeed.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/subsystem_info.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class SDP600 : public device::I2C
{
public:
	SDP600(int bus);
	~SDP600();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	virtual int		probe();

private:

	struct work_s		_work;
	unsigned			_measure_ticks;

	unsigned			_num_reports;
	volatile unsigned	_next_report;
	volatile unsigned	_oldest_report;
	struct differential_pressure_s	*_reports;

	bool				_measurement_phase;

	orb_advert_t		_dbaro_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	float 				dPressure;
	float 				temperature;
	float 				vddSupply;

	float				_max_differential_pressure_pa;

	math::LowPassFilter2p	_filter;

	float				_dbaro_Dtube;
	float 				_dbaro_Ltube;

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 *
	 * @param address	The I2C bus address to probe.
	 * @return			True if the device is present.
	 */
	int				probe_address(uint8_t address);

	/**
	 * Initialize the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 */
	void			cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * Issue a dPresssure measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int				measurement();

	/**
	 * Issue a temperature measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int				temp_measurement();

	/**
	 * Issue a VDD voltage supply measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int				vddsupply_measurement();

	/**
	 * Send a reset command to the SDP600.
	 *
	 * This is required after any bus reset.
	 */
	int				cmd_reset();

	/**
	 * Set measurement resolution for the SDP600.
	 *
	 */

	int 			res_change(uint8_t res);

	/**
	 * Set Scale/shift correction for the SDP600.
	 *
	 */

	int 			scale_correction(bool corr);

	/**
	 * CRC routine ported from SDP600 application note
	 *
	 * @param n_prom	Pointer to words read from PROM.
	 * @return		True if the CRC matches.
	 */

	bool			crc8(uint8_t *n_prom);

	/**
	 * Compensation for pressure drop in the hose
	 *
	 * @return			Correction factor
	 */

	float dbaro_pressure_corr(float dbaro_pres_pa_raw, float dbaro_temp_celcius);

};

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { _x++; if (_x >= _lim) _x = 0; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW4(_x)		((_x) * (_x) * (_x) * (_x))

/*
 * SDP600 internal constants and data structures.
 */

#define SDP600_BUS				PX4_I2C_BUS_EXPANSION

#define SDP600_ADDRESS			0x40    // PX4_I2C_OBDEV_SDP600 	/* SDP600 I2C address   */

#define USER_REG_W				0xE2	/* write address of the user register */
#define USER_REG_R				0xE3	/* read address of the user register */
#define ADV_USER_REG_W			0xE4	/* write address of the advance user register */
#define ADV_USER_REG_R			0xE5	/* read address of the advance user register */

#define MEASUREMENT_TRIG		0xF1	/* write to this address to trigger a measurement */
#define TEMP_MEASUREMENT_TRIG	0xF3	/* write to this address to trigger a temperature measurement */
#define VDD_MEASUREMENT_TRIG	0xF5	/* write to this address to trigger a VDD measurement */
#define RESET_CMD				0xFE	/* write to this address to reset chip */

#define SDP600_SCALEFACTOR 	    60.0f	/* define the SDP600-500Pa Scale factor in Pa^-1 */

#define CRC_POLYNOM 			0x131  	/* define the SDP600 CRC8 Polynomial: P(x)=x^8+x^5+x^4+1 = 100110001 */

/* Measurement rate is 20Hz */
#define MEAS_RATE 20
#define MEAS_DRIVER_FILTER_FREQ 5.0f
#define SDP600_CONVERSION_INTERVAL	(1000000 / MEAS_RATE)			/* microseconds */

#define TUBE_LOSS_COMPENSATION 1		   /* Flag for enabling the tube pressure loss compensation (1:enable) */
#define PI_f 3.141592653f
#define KELVIN_CONV 273.15f
#define BAROMETRIC_PRESSURE_MBAR 1013.25f  /* sea level standard atmospheric pressure (default value) */

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int sdp600_main(int argc, char *argv[]);


SDP600::SDP600(int bus) :
	I2C("SDP600", AIRSPEED_DEVICE_PATH, bus, 0, 400000),
	_measure_ticks(0),
	_num_reports(0),
	_next_report(0),
	_oldest_report(0),
	_reports(nullptr),
	_measurement_phase(false),
	_dbaro_topic(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "sdp600_read")),
	_comms_errors(perf_alloc(PC_COUNT, "sdp600_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "sdp600_buffer_overflows")),
	_max_differential_pressure_pa(-500.0f),
	_filter(MEAS_RATE, MEAS_DRIVER_FILTER_FREQ),
	_dbaro_Dtube(0.0f),
	_dbaro_Ltube(0.0f)
{
	// enable debug() calls
	_debug_enabled = true;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

SDP600::~SDP600()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete[] _reports;
}

int
SDP600::init()
{
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_num_reports = 2;
	_reports = new struct differential_pressure_s[_num_reports];

	if (_reports == nullptr)
		goto out;

	_oldest_report = _next_report = 0;

	/* get a publish handle on the dbaro topic */
	memset(&_reports[0], 0, sizeof(_reports[0]));
	_dbaro_topic = orb_advertise(ORB_ID(differential_pressure), &_reports[0]);

	if (_dbaro_topic < 0)
		debug("failed to create sensor_dbaro object");

	ret = OK;
out:
	return ret;
}

int
SDP600::probe()
{
	_retries = 10;

	if (OK == probe_address(SDP600_ADDRESS)){
		_retries = 1;
		return OK;
	}

	return -EIO;
}

int
SDP600::probe_address(uint8_t address)
{
	set_address(address);

	/* send reset command */
	if (OK != cmd_reset())
		return -EIO;

	scale_correction(true);

	return OK;
}

ssize_t
SDP600::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct differential_pressure_s);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_oldest_report != _next_report) {
				memcpy(buffer, _reports + _oldest_report, sizeof(*_reports));
				ret += sizeof(_reports[0]);
				INCREMENT(_oldest_report, _num_reports);
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_measurement_phase = 0;
		_oldest_report = _next_report = 0;

		/* Take a pressure measurement */

		if (OK != measurement()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		memcpy(buffer, _reports, sizeof(*_reports));
		ret = sizeof(*_reports);

	} while (0);

	return ret;
}

int
SDP600::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

				/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

				/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

				/* zero would be bad */
			case 0:
				return -EINVAL;

				/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(SDP600_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}

				/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(SDP600_CONVERSION_INTERVAL))
						return -EINVAL;

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0)
			return SENSOR_POLLRATE_MANUAL;

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* add one to account for the sentinel in the ring */
			arg++;

			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 2) || (arg > 100))
				return -EINVAL;

			/* allocate new buffer */
			struct differential_pressure_s *buf = new struct differential_pressure_s[arg];

			if (nullptr == buf)
				return -ENOMEM;

			/* reset the measurement state machine with the new buffer, free the old */
			stop();
			delete[] _reports;
			_num_reports = arg;
			_reports = buf;
			start();

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _num_reports - 1;

	case SENSORIOCRESET:
		/* reset the measurement state machine */
		stop();

		/* free any existing reports */
		if (_reports != nullptr)
			delete[] _reports;

		start();
		return OK;
		//return -EINVAL;

	case AIRSPEEDIOCSCOMPE:{
		struct airspeed_tube_compensation *s = (struct airspeed_tube_compensation*)arg;
		_dbaro_Dtube = s->dbaro_Dtube;
		_dbaro_Ltube = s->dbaro_Ltube;
		return OK;
	}

	default:
		break;
	}

	/* give it to the superclass */
	return I2C::ioctl(filp, cmd, arg);
}

void
SDP600::start()
{
	/* reset the report ring and state machine */
	_measurement_phase = true;
	_oldest_report = _next_report = 0;

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&SDP600::cycle_trampoline, this, 1);
}

void
SDP600::stop()
{
	work_cancel(HPWORK, &_work);
}

void
SDP600::cycle_trampoline(void *arg)
{
	SDP600 *dev = (SDP600 *)arg;

	dev->cycle();
}

void
SDP600::cycle()
{

	/* collection phase? */
	if (_measurement_phase) {
		/* perform dpressure measurement */
		if (OK != measurement()) {
#if 0
			warnx("dpressure measurement error, restarting SDP600 device");

			/* free any existing reports and reset the state machine and try again */
			if (_reports != nullptr)
				delete[] _reports;
			probe();
#endif
			start();
			return;
		}

		/* next phase is measurement */
		_measurement_phase = true;

		/* schedule a fresh cycle call when the measurement is done */
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&SDP600::cycle_trampoline,
			   this,
			   USEC2TICK(SDP600_CONVERSION_INTERVAL));
	}
}

int
SDP600::measurement()
{
	uint8_t cmd = MEASUREMENT_TRIG;							// Trigger differential pressure measurement;
	uint8_t data[3];
	union {
		uint8_t	b[3];
		uint32_t w;
	} cvt;

	/* read the most recent measurement */
	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	_reports[_next_report].timestamp = hrt_absolute_time();

	/* fetch the raw value */

	if (OK != transfer(&cmd, 1, &data[0], 3)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* assemble 16 bit value and convert from big endian (sensor) to little endian (MCU) */
	cvt.b[0] = data[1];
	cvt.b[1] = data[0];
	cvt.b[2] = data[2];

	/* calculate CRC and return success/failure accordingly */
	if (!crc8(&cvt.b[0])) {
		return -EIO;
	}

	/* pressure calculation, result in Pa */
	dPressure = (float)((int16_t)(cvt.w & 0xffff)) / SDP600_SCALEFACTOR;

	/* Range check /failure accordingly */

	if ( (dPressure > 500) | (dPressure < -500) ) {
		warnx("SDP600: Differential pressure is out of range: %3.6f Pa", (double) dPressure);
		return -EIO;
	}

	//warnx("calculated effective differential pressure %3.6f Pa", (double) dPressure);   			// remove
#if !TUBE_LOSS_COMPENSATION
	temperature = -1000.0f;
#else
	float epsilon = 0.0f;
	temp_measurement();																	/* get the sensors temperature (option) */
	epsilon	= dbaro_pressure_corr(dPressure, temperature);

	dPressure	  = dPressure / (1 + epsilon); 											/* Pressure in pa with tube loss compensation */
#endif

	/* track maximum differential pressure measured (so we can work out top speed). */
	if (dPressure > _max_differential_pressure_pa) {
		_max_differential_pressure_pa = dPressure;
	}

	_reports[_next_report].timestamp = hrt_absolute_time();
	_reports[_next_report].error_count = perf_event_count(_comms_errors);
	_reports[_next_report].temperature = temperature;
	_reports[_next_report].differential_pressure_filtered_pa =  _filter.apply(dPressure);
	_reports[_next_report].differential_pressure_raw_pa = dPressure;
	_reports[_next_report].max_differential_pressure_pa = _max_differential_pressure_pa;

	/* publish it */
	orb_publish(ORB_ID(differential_pressure), _dbaro_topic, &_reports[_next_report]);

	/* post a report to the ring - note, not locked */
	INCREMENT(_next_report, _num_reports);

	/* if we are running up against the oldest report, toss it */
	if (_next_report == _oldest_report) {
		perf_count(_buffer_overflows);
		INCREMENT(_oldest_report, _num_reports);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	perf_end(_sample_perf);

	return OK;

}
/* Temperature measurement: */
int
SDP600::temp_measurement()
{
	uint8_t cmd = TEMP_MEASUREMENT_TRIG;
	uint8_t data[3];
	union {
		uint8_t	b[3];
		uint32_t w;
	} cvt;

	/* read the most recent measurement */
	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	_reports[_next_report].timestamp = hrt_absolute_time();

	/* fetch the raw value */

	if (OK != transfer(&cmd, 1, &data[0], 3)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* assemble 16 bit value and convert from big endian (sensor) to little endian (MCU) */
	cvt.b[0] = data[1];
	cvt.b[1] = data[0];
	cvt.b[2] = data[2];

	/* calculate CRC and return success/failure accordingly */
	if (!crc8(&cvt.b[0])) {
		return -EIO;
		}

	/* temperature calculation, result in C */
	temperature = (float)((int16_t)(cvt.w & 0xffff)) / 10;

	if ( (temperature > 80) | (temperature < -20) ) {
			warnx("SDP600: Temperature is out of range: %3.2f C", (double) temperature);
			temperature = -1000.0f;
			return -EIO;
	}

	//warnx("measured temperature by the differential pressure sensor %3.2f C", (double) temperature);  // remove

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	perf_end(_sample_perf);

	return OK;

}

/* VDD supply measurement: */
int
SDP600::vddsupply_measurement()
{
	uint8_t cmd = VDD_MEASUREMENT_TRIG;
	uint8_t data[3];
	union {
		uint8_t	b[3];
		uint32_t w;
	} cvt;

	/* read the most recent measurement */
	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	_reports[_next_report].timestamp = hrt_absolute_time();

	/* fetch the raw value */

	if (OK != transfer(&cmd, 1, &data[0], 3)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* assemble 16 bit value and convert from big endian (sensor) to little endian (MCU) */
	cvt.b[0] = data[1];
	cvt.b[1] = data[0];
	cvt.b[2] = data[2];

	/* calculate CRC and return success/failure accordingly */
	if (!crc8(&cvt.b[0])) {
		return -EIO;
		}

	/* Voltage calculation, result in voltage */
	vddSupply = (float)((uint16_t)(cvt.w & 0xffff)) / 1000;

	if ( (vddSupply > 5) | (vddSupply < 0) ) {
				warnx("SDP600: Supply voltage is out of range: %3.2f V", (double) vddSupply);
				return -EIO;
				}

	/// warnx("measured VDD supply voltage by the differential pressure sensor %3.2f V", (double) vddSupply);   // remove

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	perf_end(_sample_perf);

	return OK;

}

int
SDP600::cmd_reset()
{
	unsigned	old_retrycount = _retries;
	uint8_t		cmd = RESET_CMD;
	int		result;

	/* bump the retry count */
	_retries = 10;
	result = transfer(&cmd, 1, nullptr, 0);
	_retries = old_retrycount;

	return result;
}

int
SDP600::res_change(uint8_t res)
{
	/*Change the measurement resolution, modify advanced user register*/
	/*	res:
	 	000: 9 bit
		001: 10 bit
		010: 11 bit
		011: 12 bit (sensor default)
		100: 13 bit
		101: 14 bit
		110: 15 bit
		111: 16 bit
	*/

	uint8_t		cmd = ADV_USER_REG_R;
	int		result;
	uint8_t data[3];
	union {
		uint8_t	b[3];
		uint32_t w;
	} cvt;


	if (OK != transfer(&cmd,1, &data[0], 3)) {
			perf_count(_comms_errors);
			return -EIO;
		}

	/* assemble 16 bit value and convert from big endian (sensor) to little endian (MCU) */
	cvt.b[0] = data[1];
	cvt.b[1] = data[0];
	cvt.b[2] = data[2];

	/* calculate CRC and return success/failure accordingly */
	if (!crc8(&cvt.b[0])) {
		return -EIO;
		}

	cmd 	= ADV_USER_REG_W;

	data[2] = data[1];
	data[1] = (data[0] & 0xf1 ) | ((res & 0xff) << 1);
	data[0] = cmd;

	if (OK != transfer(&data[0],3, nullptr, 0)) {
				perf_count(_comms_errors);
				return -EIO;
			}
	return result;
}

int
SDP600::scale_correction(bool corr)
{
	/* False: user register in Temperature raw data mode (Default)*/
	/* True:  user register set to enable temperature scale/sift correction */

	uint8_t		cmd = USER_REG_R;
	int		result;
	uint8_t data[3];
	union {
			uint8_t	b[3];
			uint32_t w;
		} cvt;

	if (OK != transfer(&cmd,1, &data[0], 3)) {
			perf_count(_comms_errors);
			return -EIO;
		}

	/* assemble 16 bit value and convert from big endian (sensor) to little endian (MCU) */
		cvt.b[0] = data[1];
		cvt.b[1] = data[0];
		cvt.b[2] = data[2];

	/* calculate CRC and return success/failure accordingly */
	if (!crc8(&cvt.b[0])) {
		return -EIO;
	}

	cmd 	= USER_REG_W;
	data[2] = data[1];

	if (corr)
		data[1] = data[0] | 0x04;
	else
		data[1] = data[0] & 0xfb;

	data[0] = cmd;

	if (OK != transfer(&data[0],3, nullptr, 0)) {
				perf_count(_comms_errors);
				return -EIO;
			}
	return result;
}

bool
SDP600::crc8(uint8_t *crc_data)
{
	uint8_t crc_read;
	uint8_t crc = 0;
	uint8_t byteCtr;
	uint8_t numberOfDataBytes = 2;

	/* save the read crc */
	crc_read = crc_data[numberOfDataBytes];

	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = numberOfDataBytes; byteCtr >0; --byteCtr){
		crc ^= (crc_data[byteCtr-1]);
		for (uint8_t bit = 8; bit > 0; --bit){
			if (crc & 0x80)
				crc = (crc << 1) ^ CRC_POLYNOM;
			else
				crc = (crc << 1);
		}
	}
	return (crc_read == crc);
}

float
SDP600::dbaro_pressure_corr(float dbaro_pres_pa_raw, float dbaro_temp_celcius)
{
	/* corrected dbaro pressure due to viscous friction of the tubes */
		float Nair;
		float Rair;
		float Eps;
		float dbaro_DtubePow4;

		if (dbaro_pres_pa_raw < 0.0f)
			dbaro_pres_pa_raw = -1*dbaro_pres_pa_raw;
					//TODO: This causes errors at low velocities for the HDIM10, which often has a negative offset! (PhOe)
					// -> we need to correct for this via calibration

		if ((dbaro_pres_pa_raw > 0.0f) && (_dbaro_Dtube > 0.0f)) {
			dbaro_DtubePow4 = POW4(_dbaro_Dtube);
			Nair = (float) (18.205f + 0.0484f * (dbaro_temp_celcius - 20.0f)) * 1e-6f;
			Rair = (float) (1.1885f * BAROMETRIC_PRESSURE_MBAR *1e-3f *(KELVIN_CONV + 20.0f))/(KELVIN_CONV + dbaro_temp_celcius);
			float denominator=PI_f*dbaro_DtubePow4*Rair*dbaro_pres_pa_raw;

			if(fabsf(denominator)>1E-32f)
				Eps  = (float) -64.0f*_dbaro_Ltube*Nair*6.17e-7f*(((float)sqrt((float)(1.0f+8.0f*dbaro_pres_pa_raw/62.0f))-1.0f))/denominator;
			else
				Eps  = 0.0f;

			if ((fabsf(Eps) < 1.0f) && (fabsf(Eps) >= 0.0f))
				return (Eps);
			else
				return (0.0f);
		}
		else
			return (0.0f);
}
void
SDP600::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("report queue:   %u (%u/%u @ %p)\n",
	       _num_reports, _oldest_report, _next_report, _reports);
	printf("Differential Pressure:   %10.4f\n", (double)(dPressure));
}

/**
 * Local functions in support of the shell command.
 */
namespace sdp600
{

SDP600	*g_dev;

void	start();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new SDP600(SDP600_BUS);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(AIRSPEED_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		goto fail;

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

void
test()
{
	struct differential_pressure_s report;
	ssize_t sz;
	int ret;

	int fd = open(AIRSPEED_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'SDP600 start' if the driver is not running)", AIRSPEED_DEVICE_PATH);

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report))
		err(1, "immediate read failed");

	warnx("single read");
	warnx("dpressure:   %10.4f", (double)report.differential_pressure_filtered_pa);
	warnx("time:        %lld", report.timestamp);

	/* set the queue depth to 10 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10))
		errx(1, "failed to set queue depth");

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2))
		errx(1, "failed to set 2Hz poll rate");

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1)
			errx(1, "timed out waiting for sensor data");

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report))
			err(1, "periodic read failed");

		warnx("dpressure:   %10.4f", (double)report.differential_pressure_filtered_pa);
		warnx("time:        %lld", report.timestamp);

	}

	/* reset the sensor polling to its default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT))
		errx(1, "failed to set default rate");

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(AIRSPEED_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "driver poll restart failed");

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}


} // namespace

int
sdp600_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		sdp600::start();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		sdp600::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		sdp600::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info"))
		sdp600::info();

	errx(1, "unrecognised command, try 'start', 'test', 'reset' or 'info'");
}
