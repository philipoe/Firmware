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
 * @file spv1020.cpp
 * Driver for the SPV1020 MPPT sensor connected via I2C over I2C/SPI bridge.
 *
 * Author: Amir Melzer  <amir.melzer@mavt.ethz.ch>
 * 	       Lorenz Meier <lm@inf.ethz.ch>
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
#include <systemlib/param/param.h>

#include <drivers/drv_mppt.h>

#define MAX_NUMBER_OF_MPPT_DEVICES		3
/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

static uint8_t 	max_mppt_devices;								/* Defines the maximal MPPT devices */

class SPV1020 : public device::I2C
{
public:
	SPV1020(int bus);
	virtual ~SPV1020();

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
	struct spv1020_report	*_reports;

	bool				_measurement_phase;

	float 				mppt_current[3];
	float 				mppt_voltage[3];
	uint16_t 			mppt_pwm[3];
	uint8_t 			mppt_status[3];

	float				mppt_current_bias_term[3];
	float				mppt_current_SF_term[3];

	orb_advert_t		_mppt_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	current_cal_term	_current_cal_term;

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 *
	 * @param address	The I2C bus address to probe.
	 * @return			True if the device is present.
	 */
	int			probe_address(uint8_t address);

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
	 * Issue a MPPT measurement command for retrieving the current, voltage , PWM  and status vector.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			mppt_measurement();

	/**
	 * Issue a MPPT current measurement command for retrieving the current value
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			mppt_current_read(uint8_t mppt_device);

	/**
	 * Issue a MPPT voltage measurement command for retrieving the voltage value
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			mppt_voltage_read(uint8_t mppt_device);

	/**
	 * Issue a MPPT PWM measurement command for retrieving the PWM value
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			mppt_pwm_read(uint8_t mppt_device);

	/**
	 * Issue a MPPT status command for retrieving the status vector
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			mppt_status_read(uint8_t mppt_device);

	/**
	 * Issue a MPPT turn off command
	 *
	 */
	int			mppt_turn_off(uint8_t mppt_device);

	/**
	 * Issue a MPPT turn on command
	 *
	 */
	int			mppt_turn_on(uint8_t mppt_device);

	/**
	 * Configure the I2C to SPI bridge (SC18IS602) to be used for the SPV1020.
	 *
	 */
	int 		conf_I2C_SPI_bridge();

};

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { _x++; if (_x >= _lim) _x = 0; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

/*
 * SPV1020 internal constants and data structures.
 */

/* internal conversion time: 1 s  */
#define SPV1020_CONVERSION_INTERVAL	1000000	/* microseconds */ /*set sample rate to 1 Hz */

#define SPV1020_BUS				PX4_I2C_BUS_EXPANSION

#define SPV1020_ADDRESS			0x28    /* SPV1020 I2C address (SC18IS602 address configuration)   */

#define WRITE_TO_SPI_DEV0		0x01	/* Write to SPI slave address 0 (of the I2C/SPI bridge)    */
#define WRITE_TO_SPI_DEV1		0x02	/* Write to SPI slave address 1 (of the I2C/SPI bridge)    */
#define WRITE_TO_SPI_DEV2		0x04	/* Write to SPI slave address 2 (of the I2C/SPI bridge)    */
#define WRITE_TO_SPI_DEV3		0x0f	/* Write to SPI slave address 3 (of the I2C/SPI bridge)    */

#define CONF_SPI_INTERFACE		0xf0	/* configuration of the 2C/SPI bridge					   */

#define MPPT_TURN_OFF			0x02	/* MPPT device turn-off									   */
#define MPPT_TURN_ON			0x03	/* MPPT device turn-on									   */
#define MPPT_READ_CURRENT		0x04	/* Read current of the MPPT device 					   	   */
#define MPPT_READ_VOLTAGE		0x05	/* Read input voltage of the MPPT device 				   */
#define MPPT_READ_PWM			0x06	/* Read pwm of the MPPT device 							   */
#define MPPT_READ_STATUS		0x07	/* Read status of the MPPT device 						   */


#define SPI_ORDER_MSB			0b0		/* Configure SPI so that MSB sent first		 			   */
#define SPI_ORDER_LSB			0b1		/* Configure SPI so that LSB sent first		 			   */
#define SPI_MODE_00				0b00	/* Configure SPI mode selection to CPOL = 0, CPHA = 0 	   */
#define SPI_MODE_01				0b01	/* Configure SPI mode selection to CPOL = 0, CPHA = 1 	   */
#define SPI_MODE_10				0b10	/* Configure SPI mode selection to CPOL = 1, CPHA = 0 	   */
#define SPI_MODE_11				0b11	/* Configure SPI mode selection to CPOL = 1, CPHA = 1 	   */
#define SPI_CLK_1843			0b00	/* Configure SPI clock rate to 1843kHz 					   */
#define SPI_CLK_461 			0b01	/* Configure SPI clock rate to 461kHz 					   */
#define SPI_CLK_115				0b10	/* Configure SPI clock rate to 115kHz 					   */
#define SPI_CLK_58				0b11	/* Configure SPI clock rate to 58kHz 					   */

#define MPPT_VOLT_COV_FACTOR	0.01876f	/* MPPT_VOLT_COV_FACTOR = (1+R101/R102)*(1.25/2^10) = 0.01876  */

#define SPV1020_MAXIMAL_VOLTAGE_RANGE	40.0f
#define SPV1020_MAXIMAL_CURRENT_RANGE	9.0f
#define SPV1020_MAXIMAL_PWM_RANGE		512

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int spv1020_main(int argc, char *argv[]);

SPV1020::SPV1020(int bus) :
	I2C("SPV1020", SPV1020_DEVICE_PATH, bus, 0, 100000),
	_measure_ticks(0),
	_num_reports(0),
	_next_report(0),
	_oldest_report(0),
	_reports(nullptr),
	_measurement_phase(false),
	_mppt_topic(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "SPV1020_read")),
	_comms_errors(perf_alloc(PC_COUNT, "SPV1020_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "SPV1020_buffer_overflows")),
	_current_cal_term{}
{
	_debug_enabled = true;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));

	_current_cal_term.bias[0] = 0.0f;
	_current_cal_term.bias[1] = 0.0f;
	_current_cal_term.bias[2] = 0.0f;
	_current_cal_term.SF[0] = 1.0f;
	_current_cal_term.SF[1] = 1.0f;
	_current_cal_term.SF[2] = 1.0f;

}

SPV1020::~SPV1020()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete[] _reports;
}

int
SPV1020::init()
{
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_num_reports = 2;
	_reports = new struct spv1020_report[_num_reports];

	if (_reports == nullptr)
		goto out;

	_oldest_report = _next_report = 0;

	/* get a publish handle on the spv1020 topic */
	memset(&_reports[0], 0, sizeof(_reports[0]));
	_mppt_topic = orb_advertise(ORB_ID(sensor_spv1020), &_reports[0]);

	if (_mppt_topic < 0)
		debug("failed to create sensor_spv1020 object");

	ret = OK;
out:
	return ret;
}

int SPV1020::probe()
{
	_retries = 10;

	if (OK == probe_address(SPV1020_ADDRESS)){
		_retries = 1;
		return OK;
	}
	return -EIO;
}

int
SPV1020::probe_address(uint8_t address)
{
	set_address(address);

	/* send reset command */
	if (OK != conf_I2C_SPI_bridge())									/* Try to initialize the I2C/SPI bridge */
		return -EIO;

	return OK;
}

ssize_t
SPV1020::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct spv1020_report);
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

		/* Take a MPPT measurement */

		if (OK != mppt_measurement()) {
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
SPV1020::ioctl(struct file *filp, int cmd, unsigned long arg)
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
					_measure_ticks = USEC2TICK(SPV1020_CONVERSION_INTERVAL);

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
					if (ticks < USEC2TICK(SPV1020_CONVERSION_INTERVAL))
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
			struct spv1020_report *buf = new struct spv1020_report[arg];

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

		start();
		return OK;

	case MPPTIOCSSCALE: {
		/* copy mppt bias */
		struct current_cal_term *s = (struct current_cal_term *) arg;
		memcpy(&_current_cal_term, s, sizeof(_current_cal_term));
		memcpy(&mppt_current_bias_term,_current_cal_term.bias,sizeof(_current_cal_term.bias));
		memcpy(&mppt_current_SF_term,_current_cal_term.SF,sizeof(_current_cal_term.SF));
		return OK;
		}

	case MPPTTURNOFF: {
		if (arg > ((unsigned long) max_mppt_devices-1) )
			return -EINVAL;
		mppt_turn_off((uint8_t) arg);
		return OK;
	}

	case MPPTTURNON: {
		if (arg > ((unsigned long)max_mppt_devices-1) )
			return -EINVAL;
		mppt_turn_on((uint8_t) arg);
		return OK;
	}

	case MPPTRESET: {
		if (arg > ((unsigned long)max_mppt_devices-1) )
			return -EINVAL;
		mppt_turn_off((uint8_t) arg);
		usleep(500000);
		mppt_turn_on((uint8_t) arg);
		return OK;
	}

	case MPPTSETCOMBRIDGE: {
		conf_I2C_SPI_bridge();
		return OK;
	}

	default:
		break;
	}

	/* give it to the superclass */
	return I2C::ioctl(filp, cmd, arg);
}

void
SPV1020::start()
{
	/* reset the report ring and state machine */
	_measurement_phase = true;
	_oldest_report = _next_report = 0;

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&SPV1020::cycle_trampoline, this, 1);
}

void
SPV1020::stop()
{
	work_cancel(HPWORK, &_work);
}

void
SPV1020::cycle_trampoline(void *arg)
{
	SPV1020 *dev = (SPV1020 *)arg;
	dev->cycle();
}

void
SPV1020::cycle()
{
	/* collection phase? */
	if (_measurement_phase) {
		/* perform MPPTs reading cycle */
		if (OK != mppt_measurement()) {
			start();
			return;
		}

	/* next phase is measurement */
	_measurement_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&SPV1020::cycle_trampoline,
		   this,
		   _measure_ticks);
	}
}

/* collect MPPT measurements: */
int
SPV1020::mppt_measurement()
{
	/* read the most recent measurement */
	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	_reports[_next_report].timestamp = hrt_absolute_time();

	for(uint8_t mppt_device=0; mppt_device < MAX_NUMBER_OF_MPPT_DEVICES; mppt_device++){
		mppt_current[mppt_device] = 0.0f;
		mppt_voltage[mppt_device] = 0.0f;
		mppt_pwm[mppt_device]	  = 0;
		mppt_status[mppt_device]  = 0;
	}

	for(uint8_t mppt_device=0; mppt_device < max_mppt_devices; mppt_device++){

		if (OK != mppt_current_read(mppt_device)){
			perf_count(_comms_errors);
			return -EIO;
		}
		if (OK != mppt_voltage_read(mppt_device)){
			perf_count(_comms_errors);
			return -EIO;
		}
		if (OK != mppt_pwm_read(mppt_device)){
			perf_count(_comms_errors);
			return -EIO;
		}
		if (OK != mppt_status_read(mppt_device)){
			perf_count(_comms_errors);
			return -EIO;
		}

		//warnx("MPPT device %d: Current: %10.4f[A], Voltage: %10.4f[v], PWM: %d, Status vector: %x, \n", mppt_device, (double)(mppt_current[mppt_device]), (double)(mppt_voltage[mppt_device]), mppt_pwm[mppt_device], mppt_status[mppt_device]);	// remove display!!!!!!!!

	}

	memcpy(&_reports[_next_report].current, mppt_current, sizeof(mppt_current));   			/* current report [amp] */
	memcpy(&_reports[_next_report].voltage, mppt_voltage, sizeof(mppt_voltage));   			/* voltage report [v] 	*/
	memcpy(&_reports[_next_report].pwm, mppt_pwm, sizeof(mppt_pwm));   						/* pwm report		 	*/
	memcpy(&_reports[_next_report].status, mppt_status, sizeof(mppt_status));  				/* Status vector report */

	/* publish it */
	orb_publish(ORB_ID(sensor_spv1020), _mppt_topic, &_reports[_next_report]);

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

int
SPV1020::mppt_current_read(uint8_t mppt_device)
{
	uint8_t cmd[4];
	uint8_t data[3];
	union {
		uint8_t	b[2];
		uint32_t w;
	} cvt;

	cmd[0] = ( 1 << mppt_device) & 0x07;
	cmd[1] = MPPT_READ_CURRENT;
	cmd[2] = 0x00;
	cmd[3] = 0x00;

	/* Send current read command to the MPPT */
	if (OK != transfer(&cmd[0], 4, nullptr, 0)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* read the value sent by the MPPT */
	if (OK != transfer(nullptr, 0, &data[0], 3)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* assemble 16 bit value and convert from big endian (sensor) to little endian (MCU) */
	cvt.b[0] = data[2];
	cvt.b[1] = data[1];

	/* current calculation (10 bit vector), result in amp */

	uint16_t mppt_current_bin = (uint16_t)(cvt.w & 0x03ff);

	mppt_current[mppt_device] = ((float)(mppt_current_bin))/55.6f/(1.0f+ 2e-6f * (POW2((float)(mppt_current_bin) - mppt_current_bias_term[mppt_device])));

	mppt_current[mppt_device] *= mppt_current_SF_term[mppt_device];

	if (mppt_current[mppt_device] > SPV1020_MAXIMAL_CURRENT_RANGE) {
		/* MPPTs current measurement is not stable, range check action is commented out */
		//warnx("SPV1020: MPPT %d Current is out of range: %3.2f [A]", mppt_device, (double) mppt_current[mppt_device]);
		//mppt_current[mppt_device] = 0.0f;
		//return -EIO;
	}
	return OK;
}

int
SPV1020::mppt_voltage_read(uint8_t mppt_device)
{
	uint8_t cmd[4];
	uint8_t data[3];
	union {
		uint8_t	b[2];
		uint32_t w;
	} cvt;

	cmd[0] = ( 1 << mppt_device) & 0x07;
	cmd[1] = MPPT_READ_VOLTAGE;
	cmd[2] = 0x00;
	cmd[3] = 0x00;

	/* Send voltage read command to the MPPT */
	if (OK != transfer(&cmd[0], 4, nullptr, 0)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* read the value sent by the MPPT */
	if (OK != transfer(nullptr, 0, &data[0], 3)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* assemble 16 bit value and convert from big endian (sensor) to little endian (MCU) */
	cvt.b[0] = data[2];
	cvt.b[1] = data[1];

	/* voltage calculation (10 bit vector), result in volts */
	mppt_voltage[mppt_device] = (float)((uint16_t)(cvt.w & 0x03ff)) * MPPT_VOLT_COV_FACTOR;

	if ( (mppt_voltage[mppt_device] > SPV1020_MAXIMAL_VOLTAGE_RANGE) | (mppt_voltage[mppt_device] < (-1*SPV1020_MAXIMAL_VOLTAGE_RANGE) ) ) {
		warnx("SPV1020: MPPT %d Voltage is out of range: %3.2f [V]", mppt_device, (double) mppt_voltage[mppt_device]);
		//return -EIO;
	}

	return OK;
}

int
SPV1020::mppt_pwm_read(uint8_t mppt_device)
{
	uint8_t cmd[4];
	uint8_t data[3];
	union {
		uint8_t	b[2];
		uint32_t w;
	} cvt;

	cmd[0] = (1 << mppt_device) & 0x07;
	cmd[1] = MPPT_READ_PWM;
	cmd[2] = 0x00;
	cmd[3] = 0x00;

	/* Send PWM read command to the MPPT */
	if (OK != transfer(&cmd[0], 4, nullptr, 0)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* read the value sent by the MPPT */
	if (OK != transfer(nullptr, 0, &data[0], 3)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* assemble 16 bit value and convert from big endian (sensor) to little endian (MCU) */
	cvt.b[0] = data[2];
	cvt.b[1] = data[1];

	/* PWM value (9 bit vector)  */
	mppt_pwm[mppt_device] = (uint16_t)(cvt.w & 0x01ff);

	if (mppt_pwm[mppt_device] > SPV1020_MAXIMAL_PWM_RANGE) {
		warnx("SPV1020: MPPT %d PWM is out of range: %d", mppt_device, mppt_pwm[mppt_device]);
		//return -EIO;
	}

	return OK;
}

int
SPV1020::mppt_status_read(uint8_t mppt_device)
{
	uint8_t cmd[3];
	uint8_t data[2];

	cmd[0] = (1 << mppt_device) & 0x07;
	cmd[1] = MPPT_READ_STATUS;
	cmd[2] = 0x00;

	/* Send status read command to the MPPT */
	if (OK != transfer(&cmd[0], 3, nullptr, 0)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* read the value sent by the MPPT */
	if (OK != transfer(nullptr, 0, &data[0], 2)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* status vector (8 bit vector)  */
	mppt_status[mppt_device] = data[1] & 0xff;
	return OK;
}

int
SPV1020::mppt_turn_off(uint8_t mppt_device)
{
	uint8_t cmd[2];

	cmd[0] = (1 << mppt_device) & 0x07;
	cmd[1] = MPPT_TURN_OFF;

	/* Send turn off command to the MPPT */
	if (OK != transfer(&cmd[0], 2, nullptr, 0)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	return OK;
}

int
SPV1020::mppt_turn_on(uint8_t mppt_device)
{
	uint8_t cmd[2];

	cmd[0] = (1 << mppt_device) & 0x07;
	cmd[1] = MPPT_TURN_ON;

	/* Send turn on command to the MPPT */
	if (OK != transfer(&cmd[0], 2, nullptr, 0)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	return OK;
}

int
SPV1020::conf_I2C_SPI_bridge()
{
	/*Change the configuration of the I2C/SPI bridge to:
	 * Send MSB data first on the SPI line
	 * Mode selection CPOL = 1 and CPHA = 1
	 * SPI clock rate of 1843KHz */

	uint8_t data[2];

	data[0] = CONF_SPI_INTERFACE;
	data[1] = (SPI_CLK_1843 << 0 ) | (SPI_MODE_11 << 2 )  | (SPI_ORDER_MSB << 5 );

	if (OK != transfer(&data[0],2, nullptr, 0)) {
			perf_count(_comms_errors);
			return -EIO;
	}
	return OK;
}

void
SPV1020::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("report queue:   %u (%u/%u @ %p)\n",
	       _num_reports, _oldest_report, _next_report, _reports);
	for(uint8_t mppt_device=0; mppt_device<max_mppt_devices ;mppt_device++){
		printf("MPPT device %d: Current: %10.4f[A], Voltage: %10.4f[v], PWM: %d, Status vector: %x, \n", mppt_device, (double) mppt_current[mppt_device] , (double) mppt_voltage[mppt_device] , mppt_pwm[mppt_device], mppt_status[mppt_device]);
	}
}

/**
 * Local functions in support of the shell command.
 */
namespace spv1020
{

SPV1020	*g_dev;

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
	g_dev = new SPV1020(SPV1020_BUS);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(SPV1020_DEVICE_PATH, O_RDONLY);

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
	struct spv1020_report report;
	ssize_t sz;
	int ret;

	int fd = open(SPV1020_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'SPV1020 start' if the driver is not running)", SPV1020_DEVICE_PATH);

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report))
		err(1, "immediate read failed");

	warnx("single read");

	for(uint8_t mppt_device=0; mppt_device < max_mppt_devices; mppt_device++){
		warnx("MPPT device %d: Current: %2.4f[A], Voltage: %2.4f[v], PWM: %d, Status vector: %x", mppt_device, (double) report.current[mppt_device], (double) report.voltage[mppt_device], report.pwm[mppt_device], report.status[mppt_device]);
	}

	warnx("time:        %lld", report.timestamp);

	/* set the queue depth to 10 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10))
		errx(1, "failed to set queue depth");

	/* start the sensor polling at 1Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 1))
		errx(1, "failed to set 1Hz poll rate");

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

		for(uint8_t mppt_device=0; mppt_device < max_mppt_devices; mppt_device++){
			warnx("MPPT device %d: Current: %2.4f[A], Voltage: %2.4f[v], PWM: %d, Status vector: %x", mppt_device, (double) report.current[mppt_device], (double) report.voltage[mppt_device] , report.pwm[mppt_device], report.status[mppt_device]);
		}
		warnx("time:        %lld", report.timestamp);
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(SPV1020_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "driver poll restart failed");

	if (ioctl(fd, MPPTSETCOMBRIDGE, 0) < 0)
		err(1, "driver set com bridge failed");

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
spv1020_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")){
		if (!strcmp(argv[2], "-d")){
			unsigned s = strtoul(argv[3], NULL, 10);
			if (s > MAX_NUMBER_OF_MPPT_DEVICES)
				errx(1, "Error the value for the maximal number of the MPPT devices (%d) is out of range (1..3)...exiting.\n ", s);
			else
				max_mppt_devices = (uint8_t) s;
		}
		else
			errx(1, "spv1020: unrecognized flag spv1020 {start} [-d <additional params>]}...exiting.\n");

		spv1020::start();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		spv1020::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		spv1020::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info"))
		spv1020::info();

	errx(1, "Unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
