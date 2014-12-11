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
 * @file hdim010.cpp
 * Driver for the HDIM010 differential barometric pressure sensor connected via I2C.
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

#include <drivers/drv_dbaro.h>

#include <uORB/topics/hdim010_i2c_wd.h>														/* added for the I2C WD */



/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif


class HDIM010 : public device::I2C
{
public:
	HDIM010(int bus);
	~HDIM010();

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
	struct dbaro_report	*_reports;

	bool				_measurement_phase;

	float 				dPressure;
	//float 				temperature;

	orb_advert_t		_dbaro_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	struct hdim010_i2c_wd_s 	 _hdim010_i2c_wd_report;	         	/* added for the I2C WD */
	orb_advert_t		 		 _hdim010_i2c_wd_pub;					/* added for the I2C WD */

	uint8_t _hdim010_error_counter;										/* HDIM010 I2C reset	*/


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
	 * Issue a measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measure();

	/**
	 * Issue a dPresssure measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measurement();

};

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { _x++; if (_x >= _lim) _x = 0; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

/*
 * HDIM010 internal constants and data structures.
 */

/* internal conversion time:  */
#define HDIM010_CONVERSION_INTERVAL	50000							/* microseconds */

#define HDIM010_BUS				PX4_I2C_BUS_EXPANSION
#define HDIM010_ADDRESS			0x78   						     	/* HDIM010 I2C address   									*/

#define HDIM010_SENSITIVITY				1092.2f						/* Sensitivity of the sensor S = (24575-2731)/(10- (-10)) 	*/
#define HDIM010_PRESSURE_OUTPUT_MIN		2731.0f						/* Output at minimum specified pressure 					*/
#define HDIM010_PRESSURE_VALUE_MIN      -10.0f						/* Min. value of pressure range [mbar]						*/

#define HDIM010_MAXIMAL_ERROR_COUNTER	5	/* Added for the HDIM010 reset (number of error required for the I2C reset) */    //// <<<--- checkb

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int hdim010_main(int argc, char *argv[]);


HDIM010::HDIM010(int bus) :
	I2C("HDIM010", DBARO_DEVICE_PATH, bus, 0, 400000),				/* I2C clock frequency 400KHz 								*/
	_measure_ticks(0),
	_num_reports(0),
	_next_report(0),
	_oldest_report(0),
	_reports(nullptr),
	_measurement_phase(false),
	_dbaro_topic(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "hdim010_read")),
	_comms_errors(perf_alloc(PC_COUNT, "hdim010_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "hdim010_buffer_overflows")),
	_hdim010_error_counter(0)														/* HDIM010 I2C reset		*/  // added test
{
	// enable debug() calls
	_debug_enabled = true;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

HDIM010::~HDIM010()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete[] _reports;
}

int
HDIM010::init()
{

	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_num_reports = 2;
	_reports = new struct dbaro_report[_num_reports];

	if (_reports == nullptr)
		goto out;

	_oldest_report = _next_report = 0;

	/* get a publish handle on the dbaro topic */
	memset(&_reports[0], 0, sizeof(_reports[0]));
	_dbaro_topic = orb_advertise(ORB_ID(sensor_dbaro), &_reports[0]);

	_hdim010_i2c_wd_report.hdim010_i2c_error = false;
	_hdim010_i2c_wd_pub = orb_advertise(ORB_ID(hdim010_i2c_wd), &_hdim010_i2c_wd_report);


	if (_dbaro_topic < 0)
		debug("failed to create sensor_dbaro object");

	ret = OK;
out:
	return ret;
}

int
HDIM010::probe()
{
	_retries = 10;

	if (OK == probe_address(HDIM010_ADDRESS)){
		_retries = 1;
		return OK;
	}

	return -EIO;
}

int
HDIM010::probe_address(uint8_t address)
{
	set_address(address);

	/* send reset command */
	if (OK != measurement())
		return -EIO;

	return OK;
}

ssize_t
HDIM010::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct dbaro_report);
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
HDIM010::ioctl(struct file *filp, int cmd, unsigned long arg)
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
					_measure_ticks = USEC2TICK(HDIM010_CONVERSION_INTERVAL);

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
					if (ticks < USEC2TICK(HDIM010_CONVERSION_INTERVAL))
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
			struct dbaro_report *buf = new struct dbaro_report[arg];

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
	default:
		break;
	}

	/* give it to the superclass */
	return I2C::ioctl(filp, cmd, arg);
}

void
HDIM010::start()
{
	/* reset the report ring and state machine */
	_measurement_phase = true;
	_oldest_report = _next_report = 0;

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&HDIM010::cycle_trampoline, this, 1);
}

void
HDIM010::stop()
{
	work_cancel(HPWORK, &_work);
}

void
HDIM010::cycle_trampoline(void *arg)
{
	HDIM010 *dev = (HDIM010 *)arg;
	dev->cycle();
}

void
HDIM010::cycle()
{
	/* collection phase? */
	if (_measurement_phase) {
		/* perform dpressure measurement */
		if (OK != measurement()) {
#if 0															// temp commented out for debugging
			log("dpressure measurement error, restarting HDIM010...");

			/* free any existing reports and reset the state machine and try again */
			if (_reports != nullptr)
				delete[] _reports;
			probe();
#endif
			// added for testing
			_hdim010_error_counter++;
			if (_hdim010_error_counter > HDIM010_MAXIMAL_ERROR_COUNTER){
				_hdim010_i2c_wd_report.hdim010_i2c_error = true;
				orb_publish(ORB_ID(hdim010_i2c_wd), _hdim010_i2c_wd_pub, &_hdim010_i2c_wd_report.hdim010_i2c_error);
				usleep(250000);
				_hdim010_error_counter = 0;
			}
			// till here ----->>

			//_hdim010_i2c_wd_report.hdim010_i2c_error = true;														// temp marked
			//orb_publish(ORB_ID(hdim010_i2c_wd), _hdim010_i2c_wd_pub, &_hdim010_i2c_wd_report.hdim010_i2c_error);	// temp marked
			//usleep(250000);																						// temp marked

			start();
			return;
		}
		_hdim010_error_counter = 0;						/* testing: Reset only after 10 consecutive errors */

		/* Ideal command to the I2C WD board */
		_hdim010_i2c_wd_report.hdim010_i2c_error = false;
		orb_publish(ORB_ID(hdim010_i2c_wd), _hdim010_i2c_wd_pub, &_hdim010_i2c_wd_report.hdim010_i2c_error);


		/* next phase is measurement */
		_measurement_phase = true;

		/* schedule a fresh cycle call when the measurement is done */
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&HDIM010::cycle_trampoline,
			   this,
			   //USEC2TICK(HDIM010_CONVERSION_INTERVAL));			// temp marked for testing the replacement of HDIM010_CONVERSION_INTERVAL
			   _measure_ticks);										// HDIM010_CONVERSION_INTERVAL replaced with _measure_ticks for init the rate form sesnors.c
	}
}

int
HDIM010::measurement()
{
	uint8_t data[2];
	union {
		uint8_t	b[2];
		uint32_t w;
	} cvt;

	/* read the most recent measurement */
	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	_reports[_next_report].timestamp = hrt_absolute_time();

	/* fetch the raw value */

	if (OK != transfer(nullptr, 0, &data[0], 2)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* assemble 16 bit value and convert from big endian (sensor) to little endian (MCU) */
	cvt.b[0] = data[1];
	cvt.b[1] = data[0];

	/* pressure calculation, result in mbar */
	dPressure = (((float)((int16_t)(cvt.w & 0x7fff))) - HDIM010_PRESSURE_OUTPUT_MIN)/HDIM010_SENSITIVITY +(HDIM010_PRESSURE_VALUE_MIN);

	/* Range check /failure accordingly */
	if ( (dPressure > 10.0f) | (dPressure < -10.0f) ) {
		warnx("HDIM010: Differential pressure is out of range: %3.6f [mbar]", (double) dPressure);
		return -EIO;
		}

	//warnx("calculated effective differential pressure %3.6f [mbar]", (double) dPressure);   				// removed display
	//warnx("calculated effective differential pressure %3.6f [Pa]", (double) dPressure * 100.0f);   	    // removed display


	/* generate a new report */
	_reports[_next_report].dpressure = dPressure * 100.0f;					/* report in Pa */

	/* publish it */
	orb_publish(ORB_ID(sensor_dbaro), _dbaro_topic, &_reports[_next_report]);

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

void
HDIM010::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("report queue:   %u (%u/%u @ %p)\n",
	       _num_reports, _oldest_report, _next_report, _reports);
	printf("Differential Pressure:   %10.4f [mbar]\n", (double)(dPressure));
}

/**
 * Local functions in support of the shell command.
 */
namespace hdim010
{

HDIM010	*g_dev;

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
	g_dev = new HDIM010(HDIM010_BUS);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(DBARO_DEVICE_PATH, O_RDONLY);

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
	struct dbaro_report report;
	ssize_t sz;
	int ret;

	int fd = open(DBARO_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'HDIM010 start' if the driver is not running)", DBARO_DEVICE_PATH);

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report))
		err(1, "immediate read failed");

	warnx("single read");
	warnx("dpressure:   %10.4f [Pa]", (double)report.dpressure);
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

		warnx("dpressure:   %10.4f [Pa]", (double)report.dpressure);
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
	int fd = open(DBARO_DEVICE_PATH, O_RDONLY);

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
hdim010_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		hdim010::start();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		hdim010::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		hdim010::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info"))
		hdim010::info();

	errx(1, "unrecognised command, try 'start', 'test', 'reset' or 'info'");
}
