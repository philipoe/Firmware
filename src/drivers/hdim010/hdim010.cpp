/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 *
 */

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
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

#include <systemlib/airspeed.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>

#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include <drivers/drv_airspeed.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/subsystem_info.h>
#include <drivers/airspeed/airspeed.h>


/* I2C bus address */
#define I2C_ADDRESS_HDIM010	0x78

#define PATH_HDIM010		"/dev/hdim010"

/* Measurement rate is 50Hz */
#define MEAS_RATE 50
#define MEAS_DRIVER_FILTER_FREQ 5.0f
#define CONVERSION_INTERVAL	(1000000 / MEAS_RATE)			/* microseconds */


/* Conversion factors */
#define HDIM010_SENSITIVITY				1092.2f				/* Sensitivity of the sensor S = (24575-2731)/(10- (-10)) 	*/
#define HDIM010_PRESSURE_OUTPUT_MIN		2731.0f				/* Output at minimum specified pressure 					*/
#define HDIM010_PRESSURE_VALUE_MIN      -10.0f				/* Min. value of pressure range [mbar]						*/

class HDIM010 : public Airspeed
{
public:
	HDIM010(int bus, int address = I2C_ADDRESS_HDIM010, const char* path = PATH_HDIM010);

protected:

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	virtual void	cycle();
	virtual int		measure();
	virtual int		collect();

	math::LowPassFilter2p	_filter;
	bool 					_probe_phase;

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int hdim010_main(int argc, char *argv[]);

HDIM010::HDIM010(int bus, int address, const char* path) : Airspeed(bus, address,
	CONVERSION_INTERVAL, path),
	_filter(MEAS_RATE, MEAS_DRIVER_FILTER_FREQ),
	_probe_phase(true)
{
}

int
HDIM010::measure()
{
	// Check hdim010 sensor response
	if (_probe_phase){
		uint8_t data[2];
		if (OK != transfer(nullptr, 0, &data[0], 2)) {
			return -EIO;
		}
		else
			_probe_phase = false;
	}

	return OK;
}

int
HDIM010::collect()
{

	uint8_t data[2];
	union {
		uint8_t	b[2];
		uint32_t w;
	} cvt;

	float dPressure = 0.0f;

	/* read the most recent measurement */
	perf_begin(_sample_perf);

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

	/* convert mbar to pa					*/
	dPressure *= 100.0f;

	/* reduce measurement offset 			*/
	//dPressure -= _diff_pres_offset;

	/* Correct measurement offset and SF 	*/
	dPressure = (dPressure - _diff_pres_offset)*_diff_pres_scale;

	/* Range check /failure accordingly */
	if ( (dPressure > 1000.0f) | (dPressure < -1000.0f) ) {
		warnx("HDIM010: Differential pressure is out of range: %3.6f [Pa]", (double) dPressure);
		return -EIO;
	}

	struct differential_pressure_s report;

	/* track maximum differential pressure measured (so we can work out top speed). */
	if (dPressure > _max_differential_pressure_pa) {
		_max_differential_pressure_pa = dPressure;
	}

	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);
	report.temperature = -1000.0f;
	report.differential_pressure_filtered_pa =  _filter.apply(dPressure);

	report.differential_pressure_raw_pa = dPressure;
	report.max_differential_pressure_pa = _max_differential_pressure_pa;

	//warnx("calculated effective differential pressure %3.6f [Pa]", (double) dPressure);   	    // removed display

	if (_airspeed_pub > 0 && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(differential_pressure), _airspeed_pub, &report);
	}

	new_report(report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	perf_end(_sample_perf);

	return OK;
}

void
HDIM010::cycle()
{
	int ret;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();
		if (OK != ret) {
			perf_count(_comms_errors);

			/* restart the measurement state machine */
			start();
			_sensor_ok = false;
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&Airspeed::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	ret = measure();
	if (OK != ret) {
		debug("measure error");
	}

	_sensor_ok = (ret == OK);

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&Airspeed::cycle_trampoline,
		   this,
		   USEC2TICK(CONVERSION_INTERVAL));
}

/**
 * Local functions in support of the shell command.
 */
namespace hdim010
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

HDIM010	*g_dev;

void	start(int i2c_bus);
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */
void
start(int i2c_bus)
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new HDIM010(i2c_bus);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->Airspeed::init())
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

	errx(1, "no HDIM010 airspeed sensor connected");
}

/**
 * Stop the driver
 */
void
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	struct differential_pressure_s report;
	ssize_t sz;
	int ret;

	int fd = open(AIRSPEED_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'hdim010 start' if the driver is not running", AIRSPEED_DEVICE_PATH);

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report))
		err(1, "immediate read failed");

	warnx("single read");
	warnx("Effective differential pressure %3.3f [Pa]", (double)report.differential_pressure_filtered_pa);


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

		warnx("periodic read %u", i);
		warnx("Effective differential pressure %3.3f [Pa]", (double)report.differential_pressure_filtered_pa);
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

}

static void
hdim010_airspeed_usage()
{
	warnx("usage: hdim010 command [options]");
	warnx("options:");
	warnx("\t-b --bus i2cbus (%d)", PX4_I2C_BUS_DEFAULT);
	warnx("command:");
	warnx("\tstart|stop|reset|test|info");
}

int
hdim010_main(int argc, char *argv[])
{
	int i2c_bus = PX4_I2C_BUS_DEFAULT;

	int i;

	for (i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--bus") == 0) {
			if (argc > i + 1) {
				i2c_bus = atoi(argv[i + 1]);
			}
		}
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		hdim010::start(i2c_bus);

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop"))
		hdim010::stop();

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
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status"))
		hdim010::info();

	hdim010_airspeed_usage();
	exit(0);
}
