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
 * @file bat_mon.cpp
 * Driver for the BAT_MON voltage sensor connected via I2C.
 *
 * Author: Amir Melzer  <amir.melzer@mavt.ethz.ch>
 * 		   Lorenz Meier <lm@inf.ethz.ch>
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

#include <drivers/drv_bat_mon.h>

#include "bq78350.h"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif


class BAT_MON : public device::I2C
{
public:
	BAT_MON(int bus);
	virtual ~BAT_MON();

	virtual int		init();

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	virtual int		probe();

private:

	struct work_s				_work;
	unsigned					_measure_ticks;

	unsigned					_num_reports;
	volatile unsigned			_next_report;
	volatile unsigned			_oldest_report;
	struct bat_mon_report		*_reports;

	bool						_measurement_phase;

	uint16_t	  				_temperature;
	uint16_t	  				_voltage;
	uint16_t	  				_current;
	uint16_t	  				_batterystatus;
	uint16_t	  				_serialnumber;
	uint16_t	  				_hostfetcontrol;
	uint16_t	  				_cellvoltage1;
	uint16_t	  				_cellvoltage2;
	uint16_t	  				_cellvoltage3;
	uint16_t	  				_cellvoltage4;
	uint16_t	  				_cellvoltage5;
	uint16_t	  				_cellvoltage6;

	orb_advert_t				_bat_mon_sensor_pb_topic;

	perf_counter_t				_sample_perf;
	perf_counter_t				_comms_errors;
	perf_counter_t				_buffer_overflows;

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
	 * @note This function is called at open and error time. It might make sense
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
	 * This is the measurement state machine. This function
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
	 * Issue a voltage measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			voltage_measurement();

	/**
	 * Issue a SBS command for and read back a single byte
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			get_OneBytesSBSReading(uint8_t sbscmd, uint8_t sbsreading);

	/**
	 * Issue a SBS command for and read back two byte
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			get_TwoBytesSBSReading(uint8_t sbscmd, uint16_t *sbsreading);

	/**
	 * collect battery monitor measurements
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			bat_mon_measurement();


};

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { _x++; if (_x >= _lim) _x = 0; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

/*
 * BAT_MON internal constants and data structures.
 */

/* internal conversion time: 100 ms  */
#define BAT_MON_CONVERSION_INTERVAL	100000	/* microseconds */

#define BAT_MON_BUS			PX4_I2C_BUS_EXPANSION

/* Measurement definitions: */

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int bat_mon_main(int argc, char *argv[]);


BAT_MON::BAT_MON(int bus) :
	I2C("BAT_MON", BAT_MON_DEVICE_PATH, bus, 0, 100000),							/* set I2C rate to 100KHz */
	_measure_ticks(0),
	_num_reports(0),
	_next_report(0),
	_oldest_report(0),
	_reports(nullptr),
	_measurement_phase(false),
     _temperature(0),
    _voltage(0),
    _current(0),
    _batterystatus(0),
    _serialnumber(0),
    _hostfetcontrol(0),
    _cellvoltage1(0),
    _cellvoltage2(0),
    _cellvoltage3(0),
    _cellvoltage4(0),
    _cellvoltage5(0),
    _cellvoltage6(0),
	_bat_mon_sensor_pb_topic(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "BAT_MON_read")),
	_comms_errors(perf_alloc(PC_COUNT, "BAT_MON_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "BAT_MON_buffer_overflows"))
{
	_debug_enabled = true;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));

}

BAT_MON::~BAT_MON()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete[] _reports;
}

int
BAT_MON::init()
{
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_num_reports = 2;
	_reports = new struct bat_mon_report[_num_reports];

	if (_reports == nullptr)
		goto out;

	_oldest_report = _next_report = 0;

	/* get a publish handle on the bat_mon topic */
	memset(&_reports[0], 0, sizeof(_reports[0]));
	_bat_mon_sensor_pb_topic = orb_advertise(ORB_ID(sensor_bat_mon), &_reports[0]);

	if (_bat_mon_sensor_pb_topic < 0)
		debug("failed to create sensor_bat_mon object");

	ret = OK;
out:
	return ret;
}

int
BAT_MON::probe()
{
	_retries = 10;

	if (OK == probe_address(SMBTAR_ADDCONF)){		///(BAT_MON_ADDRESS)){
		_retries = 1;
		return OK;
	}

	return -EIO;
}

int
BAT_MON::probe_address(uint8_t address)
{
	set_address(address >> 1);

	/* send reset command */
	if (OK != get_TwoBytesSBSReading(SERIALNUMBER, &_serialnumber))
		return -EIO;

	/* Initialization functions: */

	return OK;
}

ssize_t
BAT_MON::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct bat_mon_report);
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

		/* Take a battery monitor measurement */
		if (OK != bat_mon_measurement()){
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
BAT_MON::ioctl(struct file *filp, int cmd, unsigned long arg)
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
					_measure_ticks = USEC2TICK(BAT_MON_CONVERSION_INTERVAL);

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
					if (ticks < USEC2TICK(BAT_MON_CONVERSION_INTERVAL))
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
			struct bat_mon_report *buf = new struct bat_mon_report[arg];

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
BAT_MON::start()
{
	/* reset the report ring and state machine */
	_measurement_phase = true;
	_oldest_report = _next_report = 0;

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&BAT_MON::cycle_trampoline, this, 1);
}

void
BAT_MON::stop()
{
	work_cancel(HPWORK, &_work);
}

void
BAT_MON::cycle_trampoline(void *arg)
{
	BAT_MON *dev = (BAT_MON *)arg;

	dev->cycle();
}

void
BAT_MON::cycle()
{

	/* collection phase? */
	if (_measurement_phase) {
		/* perform voltage measurement */
		if (OK != bat_mon_measurement()) {
			start();
			return;
		}

		/* next phase is measurement */
		_measurement_phase = true;

		/* schedule a fresh cycle call when the measurement is done */
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&BAT_MON::cycle_trampoline,
			   this,
			   _measure_ticks);
	}
}

/* Single Bytes of SBS command Reading */
int
BAT_MON::get_OneBytesSBSReading(uint8_t sbscmd, uint8_t sbsreading)
{
	uint8_t data;

	/* fetch the raw value */
	if (OK != transfer(&sbscmd, 1, &data, 2)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	sbsreading = data;

return OK;
}

/* Two Bytes of SBS command Reading */
int
BAT_MON::get_TwoBytesSBSReading(uint8_t sbscmd, uint16_t *sbsreading)
{
	uint8_t data[2];
	union {
		uint8_t	b[2];
		uint16_t w;
	} cvt;

	/* fetch the raw value */
	if (OK != transfer(&sbscmd, 1, &data[0], 2)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* assemble 16 bit value */
	cvt.b[0] = data[0];
	cvt.b[1] = data[1];

	*sbsreading = cvt.w;

return OK;
}

/* collect Battery monitor measurements: */
int
BAT_MON::bat_mon_measurement()
{
	/* read the most recent measurement */
	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	_reports[_next_report].timestamp = hrt_absolute_time();


	if (OK != get_TwoBytesSBSReading(TEMPERATURE, &_temperature)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != get_TwoBytesSBSReading(VOLTAGE, &_voltage)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != get_TwoBytesSBSReading(CURRENT, &_current)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != get_TwoBytesSBSReading(BATTERYSTATUS, &_batterystatus)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != get_TwoBytesSBSReading(SERIALNUMBER, &_serialnumber)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != get_TwoBytesSBSReading(HOSTFETCONTROL, &_hostfetcontrol)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != get_TwoBytesSBSReading(CELLVOLTAGE1, &_cellvoltage1)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != get_TwoBytesSBSReading(CELLVOLTAGE2, &_cellvoltage2)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != get_TwoBytesSBSReading(CELLVOLTAGE3, &_cellvoltage3)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != get_TwoBytesSBSReading(CELLVOLTAGE4, &_cellvoltage4)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != get_TwoBytesSBSReading(CELLVOLTAGE5, &_cellvoltage5)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != get_TwoBytesSBSReading(CELLVOLTAGE6, &_cellvoltage6)){
		perf_count(_comms_errors);
		return -EIO;
	}


	/* generate a new report */
	_reports[_next_report].temperature	  	= _temperature;				/* report in [0.1 K]  	*/
	_reports[_next_report].voltage 		  	= _voltage;					/* report in [mA] 	   	*/
	_reports[_next_report].current 		  	= _current;					/* report in Hex word  	*/
	_reports[_next_report].batterystatus  	= _batterystatus;			/* report in Hex word  	*/
	_reports[_next_report].serialnumber   	= _serialnumber;			/* report in uint word 	*/
	_reports[_next_report].hostfetcontrol 	= _hostfetcontrol;			/* report in Hex word  	*/
	_reports[_next_report].cellvoltage1  	= _cellvoltage1;			/* report in [mv]		*/
	_reports[_next_report].cellvoltage2   	= _cellvoltage2;			/* report in [mv] 		*/
	_reports[_next_report].cellvoltage3   	= _cellvoltage3;			/* report in [mv] 		*/
	_reports[_next_report].cellvoltage4   	= _cellvoltage4;			/* report in [mv] 		*/
	_reports[_next_report].cellvoltage5  	= _cellvoltage5;			/* report in [mv] 		*/
	_reports[_next_report].cellvoltage6   	= _cellvoltage6;			/* report in [mv]		*/


	//warnx("measurements BAT_MON board sensor: %6d %6d %6d %6d %6d %6d %6d %6d %6d %6d %6d %6d",_temperature, _voltage, _current, _batterystatus, _serialnumber, _hostfetcontrol, _cellvoltage1, _cellvoltage2, _cellvoltage3, _cellvoltage4, _cellvoltage5, _cellvoltage6);

	/* publish it */
	orb_publish(ORB_ID(sensor_bat_mon), _bat_mon_sensor_pb_topic, &_reports[_next_report]);

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
BAT_MON::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("report queue:   %u (%u/%u @ %p)\n",
	       _num_reports, _oldest_report, _next_report, _reports);
}

/**
 * Local functions in support of the shell command.
 */
namespace bat_mon
{

BAT_MON	*g_dev;

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
	g_dev = new BAT_MON(BAT_MON_BUS);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(BAT_MON_DEVICE_PATH, O_RDONLY);

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
	struct bat_mon_report report;
	ssize_t sz;
	int ret;

	int fd = open(BAT_MON_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'BAT_MON start' if the driver is not running)", BAT_MON_DEVICE_PATH);

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report))
		err(1, "immediate read failed");

	warnx("single read");
	warnx("time:            %lld", report.timestamp);
	warnx("serialnumber:     %6d", report.serialnumber);
	warnx("temperature:      %6d", report.temperature);
	warnx("voltage:          %6d", report.voltage);
	warnx("current:   	     %6d", report.current);
	warnx("batterystatus:    %4x", report.batterystatus);
	warnx("host fet control: %4x", report.hostfetcontrol);
	warnx("cellvoltage1:     %6d", report.cellvoltage1);
	warnx("cellvoltage2:     %6d", report.cellvoltage2);
	warnx("cellvoltage3:     %6d", report.cellvoltage3);
	warnx("cellvoltage4:     %6d", report.cellvoltage4);
	warnx("cellvoltage5:     %6d", report.cellvoltage5);
	warnx("cellvoltage6:     %6d", report.cellvoltage6);


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

		warnx("time:            %lld", report.timestamp);
		warnx("serialnumber:     %6d", report.serialnumber);
		warnx("temperature:      %6d", report.temperature);
		warnx("voltage:          %6d", report.voltage);
		warnx("current:   	     %6d", report.current);
		warnx("batterystatus:    %4x", report.batterystatus);
		warnx("host fet control: %4x", report.hostfetcontrol);
		warnx("cellvoltage1:     %6d", report.cellvoltage1);
		warnx("cellvoltage2:     %6d", report.cellvoltage2);
		warnx("cellvoltage3:     %6d", report.cellvoltage3);
		warnx("cellvoltage4:     %6d", report.cellvoltage4);
		warnx("cellvoltage5:     %6d", report.cellvoltage5);
		warnx("cellvoltage6:     %6d", report.cellvoltage6);
	}

	reset();
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(BAT_MON_DEVICE_PATH, O_RDONLY);

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
bat_mon_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		bat_mon::start();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		bat_mon::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		bat_mon::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info"))
		bat_mon::info();

	errx(1, "unrecognised command, try 'start', 'test', 'reset' or 'info'");
}
