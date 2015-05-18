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
 * @file ledcnt.cpp
 * Driver for the LEDCNT sensor connected via I2C.
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

#include <drivers/drv_ledcnt.h>


/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif


class LEDCNT : public device::I2C
{
public:
	LEDCNT(int bus);
	~LEDCNT();

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
	struct ledcnt_report	*_reports;

	bool				_measurement_phase;


	orb_advert_t		_ledcnt_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;


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
	 * Get LED controller regs command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			get_ledcnt_regs();

	/**
	 * Test communication for the LED driver board
	 *
	 */

	int 		test_ledcnt_communication(void);

	/**
	 * WD functionality for the LED driver board
	 *
	 */

	int 		wd_ledcnt(void);

};

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { _x++; if (_x >= _lim) _x = 0; } while(0)

/*
 * LEDCNT internal constants and data structures.
 */

/* internal conversion time: 10 ms  */
#define LEDCNT_CONVERSION_INTERVAL	100000	/* microseconds */

#define LEDCNT_BUS				PX4_I2C_BUS_EXPANSION

#define LEDCNT_ADDRESS			0x49    /* LEDCNT I2C address 									   				*/

#define LED_STATS_REG			0x00	/* Pointer to the address of the led board status register 				*/
#define LED_BLINK_REG			0x01	/* Pointer to the address of the led board blink register  				*/
#define LED_POW_1_REG			0x02	/* Pointer to the address of the led board power register of channel 1  */
#define LED_POW_2_REG			0x03	/* Pointer to the address of the led board power register of channel 2  */
#define LED_POW_3_REG			0x04	/* Pointer to the address of the led board power register of channel 3  */
#define LED_POW_4_REG			0x05	/* Pointer to the address of the led board power register of channel 4  */


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ledcnt_main(int argc, char *argv[]);


LEDCNT::LEDCNT(int bus) :
	I2C("LEDCNT", LEDCNT_DEVICE_PATH, bus, 0, 400000),
	_measure_ticks(0),
	_num_reports(0),
	_next_report(0),
	_oldest_report(0),
	_reports(nullptr),
	_measurement_phase(false),
	_ledcnt_topic(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "LEDCNT_read")),
	_comms_errors(perf_alloc(PC_COUNT, "LEDCNT_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "LEDCNT_buffer_overflows"))

{
	// enable debug() calls
	_debug_enabled = true;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

LEDCNT::~LEDCNT()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete[] _reports;
}

int
LEDCNT::init()
{
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_num_reports = 2;
	_reports = new struct ledcnt_report[_num_reports];

	if (_reports == nullptr)
		goto out;

	_oldest_report = _next_report = 0;

	/* get a publish handle on the ledcnt topic */
	memset(&_reports[0], 0, sizeof(_reports[0]));
	_ledcnt_topic = orb_advertise(ORB_ID(sensor_ledcnt), &_reports[0]);

	if (_ledcnt_topic < 0)
		debug("failed to create sensor_ledcnt object");

	ret = OK;
out:
	return ret;
}

int
LEDCNT::probe()
{
	_retries = 10;

	if (OK == probe_address(LEDCNT_ADDRESS)){
		_retries = 1;
		return OK;
	}

	return -EIO;
}

int
LEDCNT::probe_address(uint8_t address)
{
	set_address(address);

	/* send reset command */
	if (OK != test_ledcnt_communication())
		return -EIO;

	return OK;
}

ssize_t
LEDCNT::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct ledcnt_report);
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

		/* Get LED controller regs */

		if (OK != get_ledcnt_regs()) {
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
LEDCNT::ioctl(struct file *filp, int cmd, unsigned long arg)
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
					_measure_ticks = USEC2TICK(LEDCNT_CONVERSION_INTERVAL);

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
					if (ticks < USEC2TICK(LEDCNT_CONVERSION_INTERVAL))
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
			struct ledcnt_report *buf = new struct ledcnt_report[arg];

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
LEDCNT::start()
{
	/* reset the report ring and state machine */
	_measurement_phase = true;
	_oldest_report = _next_report = 0;

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&LEDCNT::cycle_trampoline, this, 1);
}

void
LEDCNT::stop()
{
	work_cancel(HPWORK, &_work);
}

void
LEDCNT::cycle_trampoline(void *arg)
{
	LEDCNT *dev = (LEDCNT *)arg;

	dev->cycle();
}

void
LEDCNT::cycle()
{

	/* collection phase? */
	if (_measurement_phase) {
#if 1
		wd_ledcnt();					/// temp version only for the 24h AS flight

#else

		if (OK != wd_ledcnt()) {
																					// temp commented out for debugging

			log("Led board error, restarting LEDCNT device");

			/* free any existing reports and reset the state machine and try again */
			if (_reports != nullptr)
				delete[] _reports;
			probe();

			start();
			return;
		}
#endif
		/* next phase is measurement */
		_measurement_phase = true;

		/* schedule a fresh cycle call when the measurement is done */
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&LEDCNT::cycle_trampoline,
			   this,
			   USEC2TICK(LEDCNT_CONVERSION_INTERVAL));				// temp marked for testing the replacement of LEDCNT_CONVERSION_INTERVAL
		   	   //_measure_ticks);										// LEDCNT_CONVERSION_INTERVAL replaced with _measure_ticks for init the rate form sesnors.c

	}
}

/* Get LED controller registers: */
int
LEDCNT::get_ledcnt_regs()
{
	uint8_t ptr = LED_STATS_REG;
	uint8_t data[1];

	uint8_t led_status  = 0;
	uint8_t led_blink 	= 0;
	uint8_t led_power_1 = 0;
	uint8_t led_power_2 = 0;
	uint8_t led_power_3 = 0;
	uint8_t led_power_4 = 0;

	/* read the most recent measurement */
	perf_begin(_sample_perf);


	if (OK != transfer(&ptr, 1, &data[0], 1)) {
		perf_count(_comms_errors);
		return -EIO;
	}
	led_status =  data[0];

	ptr = LED_BLINK_REG;
	if (OK != transfer(&ptr, 1, &data[0], 1)) {
		perf_count(_comms_errors);
		return -EIO;
	}
	led_blink =  data[0];


	ptr = LED_POW_1_REG;
	if (OK != transfer(&ptr, 1, &data[0], 1)) {
		perf_count(_comms_errors);
		return -EIO;
	}
	led_power_1 =  data[0];

	ptr = LED_POW_2_REG;
	if (OK != transfer(&ptr, 1, &data[0], 1)) {
		perf_count(_comms_errors);
		return -EIO;
	}
	led_power_2 =  data[0];

	ptr = LED_POW_3_REG;
	if (OK != transfer(&ptr, 1, &data[0], 1)) {
		perf_count(_comms_errors);
		return -EIO;
	}
	led_power_3 =  data[0];

	ptr = LED_POW_4_REG;
	if (OK != transfer(&ptr, 1, &data[0], 1)) {
		perf_count(_comms_errors);
		return -EIO;
	}
	led_power_4 =  data[0];


	/* generate a new report */
	_reports[_next_report].timestamp   = hrt_absolute_time();
	_reports[_next_report].led_status  = led_status;
	_reports[_next_report].led_blink   = led_blink;
	_reports[_next_report].led_power_1 = led_power_1;
	_reports[_next_report].led_power_2 = led_power_2;
	_reports[_next_report].led_power_3 = led_power_3;
	_reports[_next_report].led_power_4 = led_power_4;

	/* publish it */
	orb_publish(ORB_ID(sensor_ledcnt), _ledcnt_topic, &_reports[_next_report]);

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
LEDCNT::test_ledcnt_communication(void)
{
	uint8_t data[2];

	data[1] = 0x55;
	data[0] = 0x06;

	if (OK != transfer(&data[0],2, nullptr, 0))
		return -EIO;

	return OK;
}

int
LEDCNT::wd_ledcnt(void)
{
	uint8_t data[2];

	data[1] = 0x55;
	data[0] = 0x06;

	if (OK != transfer(&data[0],2, nullptr, 0))
		return -EIO;

	return OK;
}

void
LEDCNT::print_info()
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
namespace ledcnt
{

LEDCNT	*g_dev;

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
	g_dev = new LEDCNT(LEDCNT_BUS);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(LEDCNT_DEVICE_PATH, O_RDONLY);

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
	struct ledcnt_report report;
	ssize_t sz;
	int ret;

	int fd = open(LEDCNT_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'LEDCNT start' if the driver is not running)", LEDCNT_DEVICE_PATH);

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report))
		err(1, "immediate read failed");

	warnx("single read");
	warnx("LED status:   %x", report.led_status);
	warnx("time:         %lld", report.timestamp);

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

		warnx("LED status:   %x", report.led_status);
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
	int fd = open(LEDCNT_DEVICE_PATH, O_RDONLY);

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
ledcnt_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		ledcnt::start();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		ledcnt::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		ledcnt::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info"))
		ledcnt::info();

	errx(1, "unrecognised command, try 'start', 'test', 'reset' or 'info'");
}
