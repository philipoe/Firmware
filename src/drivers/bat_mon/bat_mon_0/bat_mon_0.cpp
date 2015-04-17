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
 * @file bat_mon_0.cpp
 * Driver for the BAT_MON_0 sensor connected via I2C.
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

#include <systemlib/err.h>
#include <systemlib/perf_counter.h>

#include <drivers/drv_bat_mon.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_bat_mon.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/system_power.h>

#include <drivers/bat_mon/bat_mon.h>

#include "../bq78350.h"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif

/* Measurement rate is 2Hz */
#define MEAS_RATE 2
#define CONVERSION_INTERVAL	(1000000 / MEAS_RATE)	/* microseconds */

#define SERIAL_NUMBER_BAT_MON_0 1

class Bat_mon_0 : public Bat_mon
{
public:
	Bat_mon_0(int bus, int address = (SMBTAR_ADDCONF >> 1), const char *path = BAT_MON_0_DEVICE_PATH);//(int bus);

protected:

	virtual void cycle();
	virtual int	measure();
	virtual int	collect();
	virtual int	deviceserialnumber();

	/**
	 * Send a SBS command and read back a two bytes
	 */
	int getTwoBytesSBSReading(uint8_t sbscmd, uint16_t *sbsreading);

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int bat_mon_0_main(int argc, char *argv[]);


Bat_mon_0::Bat_mon_0(int bus, int address, const char *path) : Bat_mon(bus, address, CONVERSION_INTERVAL, path)
{
}

/* collect Battery monitor measurements: */
int
Bat_mon_0::measure()
{
	/* read the most recent measurement */
	perf_begin(_sample_perf);

	struct sensor_bat_mon_s report;


	if (OK != getTwoBytesSBSReading(TEMPERATURE, &_temperature)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != getTwoBytesSBSReading(VOLTAGE, &_voltage)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != getTwoBytesSBSReading(CURRENT, &_current)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != getTwoBytesSBSReading(BATTERYSTATUS, &_batterystatus)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != getTwoBytesSBSReading(SERIALNUMBER, &_serialnumber)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != getTwoBytesSBSReading(HOSTFETCONTROL, &_hostfetcontrol)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != getTwoBytesSBSReading(CELLVOLTAGE1, &_cellvoltage1)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != getTwoBytesSBSReading(CELLVOLTAGE2, &_cellvoltage2)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != getTwoBytesSBSReading(CELLVOLTAGE3, &_cellvoltage3)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != getTwoBytesSBSReading(CELLVOLTAGE4, &_cellvoltage4)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != getTwoBytesSBSReading(CELLVOLTAGE5, &_cellvoltage5)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != getTwoBytesSBSReading(CELLVOLTAGE6, &_cellvoltage6)){
		perf_count(_comms_errors);
		return -EIO;
	}


	/* generate a new report */
	report.timestamp 		= hrt_absolute_time();		/* report timestamp		*/
	report.temperature	  	= _temperature;				/* report in [0.1 K]  	*/
	report.voltage 		  	= _voltage;					/* report in [mA] 	   	*/
	report.current 		  	= _current;					/* report in Hex word  	*/
	report.batterystatus  	= _batterystatus;			/* report in Hex word  	*/
	report.serialnumber   	= _serialnumber;			/* report in uint word 	*/
	report.hostfetcontrol 	= _hostfetcontrol;			/* report in Hex word  	*/
	report.cellvoltage1  	= _cellvoltage1;			/* report in [mv]		*/
	report.cellvoltage2   	= _cellvoltage2;			/* report in [mv] 		*/
	report.cellvoltage3   	= _cellvoltage3;			/* report in [mv] 		*/
	report.cellvoltage4   	= _cellvoltage4;			/* report in [mv] 		*/
	report.cellvoltage5  	= _cellvoltage5;			/* report in [mv] 		*/
	report.cellvoltage6   	= _cellvoltage6;			/* report in [mv]		*/


	//warnx("measurements Bat_mon_0 board sensor: %6d %6d %6d %6d %6d %6d %6d %6d %6d %6d %6d %6d",_temperature, _voltage, _current, _batterystatus, _serialnumber, _hostfetcontrol, _cellvoltage1, _cellvoltage2, _cellvoltage3, _cellvoltage4, _cellvoltage5, _cellvoltage6);


	if (_bat_mon_pub_0 > 0 && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_bat_mon_0), _bat_mon_pub_0, &report);
	}

	new_report(report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	perf_end(_sample_perf);

	return OK;
}

int
Bat_mon_0::collect()
{
	return OK;
}

void
Bat_mon_0::cycle()
{

	/* collection phase? */
	if (_measurement_phase) {
		/* perform voltage measurement */
		if (OK != measure()) {
			start();
			return;
		}

		/* next phase is measurement */
		_measurement_phase = true;

		/* schedule a fresh cycle call when the measurement is done */
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&Bat_mon_0::cycle_trampoline,
			   this,
			   _measure_ticks);
	}
}

/* Two Bytes of SBS command reading */
int
Bat_mon_0::getTwoBytesSBSReading(uint8_t sbscmd, uint16_t *sbsreading)
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

int
Bat_mon_0::deviceserialnumber()
{
	if (OK != getTwoBytesSBSReading(SERIALNUMBER, &_serialnumber)){
		perf_count(_comms_errors);
		return -EIO;
	}

	if (_serialnumber == SERIAL_NUMBER_BAT_MON_0)
		warnx("Bat_mon_0 board sensor serial number: %d", _serialnumber);
	else
		return -EIO;

	return OK;
}

/**
 * Local functions in support of the shell command.
 */
namespace bat_mon_0
{

Bat_mon_0	*g_dev;

void	start(int i2c_bus);
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start(int i2c_bus)
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */   //BAT_MON_0_BUS
	g_dev = new Bat_mon_0(i2c_bus, (SMBTAR_ADDCONF >> 1), BAT_MON_0_DEVICE_PATH);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->Bat_mon::init()){
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(BAT_MON_0_DEVICE_PATH, O_RDONLY);

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

	errx(1, "driver bat_mon_0 start failed");
}

void
test()
{
	struct bat_mon_report report;
	ssize_t sz;
	int ret;

	int fd = open(BAT_MON_0_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'BAT_MON_0 start' if the driver is not running)", BAT_MON_0_DEVICE_PATH);

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
	int fd = open(BAT_MON_0_DEVICE_PATH, O_RDONLY);

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

static void
bat_mon_0_usage()
{
	warnx("usage: bat_mon_0 command [options]");
	warnx("options:");
	warnx("\t-b --bus i2cbus (%d)", PX4_I2C_BUS_DEFAULT);
	warnx("command:");
	warnx("\tstart|stop|reset|test|info");
}

int
bat_mon_0_main(int argc, char *argv[])
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
		bat_mon_0::start(i2c_bus);

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		bat_mon_0::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		bat_mon_0::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		bat_mon_0::info();
	}

	//errx(1, "unrecognised command, try 'start', 'test', 'reset' or 'info'");
	bat_mon_0_usage();
	exit(0);
}
