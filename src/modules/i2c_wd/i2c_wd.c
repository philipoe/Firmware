/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file i2c_wd.c
 * I2C WD driver for PX4 autopilot
 *
 * @author: Amir Melzer <amir.melzer@mavt.ethz.ch>
 */

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <board_config.h>
#include <uORB/uORB.h>
#include <uORB/topics/hdim010_i2c_wd.h>

#include <drivers/drv_gpio.h>


#define I2C_WD_INTERVAL_US 600000
#define RESET_TIME_US 	   200000
#define I2C_MAX_RESET_COUNTER 100

static bool thread_should_exit = false;		/**< daemon exit flag 				*/
static bool thread_running = false;			/**< daemon status flag 			*/
static int daemon_task;						/**< Handle of daemon task / thread */

int			_hdim010_i2c_wd_sub;			/**< I2C WD board subscription		*/

/**
 * daemon management function.
 */
__EXPORT int i2c_wd_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int i2c_wd_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
		errx(1, "usage: i2c_wd {start|stop|status} [-p <additional params>]\n\n");
}

int i2c_wd_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("i2c_wd is already running\n");
			/* this is not an error */
			exit(0);
		}
		thread_should_exit = false;
		daemon_task = task_spawn_cmd("i2c_wd",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 1024,
					 i2c_wd_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);

		/* Setting the I2C WD:						*/

		/* rate limit i2c_wd status updates to 10Hz */
		orb_set_interval(_hdim010_i2c_wd_sub, 100);


		/* Setting the I2C WD HW					*/
		stm32_configgpio(GPIO_GPIO_DIR);
		stm32_gpiowrite(GPIO_GPIO_DIR, true);
		stm32_configgpio(GPIO_GPIO1_OUTPUT);
		stm32_gpiowrite(GPIO_GPIO1_OUTPUT, true);
		usleep(RESET_TIME_US);														/*  wait for the line set	*/

		warnx("I2C WD is up and running!\n");
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("Application started\n");
		} else {
			warnx("Application is not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int i2c_wd_thread_main(int argc, char *argv[]) {

	warnx("[i2c_wd] starting\n");

	thread_running = true;

	uint32_t hdim010_i2c_reset_counter = 0;

	_hdim010_i2c_wd_sub = orb_subscribe(ORB_ID(hdim010_i2c_wd));

	while (!thread_should_exit) {

		bool hdim010_i2c_wd_updated;
		orb_check(_hdim010_i2c_wd_sub, &hdim010_i2c_wd_updated);

		bool enable_i2c_reset = false;

		if (hdim010_i2c_wd_updated) {
			struct hdim010_i2c_wd_s	hdim010_i2c_wd_report;

			orb_copy(ORB_ID(hdim010_i2c_wd), _hdim010_i2c_wd_sub, &hdim010_i2c_wd_report);

			if (hdim010_i2c_wd_report.hdim010_i2c_error){
				hdim010_i2c_reset_counter++;
				if (hdim010_i2c_reset_counter < I2C_MAX_RESET_COUNTER) {
					warnx("Performing I2C reset due to the HDIM010 error ");
					enable_i2c_reset = true;
				}
			}
			else{
				hdim010_i2c_reset_counter = 0;
			}
		}

		if (enable_i2c_reset) {
#if 0
				/*Reset the external I2C line*/
				stm32_gpiowrite(GPIO_GPIO1_OUTPUT, false);					/* Pull line down to switch off */
				usleep(RESET_TIME_US);										/* wait 200 ms for the reset	*/
				stm32_gpiowrite(GPIO_GPIO1_OUTPUT, true);					/* Pull line up to switch on 	*/
#endif
				////////////////// new code (compliant with the Xbee reset)
				int	fd = open(PX4FMU_DEVICE_PATH, 0);
				if (fd < 0) {
					printf("GPIO: open fail\n");
				}
				// set GPIO2 to outputs and reset the external I2C line
				ioctl(fd, GPIO_SET_OUTPUT, GPIO_EXT_2);
				ioctl(fd, GPIO_CLEAR, GPIO_EXT_2);
				usleep(RESET_TIME_US);
				ioctl(fd, GPIO_SET_INPUT, GPIO_EXT_2);
				if(close(fd)!=0)
					printf("Warning: Closing of GPIO file descriptor failed.");
				//////////////////
			}

		usleep(I2C_WD_INTERVAL_US);											/* WD interval 					*/
	}

	warnx("[i2c_wd] exiting.\n");

	thread_running = false;

	return 0;
}
