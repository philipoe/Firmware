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
 * @file px4_hw_wd.c
 * PX4 hardware watchdog driver for PX4 autopilot
 *
 * @author: Amir Melzer <amir.melzer@mavt.ethz.ch>
 *  	    Lorenz Meier <lm@inf.ethz.ch>
 */

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <board_config.h>
#include <uORB/uORB.h>
#include <drivers/drv_gpio.h>


#define PX4_HW_WD_INTERVAL_US 50000

static bool thread_should_exit = false;		/**< daemon exit flag 				*/
static bool thread_running = false;			/**< daemon status flag 			*/
static int daemon_task;						/**< Handle of daemon task / thread */

static unsigned long px4_gpios = GPIO_EXT_1 | GPIO_EXT_2;

/**
 * daemon management function.
 */
__EXPORT int px4_hw_wd_main(int argc, char *argv[]);

/**
 * px4 hardware watchdog GPIO initialization
 */
int px4_hw_wdgpio_init(void);

/**
 * Mainloop of daemon.
 */
int px4_hw_wd_thread_main(int argc, char *argv[]);


/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
		errx(1, "usage: px4_hw_wd {start|stop|status} [-p <additional params>]\n\n");
}

int px4_hw_wd_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("px4_hw_wd is already running\n");
			/* this is not an error */
			exit(0);
		}
		thread_should_exit = false;
		daemon_task = task_spawn_cmd("px4_hw_wd",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 4,
					 1024,
					 px4_hw_wd_thread_main,
					 (argv) ? (char * const *)&argv[2] : (char * const *)NULL);

		warnx("PX4 HW WD is up and running!\n");
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

int px4_hw_wdgpio_init(void){

	int		fd;

	fd = open(PX4FMU_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		warn("GPIO: open fail");
		return fd;
	}

	/* deactivate all outputs */
	if (ioctl(fd, GPIO_SET, px4_gpios)) {
		warn("GPIO: clearing pins fail");
		close(fd);
		return -1;
	}

	/* configure all selected GPIOs as outputs */
	if (ioctl(fd, GPIO_SET_OUTPUT, px4_gpios) != 0) {
		warn("GPIO: output set fail");
		close(fd);
		return -1;
	}

	return fd;
}

int px4_hw_wd_thread_main(int argc, char *argv[]) {

	int		fd;

	warnx("[px4_hw_wd] starting\n");

	thread_running = true;

	uint32_t gpio_vector;

	fd = px4_hw_wdgpio_init();

	if (fd < 0)
		goto fail;

	while (!thread_should_exit) {

		ioctl(fd, GPIO_GET, (uint32_t) &gpio_vector);

		gpio_vector = (gpio_vector >> (GPIO_EXT_2 >> 1)) & 0x1;

		if (gpio_vector)
			ioctl(fd, GPIO_CLEAR, GPIO_EXT_2);
		else
			ioctl(fd, GPIO_SET, GPIO_EXT_2);


		usleep(PX4_HW_WD_INTERVAL_US/2);
	}

fail:
	warnx("[px4_hw_wd] exiting.\n");

	thread_running = false;
	close(fd);

	return 0;
}
