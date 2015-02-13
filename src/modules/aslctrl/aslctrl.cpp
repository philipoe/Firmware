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
 * @file aslctrl.cpp
 * Flight control application by Autonomous Systems Lab, ETH Zurich (ASL, www.asl.ethz.ch).
 *
 * Authors: Philipp Oettershagen <philipp.oettershagen@mavt.ethz.ch>
 * 		   Konrad Rudin <konrad.rudin@mavt.ethz.ch>
 * 		   Konstantinos Alexis <konstantinos.alexis@mavt.ethz.ch>
 */

//PX4 controlframework includes
#include <stdio.h>
#include <stdlib.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

//Our includes
#include "ASLAutopilot.h"

static bool thread_should_exit = false;     /**< Deamon exit flag */
static bool thread_running = false;     /**< Deamon status flag */
static int deamon_task;             /**< Handle of deamon task / thread */

/**
 * Deamon management function.
 */
extern "C" __EXPORT int aslctrl_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int aslctrl_thread_main(int argc, char *argv[]);

/** Print the correct usage. */
static void usage(const char *reason);
//static void test ();
static void usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: aslctrl {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int aslctrl_main(int argc, char *argv[])
{

	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;

		deamon_task = task_spawn_cmd("aslctrl",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 10,
					 6144,
					 aslctrl_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	/*if (!strcmp(argv[1], "test")) {
		test();
		exit(0);
	}*/

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("is running");

		} else {
			warnx("not started");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int aslctrl_thread_main(int argc, char *argv[])
{
	warnx("starting...");

	ASLAutopilot autopilot;

	//Sleep. Then reload&republish parameters, as necessary for proper logging in sdlog2
	usleep(3000000);
	autopilot.ReloadParameters();

	warnx("ready.");

	thread_running = true;
	while (!thread_should_exit) {
		autopilot.update();
	}

	warnx("exiting.");

	thread_running = false;

	return 0;
}


