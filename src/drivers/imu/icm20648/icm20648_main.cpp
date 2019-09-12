/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "icm20648.h"

/**
 * Local functions in support of the shell command.
 */
namespace icm20648
{

ICM20648 *_dev;

bool start(enum Rotation rotation)
{

	if (_dev != nullptr) {
		// If already started, the still command succeeded.
		PX4_INFO("already started");
	}

	// Create the driver.
#if defined(PX4_SPI_BUS_SENSORS1)
	_dev = new ICM20648(PX4_SPI_BUS_SENSORS1, PX4_SPIDEV_ICM_20648, rotation);
#else
	PX4_ERR("External SPI not available");
#endif

	if (_dev != nullptr) {
		if (_dev->init() == OK) {
			return PX4_OK;
		}

		delete _dev;
		_dev = nullptr;
	}

	PX4_ERR("driver start failed");

	return PX4_ERROR;
}

void stop(void)
{
	if (_dev) {
		delete _dev;
		_dev = nullptr;
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void info(void)
{
	if (_dev) {
		_dev->print_info();
	}

	exit(0);
}

void usage(void)
{
	PX4_INFO("missing command: try 'start', 'info', 'stop'");
	PX4_INFO("options:");
	PX4_INFO("    -R rotation");
}

} // namespace

extern "C" int icm20648_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	enum Rotation rotation = ROTATION_NONE;

	/* start options */
	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;

		default:
			icm20648::usage();
			exit(0);
		}
	}

	if (myoptind >= argc) {
		icm20648::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		return icm20648::start(rotation);
	}

	if (!strcmp(verb, "stop")) {
		icm20648::stop();
	}

	if (!strcmp(verb, "info")) {
		icm20648::info();
	}

	icm20648::usage();
	return -1;
}
