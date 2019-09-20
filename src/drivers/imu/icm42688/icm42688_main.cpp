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

#include <px4_getopt.h>

#include "icm42688.h"

enum ICM42688_BUS {
	ICM42688_BUS_SPI_INTERNAL
};

/**
 * Local functions in support of the shell command.
 */
namespace icm42688
{

/*
 * list of supported bus configurations
 */
struct icm42688_bus_option {
	enum ICM42688_BUS busid;
	const char *devpath;
	ICM42688_constructor interface_constructor;
	uint8_t busnum;
	uint32_t device;
	ICM42688 *dev;
} bus_options[] = {
#if defined(PX4_SPIDEV_ICM_42688) && defined(PX4_SPI_BUS_SENSORS2)
	{ ICM42688_BUS_SPI_INTERNAL, "/dev/icm42688_spi_int", &icm42688_spi_interface, PX4_SPI_BUS_SENSORS2, PX4_SPIDEV_ICM_42688, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

/**
 * Start the driver.
 */
bool
start_bus(struct icm42688_bus_option &bus)
{
	if (bus.dev != nullptr) {
		PX4_ERR("bus option already started");
		exit(1);
	}

	IICM42688 *interface = bus.interface_constructor(bus.busnum, bus.device);

	if (interface->init() != OK) {
		delete interface;
		PX4_WARN("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new ICM42688(interface, bus.devpath);

	if (bus.dev == nullptr) {
		return false;
	}

	if (OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = nullptr;
		return false;
	}

	return true;
}

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
void
start(enum ICM42688_BUS busid, Rotation rotation)
{
	uint8_t i;
	bool started = false;

	for (i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i]);
	}

	if (!started) {
		PX4_WARN("bus option number is %d", i);
		PX4_ERR("driver start failed");
		exit(1);
	}

	// one or more drivers started OK
	exit(0);
}

} // namespace


extern "C" int icm42688_main(int argc, char *argv[])
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
			//icm42688::usage();
			exit(0);
		}
	}

	if (myoptind >= argc) {
		//icm42688::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		icm42688::start(ICM42688_BUS_SPI_INTERNAL, rotation);
	}

	if (!strcmp(verb, "stop")) {
		//icm42688::stop();
	}

	if (!strcmp(verb, "info")) {
		//icm42688::info();
	}

	//icm42688::usage();
	return -1;
}
