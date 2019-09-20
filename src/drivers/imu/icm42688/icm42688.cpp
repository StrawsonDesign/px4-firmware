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

#include "icm42688.h"

ICM42688::ICM42688(IICM42688 *interface, const char *path) :
	CDev(path),
	ScheduledWorkItem(px4::device_bus_to_wq(interface->get_device_id())),
	_interface(interface)
{
}

ICM42688::~ICM42688()
{
	delete _interface;
}

int
ICM42688::init()
{
	int ret = CDev::init();

	if (ret != OK) {
		PX4_ERR("CDev init failed");
		return ret;
	}

	// if (!soft_reset()) {
	// 	PX4_WARN("failed to reset baro during init");
	// 	return -EIO;
	// }

	if (_interface->get_reg(ICM42688_REG_WHO_AM_I) != ICM42688_DEVICE_ID) {
		PX4_WARN("id of your accel is not: 0x%02x", ICM42688_DEVICE_ID);
		return -EIO;

	} else {
		PX4_INFO("detected ICM42688!");
	}

	return OK;
}

/*
 * ScheduledWorkItem override
 */
void
ICM42688::Run()
{
	// TODO
}
