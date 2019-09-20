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

#pragma once

#include <stdint.h>

#include <perf/perf_counter.h>
#include <systemlib/conversions.h>

#include <drivers/drv_hrt.h>

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/conversion/rotation.h>
#include <systemlib/err.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/uORB.h>


#define ICM42688_BANK_0                  (0 << 7)                    /**< Register bank 0 */
#define ICM42688_REG_WHO_AM_I            (ICM42688_BANK_0 | 0x75)    /**< Device ID register */
#define ICM42688_DEVICE_ID               0x47                        /**< ICM42688 Device ID value  */


class IICM42688
{
public:
	virtual ~IICM42688() = default;

	virtual int init() = 0;

	virtual uint8_t get_reg(uint8_t addr) = 0;

	virtual uint32_t get_device_id() const = 0;
};

class ICM42688 : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	ICM42688(IICM42688 *interface, const char *path);
	virtual ~ICM42688();
	virtual int		init();

private:
	IICM42688		*_interface;
	void 			Run() override;
};


/* interface factories */
extern IICM42688 *icm42688_spi_interface(uint8_t busnum, uint32_t device);
typedef IICM42688 *(*ICM42688_constructor)(uint8_t, uint32_t);
