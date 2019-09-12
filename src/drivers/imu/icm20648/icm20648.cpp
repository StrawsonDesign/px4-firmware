/***************************************************************************//**
 * @file ICM20648.cpp
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2017 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#include "ICM20648.h"



#define ICM20648_SPI_BUS_SPEED	7*1000*1000


ICM20648::ICM20648(int bus, uint32_t device, enum Rotation rotation) :
	SPI("ICM20648", nullptr, bus, device, SPIDEV_MODE3, ICM20648_SPI_BUS_SPEED),
	ScheduledWorkItem(px4::device_bus_to_wq(SPI::get_device_id())),
	_px4_accel(SPI::get_device_id(), ORB_PRIO_HIGH, rotation),
	_px4_gyro(SPI::get_device_id(), ORB_PRIO_HIGH, rotation),
	_last_accel_data{},
	_got_duplicate(false),
	_sample_rate(1000),
	_sample_perf(perf_alloc(PC_ELAPSED, "icm20648_read")),
	_duplicates(perf_alloc(PC_COUNT, "icm20648_dupe"))
{
	_device_id.devid_s.devtype = DRV_DEVTYPE_ICM20648;
	_px4_accel.set_device_type(DRV_DEVTYPE_ICM20648);
	_px4_gyro.set_device_type(DRV_DEVTYPE_ICM20648);
}

ICM20648::~ICM20648(void)
{
	/* make sure we are truly inactive */
	stop();
}

int
ICM20648::init()
{
	float accel_res;
	float gyro_res;

	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI setup failed %d", ret);
		return ret;
	}

	if (!open()) {
		PX4_ERR("error opening interface");
		return PX4_ERROR;
	}

	set_sample_rate(_sample_rate);

	// 16g / 15 bits
	set_accel_fullscale(ICM20648_ACCEL_FULLSCALE_16G);

	// correct gyro scale factors, scale to rad/s in SI units
	// 2000 deg/s = (2000/180)*PI = 34.906585 rad/s
	// Scaling factor:
	//  Full Scale = 15 bits / 34.906585 rad/s
	//                1 bit = 0.00106526 rad/s
	set_gyro_fullscale(ICM20648_GYRO_FULLSCALE_2000DPS);

	enable_cyclemode(false);

	if (get_accel_resolution(&accel_res)) {
		PX4_ERR("error getting accel resolution");
		return PX4_ERROR;
	}

	if (get_gyro_resolution(&gyro_res)) {
		PX4_ERR("error getting gyro resolution");
		return PX4_ERROR;
	}

	_px4_accel.set_scale(CONSTANTS_ONE_G * accel_res);
	_px4_gyro.set_scale(M_PI_F / (180.0f / gyro_res));

	//
	// ICM20948 driver uses 92Hz (MPU9250_DEFAULT_ONCHIP_FILTER_FREQ)
	// which gets setup as ICM_BITS_GYRO_DLPF_CFG_119HZ and
	// ICM_BITS_ACCEL_DLPF_CFG_111HZ
	//
	set_gyro_bandwidth(ICM20648_GYRO_BW_120HZ);
	set_accel_bandwidth(ICM20648_ACCEL_BW_111HZ);

	// prime
	measure();

	// start interval
	start();

	return OK;
}

void
ICM20648::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_duplicates);

	_px4_accel.print_status();
	_px4_gyro.print_status();
}

bool ICM20648::open()
{
	uint8_t data;

	reset();

	/* Disable I2C interface, use SPI */
	write_register(ICM20648_REG_USER_CTRL, ICM20648_BIT_I2C_IF_DIS);

	read_register(ICM20648_REG_WHO_AM_I, 1, &data);

	if ((data != ICM20648_DEVICE_ID) && (data != ICM20948_DEVICE_ID)) {
		return false;
	}

	/* Auto selects the best available clock source  PLL if ready, else use the Internal oscillator */
	write_register(ICM20648_REG_PWR_MGMT_1, ICM20648_BIT_CLK_PLL);

	/* PLL startup time - maybe it is too long but better be on the safe side, no spec in the datasheet */

	px4_usleep(30000);

	// TODO: check if required...
	/* INT pin: active low, open drain, IT status read clears. It seems that latched mode does not work, the INT pin cannot be cleared if set */
	write_register(ICM20648_REG_INT_PIN_CFG, ICM20648_BIT_INT_ACTL | ICM20648_BIT_INT_OPEN);

	return true;
}


void
ICM20648::start()
{
	/* make sure we are stopped first */
	uint32_t last_call_interval = _call_interval;
	stop();
	_call_interval = last_call_interval;

	ScheduleOnInterval(_call_interval - ICM20648_TIMER_REDUCTION, 1000);
}

void
ICM20648::stop()
{
	ScheduleClear();

	/* reset internal states */
	memset(_last_accel_data, 0, sizeof(_last_accel_data));
}

void
ICM20648::Run()
{
	/* make another measurement */
	measure();
}

bool
ICM20648::check_duplicate(uint8_t *accel_data)
{
	if (!_got_duplicate && memcmp(accel_data, &_last_accel_data, sizeof(_last_accel_data)) == 0) {
		// it isn't new data - wait for next timer
		perf_end(_sample_perf);
		perf_count(_duplicates);
		_got_duplicate = true;

	} else {
		memcpy(&_last_accel_data, accel_data, sizeof(_last_accel_data));
		_got_duplicate = false;
	}

	return _got_duplicate;
}


void
ICM20648::measure()
{
	float acc_x = 0.0f;
	float acc_y = 0.0f;
	float acc_z = 0.0f;

	float gyr_x = 0.0f;
	float gyr_y = 0.0f;
	float gyr_z = 0.0f;

	float temperature = 0.0f;

	uint8_t raw_data[6] = {0};
	int16_t temp;

	/* start measuring */
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	/* Read the six raw data registers into data array */
	read_register(ICM20648_REG_ACCEL_XOUT_H_SH, 6, &raw_data[0]);

	if (check_duplicate(&raw_data[0])) {
		return;
	}

	/* Convert the MSB and LSB into a signed 16-bit value and multiply by the resolution to get the G value */
	/* Scaling occured during init */
	temp = ((int16_t) raw_data[0] << 8) | raw_data[1];
	acc_x = (float) temp;
	temp = ((int16_t) raw_data[2] << 8) | raw_data[3];
	acc_y = (float) temp;
	temp = ((int16_t) raw_data[4] << 8) | raw_data[5];
	acc_z = (float) temp;

	read_register(ICM20648_REG_GYRO_XOUT_H_SH, 6, &raw_data[0]);

	/* Convert the MSB and LSB into a signed 16-bit value and multiply by the resolution to get the dps value */
	/* Scaling occured during init */
	temp = ((int16_t) raw_data[0] << 8) | raw_data[1];
	gyr_x = (float) temp;
	temp = ((int16_t) raw_data[2] << 8) | raw_data[3];
	gyr_y = (float) temp;
	temp = ((int16_t) raw_data[4] << 8) | raw_data[5];
	gyr_z = (float) temp;

	/* Read temperature registers */
	read_register(ICM20648_REG_TEMPERATURE_H, 2, &raw_data[0]);

	/* Convert to int16 */
	temp = (int16_t)((raw_data[0] << 8) + raw_data[1]);

	/* Calculate the Celsius value from the raw reading */
	temperature = ((float) temp / 333.87f) + 21.0f;

	_px4_accel.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);

	_px4_accel.update(timestamp_sample, acc_x, acc_y, acc_z);
	_px4_gyro.update(timestamp_sample, gyr_x, gyr_y, gyr_z);

	/* stop measuring */
	perf_end(_sample_perf);

	return;
}

/***************************************************************************//**
 * @brief
 *    Reads register from the ICM20648 device
 *
 * @param[in] addr
 *    The register address to read from in the sensor
 *    Bit[8:7] - bank address
 *    Bit[6:0] - register address
 *
 * @param[in] numBytes
 *    The number of bytes to read
 *
 * @param[out] data
 *    The data read from the register
 *
 * @return
 *    None
 ******************************************************************************/
void ICM20648::read_register(uint16_t addr, int len, uint8_t *data)
{
	//uint8_t regAddr;
	uint8_t bank;

	// regAddr = (uint8_t) (addr & 0x7F);
	bank = (uint8_t)(addr >> 7);

	select_bank(bank);

	// TEMP...
	if (len >= 12) {
		return;
	}

	uint8_t cmd[13] = {0};
	cmd[0] = (uint8_t)(addr++ | ICM20648_SPI_DIR_READ);

	int ret = transfer(&cmd[0], &cmd[0], len + 1);

	if (ret == OK) {
		memcpy(data, &cmd[1], len);
	}

	return;
}

/***************************************************************************//**
 * @brief
 *    Writes a register in the ICM20648 device
 *
 * @param[in] addr
 *    The register address to write
 *    Bit[8:7] - bank address
 *    Bit[6:0] - register address
 *
 * @param[in] data
 *    The data to write to the register
 *
 * @return
 *    None
 ******************************************************************************/
void ICM20648::write_register(uint16_t addr, uint8_t data)
{
	uint8_t bank;

	bank = (uint8_t)(addr >> 7);

	select_bank(bank);

	uint8_t cmd[2] = {0};
	cmd[0] = (uint8_t)(addr & ICM20648_SPI_DIR_WRITE);
	cmd[1] = data;

	transfer(&cmd[0], &cmd[0], 2);

	return;
}

/***************************************************************************//**
 * @brief
 *    Select the desired register bank
 *
 * @param[in] bank
 *    The address of the register bank (0..3)
 *
 * @return
 *    None
 ******************************************************************************/
void ICM20648::select_bank(uint8_t bank)
{
	uint8_t cmd[2] = {0};
	cmd[0] = ICM20648_REG_BANK_SEL;
	cmd[1] = (uint8_t)(bank << 4);

	transfer(&cmd[0], &cmd[0], 2);

	return;
}

/***************************************************************************//**
 * @brief
 *    Performs soft reset on the ICM20648 chip
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::reset(void)
{
	/* Set H_RESET bit to initiate soft reset */
	write_register(ICM20648_REG_PWR_MGMT_1, ICM20648_BIT_H_RESET);

	/* Wait 100ms to complete the reset sequence */
	px4_usleep(100000);

	return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Sets the sample rate both of the accelerometer and the gyroscope.
 *
 * @param[in] sampleRate
 *    The desired sample rate in Hz. Since the resolution of the sample rate
 *    divider is different in the accel and gyro stages it is possible that
 *    the two sensor will have different sample rate set.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::set_sample_rate(float sampleRate)
{
	set_gyro_sample_rate(sampleRate);
	set_accel_sample_rate(sampleRate);

	return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Sets the sample rate of the accelerometer
 *
 * @param[in] sampleRate
 *    The desired sample rate in Hz
 *
 * @return
 *    The actual sample rate. May be different from the desired value because
 *    of the finite and discrete number of divider settings
 ******************************************************************************/
float ICM20648::set_gyro_sample_rate(float sampleRate)
{
	uint8_t gyroDiv;
	float gyroSampleRate;

	/* Calculate the sample rate divider */
	gyroSampleRate = (1125.0f / sampleRate) - 1.0f;

	/* Check if it fits in the divider register */
	if (gyroSampleRate > 255.0f) {
		gyroSampleRate = 255.0f;
	}

	if (gyroSampleRate < 0.0f) {
		gyroSampleRate = 0.0f;
	}

	/* Write the value to the register */
	gyroDiv = (uint8_t) gyroSampleRate;
	write_register(ICM20648_REG_GYRO_SMPLRT_DIV, gyroDiv);

	/* Calculate the actual sample rate from the divider value */
	gyroSampleRate = 1125.0f / (gyroDiv + 1);

	return gyroSampleRate;
}

/***************************************************************************//**
 * @brief
 *    Sets the sample rate of the gyroscope
 *
 * @param[in] sampleRate
 *    The desired sample rate in Hz
 *
 * @return
 *    The actual sample rate. May be different from the desired value because
 *    of the finite and discrete number of divider settings
 ******************************************************************************/
float ICM20648::set_accel_sample_rate(float sampleRate)
{
	uint16_t accelDiv;
	float accelSampleRate;

	/* Calculate the sample rate divider */
	accelSampleRate = (1125.0f / sampleRate) - 1.0f;

	/* Check if it fits in the divider registers */
	if (accelSampleRate > 4095.0f) {
		accelSampleRate = 4095.0f;
	}

	if (accelSampleRate < 0.0f) {
		accelSampleRate = 0.0f;
	}

	/* Write the value to the registers */
	accelDiv = (uint16_t) accelSampleRate;
	write_register(ICM20648_REG_ACCEL_SMPLRT_DIV_1, (uint8_t)(accelDiv >> 8));
	write_register(ICM20648_REG_ACCEL_SMPLRT_DIV_2, (uint8_t)(accelDiv & 0xFF));

	/* Calculate the actual sample rate from the divider value */
	accelSampleRate = 1125.0 / (accelDiv + 1);

	return accelSampleRate;
}

/***************************************************************************//**
 * @brief
 *    Sets the bandwidth of the gyroscope
 *
 * @param[in] gyroBw
 *    The desired bandwidth value. Use the ICM20648_GYRO_BW_xHZ macros, which
 *    are defined in the icm20648.h file. The value of x can be
 *    6, 12, 24, 51, 120, 150, 200, 360 or 12100.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::set_gyro_bandwidth(uint8_t gyroBw)
{
	uint8_t reg;

	/* Read the GYRO_CONFIG_1 register */
	read_register(ICM20648_REG_GYRO_CONFIG_1, 1, &reg);
	reg &= ~(ICM20648_MASK_GYRO_BW);

	/* Write the new bandwidth value to the gyro config register */
	reg |= (gyroBw & ICM20648_MASK_GYRO_BW);
	write_register(ICM20648_REG_GYRO_CONFIG_1, reg);

	return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Sets the bandwidth of the accelerometer
 *
 * @param[in] accelBw
 *    The desired bandwidth value. Use the ICM20648_ACCEL_BW_yHZ macros, which
 *    are defined in the icm20648.h file. The value of y can be
 *    6, 12, 24, 50, 111, 246, 470 or 1210.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::set_accel_bandwidth(uint8_t accelBw)
{
	uint8_t reg;

	/* Read the GYRO_CONFIG_1 register */
	read_register(ICM20648_REG_ACCEL_CONFIG, 1, &reg);
	reg &= ~(ICM20648_MASK_ACCEL_BW);

	/* Write the new bandwidth value to the gyro config register */
	reg |= (accelBw & ICM20648_MASK_ACCEL_BW);
	write_register(ICM20648_REG_ACCEL_CONFIG, reg);

	return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Gets the actual resolution of the accelerometer
 *
 * @param[out] accelRes
 *    The resolution in g/bit units
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::get_accel_resolution(float *accelRes)
{
	uint8_t reg = 0;

	/* Read the actual acceleration full scale setting */
	read_register(ICM20648_REG_ACCEL_CONFIG, 1, &reg);
	reg &= ICM20648_MASK_ACCEL_FULLSCALE;

	/* Calculate the resolution */
	switch (reg) {
	case ICM20648_ACCEL_FULLSCALE_2G:
		*accelRes = 2.0 / 32768.0;
		break;

	case ICM20648_ACCEL_FULLSCALE_4G:
		*accelRes = 4.0 / 32768.0;
		break;

	case ICM20648_ACCEL_FULLSCALE_8G:
		*accelRes = 8.0 / 32768.0;
		break;

	case ICM20648_ACCEL_FULLSCALE_16G:
		*accelRes = 16.0 / 32768.0;
		break;
	}

	return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Gets the actual resolution of the gyroscope
 *
 * @param[out] gyroRes
 *    The actual resolution in (deg/sec)/bit units
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::get_gyro_resolution(float *gyroRes)
{
	uint8_t reg = 0;

	/* Read the actual gyroscope full scale setting */
	read_register(ICM20648_REG_GYRO_CONFIG_1, 1, &reg);
	reg &= ICM20648_MASK_GYRO_FULLSCALE;

	/* Calculate the resolution */
	switch (reg) {
	case ICM20648_GYRO_FULLSCALE_250DPS:
		*gyroRes = 250.0 / 32768.0;
		break;

	case ICM20648_GYRO_FULLSCALE_500DPS:
		*gyroRes = 500.0 / 32768.0;
		break;

	case ICM20648_GYRO_FULLSCALE_1000DPS:
		*gyroRes = 1000.0 / 32768.0;
		break;

	case ICM20648_GYRO_FULLSCALE_2000DPS:
		*gyroRes = 2000.0 / 32768.0;
		break;
	}

	return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Sets the full scale value of the accelerometer
 *
 * @param[in] accelFs
 *    The desired full scale value. Use the ICM20648_ACCEL_FULLSCALE_xG
 *    macros, which are defined in the icm20648.h file. The value of x can be
 *    2, 4, 8 or 16.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::set_accel_fullscale(uint8_t accelFs)
{
	uint8_t reg;

	accelFs &= ICM20648_MASK_ACCEL_FULLSCALE;
	read_register(ICM20648_REG_ACCEL_CONFIG, 1, &reg);
	reg &= ~(ICM20648_MASK_ACCEL_FULLSCALE);
	reg |= accelFs;
	write_register(ICM20648_REG_ACCEL_CONFIG, reg);

	return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Sets the full scale value of the gyroscope
 *
 * @param[in] gyroFs
 *    The desired full scale value. Use the ICM20648_GYRO_FULLSCALE_yDPS
 *    macros, which are defined in the icm20648.h file. The value of y can be
 *    250, 500, 1000 or 2000.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::set_gyro_fullscale(uint8_t gyroFs)
{
	uint8_t reg;

	gyroFs &= ICM20648_MASK_GYRO_FULLSCALE;
	read_register(ICM20648_REG_GYRO_CONFIG_1, 1, &reg);
	reg &= ~(ICM20648_MASK_GYRO_FULLSCALE);
	reg |= gyroFs;
	write_register(ICM20648_REG_GYRO_CONFIG_1, reg);

	return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables the sleep mode of the device
 *
 * @param[in] enable
 *    If true, sleep mode is enabled. Set to false to disable sleep mode.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::enable_sleepmode(bool enable)
{
	uint8_t reg;

	read_register(ICM20648_REG_PWR_MGMT_1, 1, &reg);

	if (enable) {
		/* Sleep: set the SLEEP bit */
		reg |= ICM20648_BIT_SLEEP;

	} else {
		/* Wake up: clear the SLEEP bit */
		reg &= ~(ICM20648_BIT_SLEEP);
	}

	write_register(ICM20648_REG_PWR_MGMT_1, reg);

	return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables the cycle mode operation of the accel and gyro
 *
 * @param[in] enable
 *    If true both the accel and gyro sensors will operate in cycle mode. If
 *    false the senors working in continuous mode.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::enable_cyclemode(bool enable)
{
	uint8_t reg;

	reg = 0x00;

	if (enable) {
		reg = ICM20648_BIT_ACCEL_CYCLE | ICM20648_BIT_GYRO_CYCLE;
	}

	write_register(ICM20648_REG_LP_CONFIG, reg);

	return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables the sensors in the ICM20648 chip
 *
 * @param[in] accel
 *    If true enables the acceleration sensor
 *
 * @param[in] gyro
 *    If true enables the gyroscope sensor
 *
 * @param[in] temp
 *    If true enables the temperature sensor
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::enable_sensor(bool accel, bool gyro, bool temp)
{
	uint8_t pwrManagement1;
	uint8_t pwrManagement2;

	read_register(ICM20648_REG_PWR_MGMT_1, 1, &pwrManagement1);
	pwrManagement2 = 0;

	/* To enable the accelerometer clear the DISABLE_ACCEL bits in PWR_MGMT_2 */
	if (accel) {
		pwrManagement2 &= ~(ICM20648_BIT_PWR_ACCEL_STBY);

	} else {
		pwrManagement2 |= ICM20648_BIT_PWR_ACCEL_STBY;
	}

	/* To enable gyro clear the DISABLE_GYRO bits in PWR_MGMT_2 */
	if (gyro) {
		pwrManagement2 &= ~(ICM20648_BIT_PWR_GYRO_STBY);

	} else {
		pwrManagement2 |= ICM20648_BIT_PWR_GYRO_STBY;
	}

	/* To enable the temperature sensor clear the TEMP_DIS bit in PWR_MGMT_1 */
	if (temp) {
		pwrManagement1 &= ~(ICM20648_BIT_TEMP_DIS);

	} else {
		pwrManagement1 |= ICM20648_BIT_TEMP_DIS;
	}

	/* Write back the modified values */
	write_register(ICM20648_REG_PWR_MGMT_1, pwrManagement1);
	write_register(ICM20648_REG_PWR_MGMT_2, pwrManagement2);

	return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables the sensors in low power mode in the ICM20648 chip
 *
 * @param[in] enAccel
 *    If true enables the acceleration sensor in low power mode
 *
 * @param[in] enGyro
 *    If true enables the gyroscope sensor in low power mode
 *
 * @param[in] enTemp
 *    If true enables the temperature sensor in low power mode
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::enter_lowpowermode(bool enAccel, bool enGyro, bool enTemp)
{
	uint8_t data;

	read_register(ICM20648_REG_PWR_MGMT_1, 1, &data);

	if (enAccel || enGyro || enTemp) {
		/* Make sure that the chip is not in sleep */
		enable_sleepmode(false);

		/* And in continuous mode */
		enable_cyclemode(false);

		/* Enable the accelerometer and the gyroscope*/
		enable_sensor(enAccel, enGyro, enTemp);

		// TODO
		px4_usleep(50000);

		/* Enable cycle mode */
		enable_cyclemode(true);

		/* Set the LP_EN bit to enable low power mode */
		data |= ICM20648_BIT_LP_EN;

	} else {
		/* Enable continuous mode */
		enable_cyclemode(false);

		/* Clear the LP_EN bit to disable low power mode */
		data &= ~ICM20648_BIT_LP_EN;
	}

	/* Write the updated value to the PWR_MGNT_1 register */
	write_register(ICM20648_REG_PWR_MGMT_1, data);

	return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Enables or disables the interrupts in the ICM20648 chip
 *
 * @param[in] dataReadyEnable
 *    If true enables the Raw Data Ready interrupt, otherwise disables.
 *
 * @param[in] womEnable
 *    If true enables the Wake-up On Motion interrupt, otherwise disables.
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::enable_irq(bool dataReadyEnable, bool womEnable)
{
	uint8_t intEnable;

	/* All interrupts disabled by default */
	intEnable = 0;

	/* Enable one or both of the interrupt sources if required */
	if (womEnable) {
		intEnable = ICM20648_BIT_WOM_INT_EN;
	}

	/* Write value to register */
	write_register(ICM20648_REG_INT_ENABLE, intEnable);

	/* All interrupts disabled by default */
	intEnable = 0;

	if (dataReadyEnable) {
		intEnable = ICM20648_BIT_RAW_DATA_0_RDY_EN;
	}

	/* Write value to register */
	write_register(ICM20648_REG_INT_ENABLE_1, intEnable);

	return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Reads the interrupt status registers of the ICM20648 chip
 *
 * @param[out] intStatus
 *    The content the four interrupt registers. LSByte is INT_STATUS, MSByte is
 *    INT_STATUS_3
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::read_irqstatus(uint32_t *int_status)
{
	uint8_t reg[4] = {0};

	read_register(ICM20648_REG_INT_STATUS, 4, reg);
	*int_status = (uint32_t) reg[0];
	*int_status |= (((uint32_t) reg[1]) << 8);
	*int_status |= (((uint32_t) reg[2]) << 16);
	*int_status |= (((uint32_t) reg[3]) << 24);

	return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Checks if new data is available for read
 *
 * @return
 *    Returns true if the Raw Data Ready interrupt bit set, false otherwise
 ******************************************************************************/
bool ICM20648::is_data_ready(void)
{
	uint8_t status = 0;
	bool ret;

	ret = false;
	read_register(ICM20648_REG_INT_STATUS_1, 1, &status);

	if (status & ICM20648_BIT_RAW_DATA_0_RDY_INT) {
		ret = true;
	}

	return ret;
}

/***************************************************************************//**
 * @brief
 *    Sets up and enables the Wake-up On Motion feature
 *
 * @param[in] enable
 *    If true enables the WOM feature, disables otherwise
 *
 * @param[in] womThreshold
 *    Threshold value for the Wake on Motion Interrupt for ACCEL x/y/z axes.
 *    LSB = 4mg. Range is 0mg to 1020mg
 *
 * @param[in] sampleRate
 *    The desired sample rate of the accel sensor in Hz
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::enable_wake_on_motion(bool enable, uint8_t womThreshold, float sampleRate)
{
	if (enable) {
		/* Make sure that the chip is not in sleep */
		enable_sleepmode(false);

		/* And in continuous mode */
		enable_cyclemode(false);

		/* Enable only the accelerometer */
		enable_sensor(true, false, false);

		/* Set sample rate */
		set_sample_rate(sampleRate);

		/* Set the bandwidth to 1210Hz */
		set_accel_bandwidth(ICM20648_ACCEL_BW_1210HZ);

		/* Accel: 2G full scale */
		set_accel_fullscale(ICM20648_ACCEL_FULLSCALE_2G);

		/* Enable the Wake On Motion interrupt */
		enable_irq(false, true);
		px4_usleep(5000);

		/* Enable Wake On Motion feature */
		write_register(ICM20648_REG_ACCEL_INTEL_CTRL, ICM20648_BIT_ACCEL_INTEL_EN | ICM20648_BIT_ACCEL_INTEL_MODE);

		/* Set the wake on motion threshold value */
		write_register(ICM20648_REG_ACCEL_WOM_THR, womThreshold);

		/* Enable low power mode */
		enter_lowpowermode(true, false, false);

	} else {
		/* Disable Wake On Motion feature */
		write_register(ICM20648_REG_ACCEL_INTEL_CTRL, 0x00);

		/* Disable the Wake On Motion interrupt */
		enable_irq(false, false);

		/* Disable cycle mode */
		enable_cyclemode(false);
	}

	return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Accelerometer and gyroscope calibration function. Reads the gyroscope
 *    and accelerometer values while the device is at rest and in level. The
 *    resulting values are loaded to the accel and gyro bias registers to cancel
 *    the static offset error.
 *
 * @param[out] accelBiasScaled
 *    The mesured acceleration sensor bias in mg
 *
 * @param[out] gyroBiasScaled
 *    The mesured gyro sensor bias in deg/sec
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::calibrate(float *accelBiasScaled, float *gyroBiasScaled)
{
	uint8_t data[12] = {0};
	uint16_t i, packetCount, fifoCount;
	int32_t gyroBias[3] = { 0, 0, 0 };
	int32_t accelBias[3] = { 0, 0, 0 };
	int32_t accelTemp[3] = {0};
	int32_t gyroTemp[3] = {0};
	int32_t accelBiasFactory[3] = {0};
	int32_t gyroBiasStored[3] = {0};
	float gyroRes, accelRes;

	/* Enable the accelerometer and the gyro */
	enable_sensor(true, true, false);

	/* Set 1kHz sample rate */
	set_sample_rate(1100.0);

	/* 246Hz BW for the accelerometer and 200Hz for the gyroscope */
	set_accel_bandwidth(ICM20648_ACCEL_BW_246HZ);
	set_gyro_bandwidth(ICM20648_GYRO_BW_12HZ);

	/* Set the most sensitive range: 2G full scale and 250dps full scale */
	set_accel_fullscale(ICM20648_ACCEL_FULLSCALE_2G);
	set_gyro_fullscale(ICM20648_GYRO_FULLSCALE_250DPS);

	/* Retrieve the resolution per bit */
	get_accel_resolution(&accelRes);
	get_gyro_resolution(&gyroRes);

	/* The accel sensor needs max 30ms, the gyro max 35ms to fully start */
	/* Experiments show that the gyro needs more time to get reliable results */
	px4_usleep(50000);

	/* Disable the FIFO */
	write_register(ICM20648_REG_USER_CTRL, ICM20648_BIT_FIFO_EN);
	write_register(ICM20648_REG_FIFO_MODE, 0x0F);

	/* Enable accelerometer and gyro to store the data in FIFO */
	write_register(ICM20648_REG_FIFO_EN_2, ICM20648_BIT_ACCEL_FIFO_EN | ICM20648_BITS_GYRO_FIFO_EN);

	/* Reset the FIFO */
	write_register(ICM20648_REG_FIFO_RST, 0x0F);
	write_register(ICM20648_REG_FIFO_RST, 0x00);

	/* Enable the FIFO */
	write_register(ICM20648_REG_USER_CTRL, ICM20648_BIT_FIFO_EN);

	/* The max FIFO size is 4096 bytes, one set of measurements takes 12 bytes */
	/* (3 axes, 2 sensors, 2 bytes each value ) 340 samples use 4080 bytes of FIFO */
	/* Loop until at least 4080 samples gathered */
	fifoCount = 0;

	while (fifoCount < 4080) {
		px4_usleep(5000);
		/* Read FIFO sample count */
		read_register(ICM20648_REG_FIFO_COUNT_H, 2, &data[0]);
		/* Convert to a 16 bit value */
		fifoCount = ((uint16_t)(data[0] << 8) | data[1]);
	}

	/* Disable accelerometer and gyro to store the data in FIFO */
	write_register(ICM20648_REG_FIFO_EN_2, 0x00);

	/* Read FIFO sample count */
	read_register(ICM20648_REG_FIFO_COUNT_H, 2, &data[0]);

	/* Convert to a 16 bit value */
	fifoCount = ((uint16_t)(data[0] << 8) | data[1]);

	/* Calculate the number of data sets (3 axis of accel an gyro, two bytes each = 12 bytes) */
	packetCount = fifoCount / 12;

	/* Retrieve the data from the FIFO */
	for (i = 0; i < packetCount; i++) {
		read_register(ICM20648_REG_FIFO_R_W, 12, &data[0]);
		/* Convert to 16 bit signed accel and gyro x,y and z values */
		accelTemp[0] = ((int16_t)(data[0] << 8) | data[1]);
		accelTemp[1] = ((int16_t)(data[2] << 8) | data[3]);
		accelTemp[2] = ((int16_t)(data[4] << 8) | data[5]);
		gyroTemp[0] = ((int16_t)(data[6] << 8) | data[7]);
		gyroTemp[1] = ((int16_t)(data[8] << 8) | data[9]);
		gyroTemp[2] = ((int16_t)(data[10] << 8) | data[11]);

		/* Sum the values */
		accelBias[0] += accelTemp[0];
		accelBias[1] += accelTemp[1];
		accelBias[2] += accelTemp[2];
		gyroBias[0] += gyroTemp[0];
		gyroBias[1] += gyroTemp[1];
		gyroBias[2] += gyroTemp[2];
	}

	/* Divide by packet count to get the average */
	accelBias[0] /= packetCount;
	accelBias[1] /= packetCount;
	accelBias[2] /= packetCount;
	gyroBias[0] /= packetCount;
	gyroBias[1] /= packetCount;
	gyroBias[2] /= packetCount;

	/* Acceleormeter: add or remove (depending on the orientation of the chip) 1G (gravity) from the Z axis value */
	if (accelBias[2] > 0L) {
		accelBias[2] -= (int32_t)(1.0f / accelRes);

	} else {
		accelBias[2] += (int32_t)(1.0f / accelRes);
	}

	/* Convert the values to degrees per sec for displaying */
	gyroBiasScaled[0] = (float) gyroBias[0] * gyroRes;
	gyroBiasScaled[1] = (float) gyroBias[1] * gyroRes;
	gyroBiasScaled[2] = (float) gyroBias[2] * gyroRes;

	/* Read stored gyro trim values. After reset these values are all 0 */
	read_register(ICM20648_REG_XG_OFFS_USRH, 2, &data[0]);
	gyroBiasStored[0] = ((int16_t)(data[0] << 8) | data[1]);
	read_register(ICM20648_REG_YG_OFFS_USRH, 2, &data[0]);
	gyroBiasStored[1] = ((int16_t)(data[0] << 8) | data[1]);
	read_register(ICM20648_REG_ZG_OFFS_USRH, 2, &data[0]);
	gyroBiasStored[2] = ((int16_t)(data[0] << 8) | data[1]);

	/* The gyro bias should be stored in 1000dps full scaled format. We measured in 250dps to get */
	/* the best sensitivity, so need to divide by 4 */
	/* Substract from the stored calibration value */
	gyroBiasStored[0] -= gyroBias[0] / 4;
	gyroBiasStored[1] -= gyroBias[1] / 4;
	gyroBiasStored[2] -= gyroBias[2] / 4;

	/* Split the values into two bytes */
	data[0] = (gyroBiasStored[0] >> 8) & 0xFF;
	data[1] = (gyroBiasStored[0]) & 0xFF;
	data[2] = (gyroBiasStored[1] >> 8) & 0xFF;
	data[3] = (gyroBiasStored[1]) & 0xFF;
	data[4] = (gyroBiasStored[2] >> 8) & 0xFF;
	data[5] = (gyroBiasStored[2]) & 0xFF;

	/* Write the  gyro bias values to the chip */
	write_register(ICM20648_REG_XG_OFFS_USRH, data[0]);
	write_register(ICM20648_REG_XG_OFFS_USRL, data[1]);
	write_register(ICM20648_REG_YG_OFFS_USRH, data[2]);
	write_register(ICM20648_REG_YG_OFFS_USRL, data[3]);
	write_register(ICM20648_REG_ZG_OFFS_USRH, data[4]);
	write_register(ICM20648_REG_ZG_OFFS_USRL, data[5]);

	/* Calculate the accelerometer bias values to store in the hardware accelerometer bias registers. These registers contain */
	/* factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold */
	/* non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature */
	/* compensation calculations(? the datasheet is not clear). Accelerometer bias registers expect bias input */
	/* as 2048 LSB per g, so that the accelerometer biases calculated above must be divided by 8. */

	/* Read factory accelerometer trim values */
	read_register(ICM20648_REG_XA_OFFSET_H, 2, &data[0]);
	accelBiasFactory[0] = ((int16_t)(data[0] << 8) | data[1]);
	read_register(ICM20648_REG_YA_OFFSET_H, 2, &data[0]);
	accelBiasFactory[1] = ((int16_t)(data[0] << 8) | data[1]);
	read_register(ICM20648_REG_ZA_OFFSET_H, 2, &data[0]);
	accelBiasFactory[2] = ((int16_t)(data[0] << 8) | data[1]);

	/* Construct total accelerometer bias, including calculated average accelerometer bias from above */
	/* Scale the 2g full scale (most sensitive range) results to 16g full scale - divide by 8 */
	/* Clear the last bit (temperature compensation? - the datasheet is not clear) */
	/* Substract from the factory calibration value */

	accelBiasFactory[0] -= ((accelBias[0] / 8) & ~1);
	accelBiasFactory[1] -= ((accelBias[1] / 8) & ~1);
	accelBiasFactory[2] -= ((accelBias[2] / 8) & ~1);

	/* Split the values into two bytes */
	data[0] = (accelBiasFactory[0] >> 8) & 0xFF;
	data[1] = (accelBiasFactory[0]) & 0xFF;
	data[2] = (accelBiasFactory[1] >> 8) & 0xFF;
	data[3] = (accelBiasFactory[1]) & 0xFF;
	data[4] = (accelBiasFactory[2] >> 8) & 0xFF;
	data[5] = (accelBiasFactory[2]) & 0xFF;

	/* Store them in the accelerometer offset registers */
	write_register(ICM20648_REG_XA_OFFSET_H, data[0]);
	write_register(ICM20648_REG_XA_OFFSET_L, data[1]);
	write_register(ICM20648_REG_YA_OFFSET_H, data[2]);
	write_register(ICM20648_REG_YA_OFFSET_L, data[3]);
	write_register(ICM20648_REG_ZA_OFFSET_H, data[4]);
	write_register(ICM20648_REG_ZA_OFFSET_L, data[5]);

	/* Convert the values to G for displaying */
	accelBiasScaled[0] = (float) accelBias[0] * accelRes;
	accelBiasScaled[1] = (float) accelBias[1] * accelRes;
	accelBiasScaled[2] = (float) accelBias[2] * accelRes;

	/* Turn off FIFO */
	write_register(ICM20648_REG_USER_CTRL, 0x00);

	/* Disable all sensors */
	enable_sensor(false, false, false);

	return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Gyroscope calibration function. Reads the gyroscope
 *    values while the device is at rest and in level. The
 *    resulting values are loaded to the gyro bias registers to cancel
 *    the static offset error.
 *
 * @param[out] gyroBiasScaled
 *    The mesured gyro sensor bias in deg/sec
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::calibrate_gyro(float *gyroBiasScaled)
{
	uint8_t data[12] = {0};
	uint16_t i, packetCount, fifoCount;
	int32_t gyroBias[3] = { 0, 0, 0 };
	int32_t gyroTemp[3] = {0};
	int32_t gyroBiasStored[3];
	float gyroRes;

	/* Enable the accelerometer and the gyro */
	enable_sensor(true, true, false);

	/* Set 1kHz sample rate */
	set_sample_rate(1100.0);

	/* Configure bandwidth for gyroscope to 12Hz */
	set_gyro_bandwidth(ICM20648_GYRO_BW_12HZ);

	/* Configure sensitivity to 250dps full scale */
	set_gyro_fullscale(ICM20648_GYRO_FULLSCALE_250DPS);

	/* Retrieve the resolution per bit */
	get_gyro_resolution(&gyroRes);

	/* The accel sensor needs max 30ms, the gyro max 35ms to fully start */
	/* Experiments show that the gyro needs more time to get reliable results */
	px4_usleep(50000);

	/* Disable the FIFO */
	write_register(ICM20648_REG_USER_CTRL, ICM20648_BIT_FIFO_EN);
	write_register(ICM20648_REG_FIFO_MODE, 0x0F);

	/* Enable accelerometer and gyro to store the data in FIFO */
	write_register(ICM20648_REG_FIFO_EN_2, ICM20648_BITS_GYRO_FIFO_EN);

	/* Reset the FIFO */
	write_register(ICM20648_REG_FIFO_RST, 0x0F);
	write_register(ICM20648_REG_FIFO_RST, 0x00);

	/* Enable the FIFO */
	write_register(ICM20648_REG_USER_CTRL, ICM20648_BIT_FIFO_EN);

	/* The max FIFO size is 4096 bytes, one set of measurements takes 12 bytes */
	/* (3 axes, 2 sensors, 2 bytes each value ) 340 samples use 4080 bytes of FIFO */
	/* Loop until at least 4080 samples gathered */
	fifoCount = 0;

	while (fifoCount < 4080) {
		px4_usleep(5000);

		/* Read FIFO sample count */
		read_register(ICM20648_REG_FIFO_COUNT_H, 2, &data[0]);

		/* Convert to a 16 bit value */
		fifoCount = ((uint16_t)(data[0] << 8) | data[1]);
	}

	/* Disable accelerometer and gyro to store the data in FIFO */
	write_register(ICM20648_REG_FIFO_EN_2, 0x00);

	/* Read FIFO sample count */
	read_register(ICM20648_REG_FIFO_COUNT_H, 2, &data[0]);

	/* Convert to a 16 bit value */
	fifoCount = ((uint16_t)(data[0] << 8) | data[1]);

	/* Calculate the number of data sets (3 axis of accel an gyro, two bytes each = 12 bytes) */
	packetCount = fifoCount / 12;

	/* Retrieve the data from the FIFO */
	for (i = 0; i < packetCount; i++) {
		read_register(ICM20648_REG_FIFO_R_W, 12, &data[0]);
		/* Convert to 16 bit signed accel and gyro x,y and z values */
		gyroTemp[0] = ((int16_t)(data[6] << 8) | data[7]);
		gyroTemp[1] = ((int16_t)(data[8] << 8) | data[9]);
		gyroTemp[2] = ((int16_t)(data[10] << 8) | data[11]);

		/* Sum the values */
		gyroBias[0] += gyroTemp[0];
		gyroBias[1] += gyroTemp[1];
		gyroBias[2] += gyroTemp[2];
	}

	/* Divide by packet count to get the average */
	gyroBias[0] /= packetCount;
	gyroBias[1] /= packetCount;
	gyroBias[2] /= packetCount;

	/* Convert the values to degrees per sec for displaying */
	gyroBiasScaled[0] = (float) gyroBias[0] * gyroRes;
	gyroBiasScaled[1] = (float) gyroBias[1] * gyroRes;
	gyroBiasScaled[2] = (float) gyroBias[2] * gyroRes;

	/* Read stored gyro trim values. After reset these values are all 0 */
	read_register(ICM20648_REG_XG_OFFS_USRH, 2, &data[0]);
	gyroBiasStored[0] = ((int16_t)(data[0] << 8) | data[1]);

	read_register(ICM20648_REG_YG_OFFS_USRH, 2, &data[0]);
	gyroBiasStored[1] = ((int16_t)(data[0] << 8) | data[1]);

	read_register(ICM20648_REG_ZG_OFFS_USRH, 2, &data[0]);
	gyroBiasStored[2] = ((int16_t)(data[0] << 8) | data[1]);

	/* The gyro bias should be stored in 1000dps full scaled format. We measured in 250dps to get */
	/* the best sensitivity, so need to divide by 4 */
	/* Substract from the stored calibration value */
	gyroBiasStored[0] -= gyroBias[0] / 4;
	gyroBiasStored[1] -= gyroBias[1] / 4;
	gyroBiasStored[2] -= gyroBias[2] / 4;

	/* Split the values into two bytes */
	data[0] = (gyroBiasStored[0] >> 8) & 0xFF;
	data[1] = (gyroBiasStored[0]) & 0xFF;
	data[2] = (gyroBiasStored[1] >> 8) & 0xFF;
	data[3] = (gyroBiasStored[1]) & 0xFF;
	data[4] = (gyroBiasStored[2] >> 8) & 0xFF;
	data[5] = (gyroBiasStored[2]) & 0xFF;

	/* Write the  gyro bias values to the chip */
	write_register(ICM20648_REG_XG_OFFS_USRH, data[0]);
	write_register(ICM20648_REG_XG_OFFS_USRL, data[1]);
	write_register(ICM20648_REG_YG_OFFS_USRH, data[2]);
	write_register(ICM20648_REG_YG_OFFS_USRL, data[3]);
	write_register(ICM20648_REG_ZG_OFFS_USRH, data[4]);
	write_register(ICM20648_REG_ZG_OFFS_USRL, data[5]);

	/* Turn off FIFO */
	write_register(ICM20648_REG_USER_CTRL, 0x00);

	/* Disable all sensors */
	enable_sensor(false, false, false);

	return ICM20648_OK;
}

/***************************************************************************//**
 * @brief
 *    Reads the device ID of the ICM20648
 *
 * @param[out] devID
 *    The ID of the device read from teh WHO_AM_I register. Expected value? 0xE0
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t ICM20648::get_device_id(uint8_t *device_id)
{
	read_register(ICM20648_REG_WHO_AM_I, 1, device_id);

	return ICM20648_OK;
}
