/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "AS5600.hpp"

#include <inttypes.h>
#include <mathlib/mathlib.h>

AS5600::AS5600(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
	set_device_type(DRV_SENS_DEVTYPE_AS5600);
}

AS5600::~AS5600()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int AS5600::init()
{
	const int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	ScheduleOnInterval(SAMPLE_INTERVAL_US, SAMPLE_INTERVAL_US);
	return PX4_OK;
}

int AS5600::probe()
{
	uint16_t raw_angle = 0;
	return read_angle(raw_angle);
}

int AS5600::read_angle(uint16_t &raw_angle)
{
	uint8_t reg = REG_RAW_ANGLE_H;
	uint8_t buffer[2] {};

	const int ret = transfer(&reg, 1, buffer, sizeof(buffer));

	if (ret != PX4_OK) {
		return ret;
	}

	raw_angle = (static_cast<uint16_t>(buffer[0]) << 8) | buffer[1];
	raw_angle &= 0x0FFF;
	return PX4_OK;
}

void AS5600::RunImpl()
{
	perf_begin(_sample_perf);
	static constexpr float TWO_PI_F = 6.28318530717958647692f;

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	uint16_t raw_angle = 0;
	const int ret = read_angle(raw_angle);

	if (ret == PX4_OK) {
		const float angle_rad = (static_cast<float>(raw_angle) * TWO_PI_F) / static_cast<float>(RAW_ANGLE_MAX);
		const float angle_deg = math::degrees(angle_rad);

		sensor_aoa_s sensor_aoa{};
		sensor_aoa.timestamp_sample = timestamp_sample;
		sensor_aoa.device_id = get_device_id();
		sensor_aoa.error_count = _error_count;
		sensor_aoa.raw_angle = raw_angle;
		sensor_aoa.angle_rad = angle_rad;
		sensor_aoa.angle_deg = angle_deg;
		sensor_aoa.timestamp = hrt_absolute_time();
		_sensor_aoa_pub.publish(sensor_aoa);

	} else {
		++_error_count;
		perf_count(_comms_errors);
	}

	perf_end(_sample_perf);
}

void AS5600::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("error_count: %" PRIu32, _error_count);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
