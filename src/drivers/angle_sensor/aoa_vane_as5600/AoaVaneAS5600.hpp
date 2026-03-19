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

#pragma once

#include <drivers/drv_sensor.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_aoa.h>

class AoaVaneAS5600 : public device::I2C, public I2CSPIDriver<AoaVaneAS5600>
{
public:
	static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x36;
	static constexpr uint32_t I2C_SPEED_DEFAULT = 100000;

	AoaVaneAS5600(const I2CSPIDriverConfig &config);
	~AoaVaneAS5600() override;

	static void print_usage();

	int init() override;
	void print_status() override;
	void RunImpl();

private:
	static constexpr uint8_t REG_RAW_ANGLE_H = 0x0C;

	static constexpr uint16_t RAW_ANGLE_MAX = 4096;
	static constexpr uint32_t SAMPLE_INTERVAL_US = 20000;

	int probe() override;
	int read_angle(uint16_t &raw_angle);

	uORB::Publication<sensor_aoa_s> _sensor_aoa_pub{ORB_ID(sensor_aoa)};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};

	uint32_t _error_count{0};
};
