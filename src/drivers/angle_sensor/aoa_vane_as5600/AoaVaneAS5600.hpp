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
#include <parameters/param.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/sensor_aoa.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

class AoaVaneAS5600 : public device::I2C, public I2CSPIDriver<AoaVaneAS5600>
{
public:
	static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x36;
	static constexpr uint8_t I2C_ADDRESS_ALTERNATE = 0x40;
	static constexpr uint32_t I2C_SPEED_DEFAULT = 100000;
	static constexpr int CAL_POINT_COUNT = 6;

	enum class SensorRole : int32_t {
		Unknown = sensor_aoa_s::ROLE_UNKNOWN,
		Aoa = sensor_aoa_s::ROLE_AOA,
		Sideslip = sensor_aoa_s::ROLE_SIDESLIP
	};

	AoaVaneAS5600(const I2CSPIDriverConfig &config, SensorRole role);
	~AoaVaneAS5600() override;

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	int init() override;
	void print_status() override;
	void RunImpl();

private:
	static constexpr uint8_t REG_RAW_ANGLE_H = 0x0C;
	static constexpr uint8_t REG_STATUS = 0x0B;
	static constexpr uint8_t REG_CONF_H = 0x07;
	static constexpr uint8_t REG_AGC = 0x1A;
	static constexpr uint8_t REG_MAGNITUDE_H = 0x1B;

	static constexpr uint16_t RAW_ANGLE_MAX = 4096;
	static constexpr uint32_t SAMPLE_INTERVAL_US = 20000;

	struct CalibrationData {
		bool enabled{false};
		bool valid{false};
		int32_t sign{1};
		int32_t slow_filter{16};
		int32_t fast_filter_threshold{0};
		int32_t raw_points[CAL_POINT_COUNT]{};
	};

	int probe() override;
	int read_angle(uint16_t &raw_angle);
	int read_diagnostics(uint8_t &status, uint8_t &agc, uint16_t &magnitude);
	int update_conf_register();
	void update_params(bool force = false);
	float calibrated_angle_deg(uint16_t raw_angle) const;
	bool build_calibration_points(int32_t (&unwrapped_points)[CAL_POINT_COUNT]) const;
	static int32_t unwrap_raw_count(int32_t raw_count, int32_t reference);
	static float wrap_angle_180(float angle_deg);
	static uint8_t slow_filter_to_conf_bits(int32_t slow_filter);
	static uint8_t fast_filter_threshold_to_conf_bits(int32_t fast_filter_threshold);

	static constexpr int PARAM_HANDLE_COUNT = CAL_POINT_COUNT + 4;

	uORB::PublicationMulti<sensor_aoa_s> _sensor_aoa_pub{ORB_ID(sensor_aoa)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};

	const SensorRole _role{SensorRole::Unknown};
	param_t _param_handles[PARAM_HANDLE_COUNT] {};

	CalibrationData _calibration{};
	uint32_t _error_count{0};

	static const char *role_name(SensorRole role);
	static const char *param_name(SensorRole role, int index);
};
