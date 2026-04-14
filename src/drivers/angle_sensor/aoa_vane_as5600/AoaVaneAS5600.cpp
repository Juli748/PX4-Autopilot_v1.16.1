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

#include "AoaVaneAS5600.hpp"

#include <inttypes.h>
#include <mathlib/mathlib.h>
#include <pthread.h>

namespace
{
static constexpr float CALIBRATION_ANGLES_DEG[AoaVaneAS5600::CAL_POINT_COUNT] {0.f, 5.f, 10.f, 15.f, 20.f, 45.f};
static constexpr hrt_abstime AGGREGATE_SAMPLE_TIMEOUT_US = 500_ms;

struct AggregateRoleSample {
	hrt_abstime timestamp{0};
	hrt_abstime timestamp_sample{0};
	uint32_t device_id{0};
	uint32_t error_count{0};
	uint16_t raw_angle{0};
	float angle_rad{NAN};
	float angle_deg{NAN};
};

struct AggregateCache {
	pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
	AggregateRoleSample aoa{};
	AggregateRoleSample ssa{};
};

AggregateCache g_aggregate_cache{};
}

AoaVaneAS5600::AoaVaneAS5600(const I2CSPIDriverConfig &config, SensorRole role) :
	I2C(config),
	I2CSPIDriver(config),
	_role(role)
{
	set_device_type(DRV_SENS_DEVTYPE_AOA_VANE_AS5600);

	for (int i = 0; i < PARAM_HANDLE_COUNT; ++i) {
		_param_handles[i] = param_find(param_name(_role, i));
	}
}

AoaVaneAS5600::~AoaVaneAS5600()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int AoaVaneAS5600::init()
{
	const int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	update_params(true);
	ScheduleOnInterval(SAMPLE_INTERVAL_US, SAMPLE_INTERVAL_US);
	return PX4_OK;
}

I2CSPIDriverBase *AoaVaneAS5600::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
	SensorRole role = SensorRole::Unknown;

	switch (config.custom1) {
	case static_cast<int32_t>(SensorRole::Aoa):
		role = SensorRole::Aoa;
		break;

	case static_cast<int32_t>(SensorRole::Sideslip):
		role = SensorRole::Sideslip;
		break;

	default:
		PX4_ERR("invalid sensor role");
		return nullptr;
	}

	AoaVaneAS5600 *instance = new AoaVaneAS5600(config, role);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	return instance;
}

int AoaVaneAS5600::probe()
{
	uint16_t raw_angle = 0;
	return read_angle(raw_angle);
}

int AoaVaneAS5600::read_angle(uint16_t &raw_angle)
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

int AoaVaneAS5600::read_diagnostics(uint8_t &status, uint8_t &agc, uint16_t &magnitude)
{
	uint8_t reg = REG_STATUS;
	uint8_t buffer[4] {};

	const int ret = transfer(&reg, 1, buffer, sizeof(buffer));

	if (ret != PX4_OK) {
		return ret;
	}

	status = buffer[0];
	agc = buffer[1];
	magnitude = (static_cast<uint16_t>(buffer[2]) << 8) | buffer[3];
	magnitude &= 0x0FFF;
	return PX4_OK;
}

uint8_t AoaVaneAS5600::slow_filter_to_conf_bits(int32_t slow_filter)
{
	switch (slow_filter) {
	case 2: return 0x3;
	case 4: return 0x2;
	case 8: return 0x1;
	case 16:
	default:
		return 0x0;
	}
}

uint8_t AoaVaneAS5600::fast_filter_threshold_to_conf_bits(int32_t fast_filter_threshold)
{
	switch (fast_filter_threshold) {
	case 6: return 0x1;
	case 7: return 0x2;
	case 9: return 0x3;
	case 18: return 0x4;
	case 21: return 0x5;
	case 24: return 0x6;
	case 10: return 0x7;
	case 0:
	default:
		return 0x0;
	}
}

int AoaVaneAS5600::update_conf_register()
{
	uint8_t reg = REG_CONF_H;
	uint8_t conf[2] {};

	int ret = transfer(&reg, 1, conf, sizeof(conf));

	if (ret != PX4_OK) {
		return ret;
	}

	const uint8_t original_conf_h = conf[0];
	const uint8_t new_conf_h = static_cast<uint8_t>((original_conf_h & 0x80)
				| ((fast_filter_threshold_to_conf_bits(_calibration.fast_filter_threshold) & 0x7) << 2)
				| (slow_filter_to_conf_bits(_calibration.slow_filter) & 0x3));

	if (new_conf_h == original_conf_h) {
		return PX4_OK;
	}

	uint8_t write_buffer[3] {REG_CONF_H, new_conf_h, conf[1]};
	return transfer(write_buffer, sizeof(write_buffer), nullptr, 0);
}

void AoaVaneAS5600::update_params(bool force)
{
	if (!_parameter_update_sub.updated() && !force) {
		return;
	}

	parameter_update_s parameter_update{};
	_parameter_update_sub.copy(&parameter_update);

	int32_t enabled = 0;
	param_get(_param_handles[0], &enabled);
	_calibration.enabled = enabled != 0;
	param_get(_param_handles[1], &_calibration.sign);
	_calibration.sign = (_calibration.sign >= 0) ? 1 : -1;
	param_get(_param_handles[2], &_calibration.slow_filter);
	param_get(_param_handles[3], &_calibration.fast_filter_threshold);

	switch (_calibration.slow_filter) {
	case 2:
	case 4:
	case 8:
	case 16:
		break;

	default:
		_calibration.slow_filter = 16;
		break;
	}

	switch (_calibration.fast_filter_threshold) {
	case 0:
	case 6:
	case 7:
	case 9:
	case 10:
	case 18:
	case 21:
	case 24:
		break;

	default:
		_calibration.fast_filter_threshold = 0;
		break;
	}

	for (int i = 0; i < CAL_POINT_COUNT; i++) {
		param_get(_param_handles[i + 4], &_calibration.raw_points[i]);
	}

	int32_t unwrapped_points[CAL_POINT_COUNT] {};
	_calibration.valid = _calibration.enabled && build_calibration_points(unwrapped_points);

	const int ret = update_conf_register();

	if (ret != PX4_OK) {
		++_error_count;
		perf_count(_comms_errors);
	}
}

int32_t AoaVaneAS5600::unwrap_raw_count(int32_t raw_count, int32_t reference)
{
	int32_t candidate = raw_count;
	const int32_t diff = candidate - reference;

	if (diff > static_cast<int32_t>(RAW_ANGLE_MAX / 2)) {
		candidate -= RAW_ANGLE_MAX;

	} else if (diff < -static_cast<int32_t>(RAW_ANGLE_MAX / 2)) {
		candidate += RAW_ANGLE_MAX;
	}

	return candidate;
}

float AoaVaneAS5600::wrap_angle_180(float angle_deg)
{
	while (angle_deg <= -180.f) {
		angle_deg += 360.f;
	}

	while (angle_deg > 180.f) {
		angle_deg -= 360.f;
	}

	return angle_deg;
}

bool AoaVaneAS5600::build_calibration_points(int32_t (&unwrapped_points)[CAL_POINT_COUNT]) const
{
	unwrapped_points[0] = _calibration.raw_points[0];

	bool strictly_increasing = true;
	bool strictly_decreasing = true;

	for (int i = 1; i < CAL_POINT_COUNT; i++) {
		unwrapped_points[i] = unwrap_raw_count(_calibration.raw_points[i], unwrapped_points[i - 1]);

		if (unwrapped_points[i] <= unwrapped_points[i - 1]) {
			strictly_increasing = false;
		}

		if (unwrapped_points[i] >= unwrapped_points[i - 1]) {
			strictly_decreasing = false;
		}
	}

	return strictly_increasing || strictly_decreasing;
}

float AoaVaneAS5600::calibrated_angle_deg(uint16_t raw_angle) const
{
	if (!_calibration.valid) {
		return wrap_angle_180(_calibration.sign * ((static_cast<float>(raw_angle) * 360.f) / static_cast<float>(RAW_ANGLE_MAX)));
	}

	int32_t calibration_points[CAL_POINT_COUNT] {};
	build_calibration_points(calibration_points);

	const bool increasing = calibration_points[1] > calibration_points[0];
	int32_t raw_unwrapped = unwrap_raw_count(raw_angle, calibration_points[0]);

	if (increasing) {
		for (int i = 0; i < CAL_POINT_COUNT - 1; i++) {
			if (raw_unwrapped >= calibration_points[i] && raw_unwrapped <= calibration_points[i + 1]) {
				const float span = static_cast<float>(calibration_points[i + 1] - calibration_points[i]);
				const float ratio = static_cast<float>(raw_unwrapped - calibration_points[i]) / span;
				const float angle_deg = CALIBRATION_ANGLES_DEG[i] + ratio * (CALIBRATION_ANGLES_DEG[i + 1] - CALIBRATION_ANGLES_DEG[i]);
				return wrap_angle_180(_calibration.sign * angle_deg);
			}
		}

		if (raw_unwrapped < calibration_points[0]) {
			raw_unwrapped += RAW_ANGLE_MAX;
		}

		const int last = CAL_POINT_COUNT - 1;
		const float span = static_cast<float>((calibration_points[0] + RAW_ANGLE_MAX) - calibration_points[last]);
		const float ratio = static_cast<float>(raw_unwrapped - calibration_points[last]) / span;
		const float angle_deg = CALIBRATION_ANGLES_DEG[last] + ratio * ((CALIBRATION_ANGLES_DEG[0] + 360.f) - CALIBRATION_ANGLES_DEG[last]);
		return wrap_angle_180(_calibration.sign * angle_deg);

	} else {
		for (int i = 0; i < CAL_POINT_COUNT - 1; i++) {
			if (raw_unwrapped <= calibration_points[i] && raw_unwrapped >= calibration_points[i + 1]) {
				const float span = static_cast<float>(calibration_points[i + 1] - calibration_points[i]);
				const float ratio = static_cast<float>(raw_unwrapped - calibration_points[i]) / span;
				const float angle_deg = CALIBRATION_ANGLES_DEG[i] + ratio * (CALIBRATION_ANGLES_DEG[i + 1] - CALIBRATION_ANGLES_DEG[i]);
				return wrap_angle_180(_calibration.sign * angle_deg);
			}
		}

		if (raw_unwrapped > calibration_points[0]) {
			raw_unwrapped -= RAW_ANGLE_MAX;
		}

		const int last = CAL_POINT_COUNT - 1;
		const float span = static_cast<float>((calibration_points[0] - RAW_ANGLE_MAX) - calibration_points[last]);
		const float ratio = static_cast<float>(raw_unwrapped - calibration_points[last]) / span;
		const float angle_deg = CALIBRATION_ANGLES_DEG[last] + ratio * ((CALIBRATION_ANGLES_DEG[0] - 360.f) - CALIBRATION_ANGLES_DEG[last]);
		return wrap_angle_180(_calibration.sign * angle_deg);
	}
}

void AoaVaneAS5600::RunImpl()
{
	perf_begin(_sample_perf);
	update_params();

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	uint16_t raw_angle = 0;
	uint8_t status = 0;
	uint8_t agc = 0;
	uint16_t magnitude = 0;
	const int ret = read_angle(raw_angle);

	if (ret == PX4_OK && read_diagnostics(status, agc, magnitude) == PX4_OK) {
		const float angle_deg = calibrated_angle_deg(raw_angle);
		const float angle_rad = math::radians(angle_deg);

		sensor_aoa_s sensor_aoa{};
		sensor_aoa.timestamp_sample = timestamp_sample;
		sensor_aoa.device_id = get_device_id();
		sensor_aoa.error_count = _error_count;
		sensor_aoa.sensor_role = static_cast<uint8_t>(_role);
		sensor_aoa.raw_angle = raw_angle;
		sensor_aoa.angle_rad = angle_rad;
		sensor_aoa.angle_deg = angle_deg;
		sensor_aoa.status = status;
		sensor_aoa.agc = agc;
		sensor_aoa.magnitude = magnitude;
		sensor_aoa.magnet_detected = (status & 0x20) != 0;
		sensor_aoa.magnet_too_weak = (status & 0x10) != 0;
		sensor_aoa.magnet_too_strong = (status & 0x08) != 0;
		sensor_aoa.timestamp = hrt_absolute_time();
		_sensor_aoa_pub.publish(sensor_aoa);
		publish_aggregate_topic(sensor_aoa);

	} else {
		++_error_count;
		perf_count(_comms_errors);
	}

	perf_end(_sample_perf);
}

void AoaVaneAS5600::publish_aggregate_topic(const sensor_aoa_s &sensor_aoa)
{
	AggregateRoleSample current_role_sample{};
	current_role_sample.timestamp = sensor_aoa.timestamp;
	current_role_sample.timestamp_sample = sensor_aoa.timestamp_sample;
	current_role_sample.device_id = sensor_aoa.device_id;
	current_role_sample.error_count = sensor_aoa.error_count;
	current_role_sample.raw_angle = sensor_aoa.raw_angle;
	current_role_sample.angle_rad = sensor_aoa.angle_rad;
	current_role_sample.angle_deg = sensor_aoa.angle_deg;

	AggregateRoleSample aoa_sample{};
	AggregateRoleSample ssa_sample{};

	pthread_mutex_lock(&g_aggregate_cache.lock);

	if (sensor_aoa.sensor_role == sensor_aoa_s::ROLE_SIDESLIP) {
		g_aggregate_cache.ssa = current_role_sample;

	} else {
		g_aggregate_cache.aoa = current_role_sample;
	}

	aoa_sample = g_aggregate_cache.aoa;
	ssa_sample = g_aggregate_cache.ssa;

	pthread_mutex_unlock(&g_aggregate_cache.lock);

	const hrt_abstime now = sensor_aoa.timestamp;
	const bool aoa_valid = (aoa_sample.timestamp != 0) && ((now - aoa_sample.timestamp) <= AGGREGATE_SAMPLE_TIMEOUT_US);
	const bool ssa_valid = (ssa_sample.timestamp != 0) && ((now - ssa_sample.timestamp) <= AGGREGATE_SAMPLE_TIMEOUT_US);

	sensor_airflow_angles_s aggregate{};
	aggregate.timestamp = now;
	aggregate.timestamp_sample = math::max(aoa_sample.timestamp_sample, ssa_sample.timestamp_sample);
	aggregate.aoa_valid = aoa_valid;
	aggregate.ssa_valid = ssa_valid;
	aggregate.aoa_device_id = aoa_valid ? aoa_sample.device_id : 0;
	aggregate.ssa_device_id = ssa_valid ? ssa_sample.device_id : 0;
	aggregate.aoa_error_count = aoa_valid ? aoa_sample.error_count : 0;
	aggregate.ssa_error_count = ssa_valid ? ssa_sample.error_count : 0;
	aggregate.aoa_raw_angle = aoa_valid ? aoa_sample.raw_angle : 0;
	aggregate.ssa_raw_angle = ssa_valid ? ssa_sample.raw_angle : 0;
	aggregate.aoa_angle_rad = aoa_valid ? aoa_sample.angle_rad : NAN;
	aggregate.ssa_angle_rad = ssa_valid ? ssa_sample.angle_rad : NAN;
	aggregate.aoa_angle_deg = aoa_valid ? aoa_sample.angle_deg : NAN;
	aggregate.ssa_angle_deg = ssa_valid ? ssa_sample.angle_deg : NAN;
	_sensor_airflow_angles_pub.publish(aggregate);
}

void AoaVaneAS5600::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("role: %s", role_name(_role));
	PX4_INFO("error_count: %" PRIu32, _error_count);
	PX4_INFO("aoa calibration: %s", _calibration.valid ? "enabled" : (_calibration.enabled ? "invalid" : "disabled"));
	PX4_INFO("aoa sign: %" PRId32, _calibration.sign);
	PX4_INFO("aoa slow filter: %" PRId32 "x", _calibration.slow_filter);
	PX4_INFO("aoa fast threshold: %" PRId32 " LSB", _calibration.fast_filter_threshold);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

const char *AoaVaneAS5600::role_name(SensorRole role)
{
	switch (role) {
	case SensorRole::Aoa:
		return "aoa";

	case SensorRole::Sideslip:
		return "sideslip";

	default:
		return "unknown";
	}
}

const char *AoaVaneAS5600::param_name(SensorRole role, int index)
{
	static constexpr const char *aoa_param_names[PARAM_HANDLE_COUNT] = {
		"SENS_AOA_CAL_EN",
		"SENS_AOA_SIGN",
		"SENS_AOA_SF",
		"SENS_AOA_FTH",
		"SENS_AOA_RAW_0",
		"SENS_AOA_RAW_5",
		"SENS_AOA_RAW_10",
		"SENS_AOA_RAW_15",
		"SENS_AOA_RAW_20",
		"SENS_AOA_RAW_45",
	};

	static constexpr const char *ssa_param_names[PARAM_HANDLE_COUNT] = {
		"SENS_SSA_CAL_EN",
		"SENS_SSA_SIGN",
		"SENS_SSA_SF",
		"SENS_SSA_FTH",
		"SENS_SSA_RAW_0",
		"SENS_SSA_RAW_5",
		"SENS_SSA_RAW_10",
		"SENS_SSA_RAW_15",
		"SENS_SSA_RAW_20",
		"SENS_SSA_RAW_45",
	};

	if (index < 0 || index >= PARAM_HANDLE_COUNT) {
		return "";
	}

	return (role == SensorRole::Sideslip) ? ssa_param_names[index] : aoa_param_names[index];
}
