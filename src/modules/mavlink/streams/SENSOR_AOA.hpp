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

#ifndef SENSOR_AOA_HPP
#define SENSOR_AOA_HPP

#include <math.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/sensor_airflow_angles.h>
#include <uORB/topics/sensor_temp.h>

class MavlinkStreamSensorAoa : public ModuleParams, public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamSensorAoa(mavlink); }

	static constexpr const char *get_name_static() { return "SENSOR_AIRFLOW_ANGLES"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_DEBUG_VECT; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return (_sensor_airflow_angles_sub.advertised() || _sensor_temp_subs.advertised_count() > 0) ?
		       MAVLINK_MSG_ID_DEBUG_VECT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamSensorAoa(Mavlink *mavlink) : ModuleParams(nullptr), MavlinkStream(mavlink) {}

	static constexpr hrt_abstime kAirflowTimeout = 500_ms;
	static constexpr hrt_abstime kTemperatureTimeout = 2_s;

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _sensor_airflow_angles_sub{ORB_ID::sensor_airflow_angles};
	uORB::SubscriptionMultiArray<sensor_temp_s> _sensor_temp_subs{ORB_ID::sensor_temp};

	sensor_airflow_angles_s _last_airflow{};
	sensor_temp_s _last_temp{};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MAV_AIRFLOW_MODE>) _param_mav_airflow_mode
	);

	bool send() override
	{
		parameter_update_s parameter_update;

		if (_parameter_update_sub.update(&parameter_update)) {
			updateParams();
		}

		bool updated = false;
		sensor_airflow_angles_s sensor_airflow_angles{};

		if (_sensor_airflow_angles_sub.update(&sensor_airflow_angles)) {
			_last_airflow = sensor_airflow_angles;
			updated = true;
		}

		for (int i = 0; i < _sensor_temp_subs.size(); i++) {
			sensor_temp_s sensor_temp{};

			if (_sensor_temp_subs[i].update(&sensor_temp)) {
				_last_temp = sensor_temp;
				updated = true;
			}
		}

		if (!updated) {
			return false;
		}

		const hrt_abstime now = hrt_absolute_time();
		const bool airflow_fresh = (_last_airflow.timestamp != 0) && ((now - _last_airflow.timestamp) <= kAirflowTimeout);
		const bool temp_fresh = (_last_temp.timestamp != 0) && ((now - _last_temp.timestamp) <= kTemperatureTimeout);
		const bool raw_mode = (_param_mav_airflow_mode.get() == 1);

		mavlink_debug_vect_t msg{};
		msg.time_usec = now;
		memcpy(msg.name, raw_mode ? "AIRFLOWRAW" : "AIRFLOW", raw_mode ? 10 : 8);
		msg.x = (airflow_fresh && _last_airflow.aoa_valid) ?
			(raw_mode ? static_cast<float>(_last_airflow.aoa_raw_angle) : _last_airflow.aoa_angle_deg) : NAN;
		msg.y = (airflow_fresh && _last_airflow.ssa_valid) ?
			(raw_mode ? static_cast<float>(_last_airflow.ssa_raw_angle) : _last_airflow.ssa_angle_deg) : NAN;
		msg.z = temp_fresh ? _last_temp.temperature : NAN;
		mavlink_msg_debug_vect_send_struct(_mavlink->get_channel(), &msg);

		return true;
	}
};

#endif // SENSOR_AOA_HPP
