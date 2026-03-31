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

#include <cstring>

#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/sensor_aoa.h>

class MavlinkStreamSensorAoa : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamSensorAoa(mavlink); }

	static constexpr const char *get_name_static() { return "SENSOR_AOA"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sensor_aoa_subs.advertised_count() * (MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
	explicit MavlinkStreamSensorAoa(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::SubscriptionMultiArray<sensor_aoa_s> _sensor_aoa_subs{ORB_ID::sensor_aoa};

	bool send() override
	{
		bool updated = false;

		for (int i = 0; i < _sensor_aoa_subs.size(); i++) {
			sensor_aoa_s sensor_aoa{};

			if (_sensor_aoa_subs[i].update(&sensor_aoa)) {
				mavlink_debug_float_array_t msg{};
				msg.time_usec = sensor_aoa.timestamp;
				msg.array_id = i;
				memcpy(msg.name, (sensor_aoa.sensor_role == sensor_aoa_s::ROLE_SIDESLIP) ? "SENSOR_SSA" : "SENSOR_AOA", 10);
				msg.data[0] = sensor_aoa.angle_deg;
				msg.data[1] = static_cast<float>(sensor_aoa.raw_angle);
				msg.data[2] = sensor_aoa.angle_rad;
				msg.data[3] = static_cast<float>(sensor_aoa.status);
				msg.data[4] = static_cast<float>(sensor_aoa.agc);
				msg.data[5] = static_cast<float>(sensor_aoa.magnitude);
				msg.data[6] = sensor_aoa.magnet_detected ? 1.f : 0.f;
				msg.data[7] = sensor_aoa.magnet_too_weak ? 1.f : 0.f;
				msg.data[8] = sensor_aoa.magnet_too_strong ? 1.f : 0.f;
				msg.data[9] = static_cast<float>(sensor_aoa.sensor_role);

				mavlink_msg_debug_float_array_send_struct(_mavlink->get_channel(), &msg);
				updated = true;
			}
		}

		return updated;
	}
};

#endif // SENSOR_AOA_HPP
