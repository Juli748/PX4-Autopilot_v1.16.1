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

#ifndef SENSOR_GROUND_HEIGHTS_HPP
#define SENSOR_GROUND_HEIGHTS_HPP

#include <math.h>

#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/distance_sensor.h>

class MavlinkStreamSensorGroundHeights : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamSensorGroundHeights(mavlink); }

	static constexpr const char *get_name_static() { return "SENSOR_GROUND_HEIGHTS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _distance_sensor_subs.advertised_count() > 0 ?
		       MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamSensorGroundHeights(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	static constexpr hrt_abstime kDistanceTimeout = 500_ms;

	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor};

	distance_sensor_s _last_radar{};
	distance_sensor_s _last_lidar{};

	bool send() override
	{
		for (int i = 0; i < _distance_sensor_subs.size(); i++) {
			distance_sensor_s dist_sensor{};

			if (_distance_sensor_subs[i].update(&dist_sensor)) {
				if (dist_sensor.orientation != distance_sensor_s::ROTATION_DOWNWARD_FACING) {
					continue;
				}

				switch (dist_sensor.type) {
				case distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR:
					_last_radar = dist_sensor;
					break;

				case distance_sensor_s::MAV_DISTANCE_SENSOR_LASER:
					_last_lidar = dist_sensor;
					break;

				default:
					break;
				}
			}
		}

		const hrt_abstime now = hrt_absolute_time();
		const bool radar_fresh = (_last_radar.timestamp != 0) && ((now - _last_radar.timestamp) <= kDistanceTimeout);
		const bool lidar_fresh = (_last_lidar.timestamp != 0) && ((now - _last_lidar.timestamp) <= kDistanceTimeout);

		if (!radar_fresh && !lidar_fresh) {
			return false;
		}

		mavlink_debug_float_array_t msg{};
		msg.time_usec = now;
		msg.array_id = 0;
		memcpy(msg.name, "GNDHEIGHTS", 10);
		msg.data[0] = radar_fresh ? _last_radar.current_distance : NAN;
		msg.data[1] = lidar_fresh ? _last_lidar.current_distance : NAN;
		msg.data[2] = NAN;

		for (size_t i = 3; i < sizeof(msg.data) / sizeof(msg.data[0]); i++) {
			msg.data[i] = NAN;
		}

		mavlink_msg_debug_float_array_send_struct(_mavlink->get_channel(), &msg);

		return true;
	}
};

#endif // SENSOR_GROUND_HEIGHTS_HPP
