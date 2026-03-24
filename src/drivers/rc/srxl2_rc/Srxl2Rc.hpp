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

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/Serial.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/input_rc.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/rc/srxl2.h>

using namespace device;

class Srxl2Rc : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	explicit Srxl2Rc(const char *device);
	~Srxl2Rc() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

private:
	void Run() override;

	bool init_uart();
	bool configure_baud(uint32_t baudrate);
	void restart_discovery();
	void maybe_send_startup_handshake(const hrt_abstime now);
	void maybe_advance_discovery(const hrt_abstime now);
	void process_packet(const srxl2_packet_t &packet, const hrt_abstime now);
	void process_control_packet(const srxl2_packet_t &packet, const hrt_abstime now);
	void publish_rc(const hrt_abstime now);
	uint32_t current_baud() const { return _current_baudrate; }
	uint32_t device_uid() const;

	uORB::PublicationMulti<input_rc_s> _input_rc_pub{ORB_ID(input_rc)};
	input_rc_s _input_rc{};

	Serial *_uart{nullptr};
	char _device[20] {};

	static constexpr size_t RC_MAX_BUFFER_SIZE{SRXL2_PACKET_LENGTH_MAX};
	uint8_t _rcs_buf[RC_MAX_BUFFER_SIZE] {};
	uint8_t _tx_buf[SRXL2_PACKET_LENGTH_MAX] {};

	uint16_t _channel_values[input_rc_s::RC_INPUT_MAX_CHANNELS] {};
	bool _channel_valid[input_rc_s::RC_INPUT_MAX_CHANNELS] {};

	uint32_t _current_baudrate{SRXL2_BAUD_115200};
	hrt_abstime _baud_set_time{0};
	hrt_abstime _discovery_start{0};
	hrt_abstime _last_startup_handshake{0};
	hrt_abstime _last_packet_seen{0};
	hrt_abstime _last_rc_update{0};
	hrt_abstime _last_publish{0};
	bool _discovery_mode{true};
	bool _handshake_seen{false};
	uint32_t _bytes_rx{0};
	uint32_t _total_frame_count{0};
	uint32_t _handshake_unprompted_count{0};
	uint32_t _handshake_targeted_count{0};
	uint32_t _handshake_broadcast_count{0};
	bool _have_valid_channels{false};
	bool _singlewire_enabled{false};
	bool _swap_rxtx_enabled{false};

	perf_counter_t _cycle_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};
	perf_counter_t _publish_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": publish interval")};
};
