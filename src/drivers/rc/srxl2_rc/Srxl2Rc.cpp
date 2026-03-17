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

#include "Srxl2Rc.hpp"

#include <lib/rc/spektrum_rssi.h>
#include <px4_platform_common/board_common.h>
#include <px4_platform_common/log.h>

#include <inttypes.h>
#include <cmath>
#include <cstring>

using namespace time_literals;

ModuleBase::Descriptor Srxl2Rc::desc{task_spawn, custom_command, print_usage};

Srxl2Rc::Srxl2Rc(const char *device) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device))
{
	if (device) {
		strncpy(_device, device, sizeof(_device) - 1);
		_device[sizeof(_device) - 1] = '\0';
	}

	for (uint8_t i = 0; i < input_rc_s::RC_INPUT_MAX_CHANNELS; ++i) {
		_channel_values[i] = UINT16_MAX;
		_input_rc.values[i] = UINT16_MAX;
	}

	_input_rc.rssi = -1;
	_input_rc.rssi_dbm = NAN;
	_input_rc.link_quality = -1;
	_input_rc.link_snr = -1;
}

Srxl2Rc::~Srxl2Rc()
{
	perf_free(_cycle_interval_perf);
	perf_free(_publish_interval_perf);
}

int Srxl2Rc::task_spawn(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device_name = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return PX4_ERROR;
	}

	if (!device_name) {
		PX4_ERR("valid device required");
		return PX4_ERROR;
	}

	if (board_rc_conflicting(device_name)) {
		PX4_INFO("unable to start, conflict with PX4IO on %s", device_name);
		return PX4_ERROR;
	}

	Srxl2Rc *instance = new Srxl2Rc(device_name);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;
	instance->ScheduleNow();

	return PX4_OK;
}

int Srxl2Rc::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

bool Srxl2Rc::init_uart()
{
	if (_uart != nullptr) {
		return true;
	}

	_uart = new Serial(_device);

	if (_uart == nullptr) {
		PX4_ERR("alloc failed");
		return false;
	}

	if (!_uart->setBaudrate(SRXL2_BAUD_115200) || !_uart->setParity(Parity::None)
	    || !_uart->setStopbits(StopBits::One) || !_uart->setBytesize(ByteSize::EightBits)) {
		PX4_ERR("serial config failed");
		return false;
	}

	if (!_uart->open()) {
		PX4_ERR("open failed on %s", _device);
		return false;
	}

	if (board_rc_swap_rxtx(_device)) {
		_uart->setSwapRxTxMode();
	}

	if (board_rc_singlewire(_device)) {
		_uart->setSingleWireMode();
	}

	srxl2_reset_parser();
	_uart->flush();
	restart_discovery();
	return true;
}

bool Srxl2Rc::configure_baud(uint32_t baudrate)
{
	if (_uart == nullptr || !_uart->isOpen()) {
		return false;
	}

	if (_current_baudrate == baudrate) {
		return true;
	}

	if (!_uart->setBaudrate(baudrate)) {
		PX4_ERR("failed setting baudrate to %" PRIu32, baudrate);
		return false;
	}

	_current_baudrate = baudrate;
	_baud_set_time = hrt_absolute_time();
	srxl2_reset_parser();
	_uart->flush();
	return true;
}

void Srxl2Rc::restart_discovery()
{
	_discovery_mode = true;
	_handshake_seen = false;
	_discovery_start = hrt_absolute_time();
	_last_startup_handshake = 0;
	_baud_set_time = _discovery_start;
	_current_baudrate = SRXL2_BAUD_115200;
	srxl2_reset_parser();

	if (_uart && _uart->isOpen()) {
		_uart->setBaudrate(SRXL2_BAUD_115200);
		_uart->flush();
	}
}

uint32_t Srxl2Rc::device_uid() const
{
	px4_guid_t px4_guid {};
	board_get_px4_guid(px4_guid);
	return static_cast<uint32_t>(px4_guid[0])
	       | (static_cast<uint32_t>(px4_guid[1]) << 8)
	       | (static_cast<uint32_t>(px4_guid[2]) << 16)
	       | (static_cast<uint32_t>(px4_guid[3]) << 24);
}

void Srxl2Rc::maybe_send_startup_handshake(const hrt_abstime now)
{
	if (!_discovery_mode || current_baud() != SRXL2_BAUD_115200) {
		return;
	}

	if (now - _discovery_start > 200_ms) {
		return;
	}

	if (_last_startup_handshake != 0 && now - _last_startup_handshake < 50_ms) {
		return;
	}

	const size_t length = srxl2_build_handshake_packet(_tx_buf, sizeof(_tx_buf),
			      SRXL2_DEVICE_ID_FC_DEFAULT, SRXL2_DEVICE_ID_NONE,
			      10, 1, 0, device_uid());

	if (length > 0) {
		_uart->write(_tx_buf, length);
		_last_startup_handshake = now;
	}
}

void Srxl2Rc::process_control_packet(const srxl2_packet_t &packet, const hrt_abstime now)
{
	_total_frame_count++;
	_input_rc.timestamp_last_signal = now;
	_last_rc_update = now;
	_input_rc.rc_lost_frame_count = packet.frame_loss_count;
	_input_rc.rc_total_frame_count = _total_frame_count;
	_input_rc.rc_failsafe = packet.is_failsafe;
	_input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_SRXL2;

	if (packet.rssi > 0) {
		_input_rc.rssi = packet.rssi;
		_input_rc.rssi_dbm = NAN;

	} else if (packet.rssi < 0) {
		_input_rc.rssi = spek_dbm_to_percent(packet.rssi);
		_input_rc.rssi_dbm = packet.rssi;

	} else {
		_input_rc.rssi = 0;
		_input_rc.rssi_dbm = NAN;
	}

	if (packet.channel_mask == 0) {
		_input_rc.rc_lost = true;
		_input_rc.channel_count = 0;
		return;
	}

	if (packet.is_failsafe) {
		for (uint8_t i = 0; i < input_rc_s::RC_INPUT_MAX_CHANNELS; ++i) {
			_channel_valid[i] = false;
			_channel_values[i] = UINT16_MAX;
		}
	}

	uint8_t data_index = 0;

	for (uint8_t channel = 0; channel < 32 && data_index < packet.num_channels; ++channel) {
		if ((packet.channel_mask & (1u << channel)) == 0) {
			continue;
		}

		if (channel < input_rc_s::RC_INPUT_MAX_CHANNELS) {
			_channel_values[channel] = srxl2_raw_to_pwm(packet.channel_data[data_index]);
			_channel_valid[channel] = true;
		}

		data_index++;
	}

	uint8_t highest_valid_channel = 0;

	for (uint8_t channel = 0; channel < input_rc_s::RC_INPUT_MAX_CHANNELS; ++channel) {
		_input_rc.values[channel] = _channel_valid[channel] ? _channel_values[channel] : UINT16_MAX;

		if (_channel_valid[channel]) {
			highest_valid_channel = channel + 1;
		}
	}

	_input_rc.channel_count = highest_valid_channel;
	_input_rc.rc_lost = false;
}

void Srxl2Rc::process_packet(const srxl2_packet_t &packet, const hrt_abstime now)
{
	_last_packet_seen = now;

	switch (packet.packet_type) {
	case SRXL2_PACKET_TYPE_HANDSHAKE:
		_handshake_seen = true;

		if (packet.dest_id == SRXL2_DEVICE_ID_FC_DEFAULT) {
			const size_t length = srxl2_build_handshake_packet(_tx_buf, sizeof(_tx_buf),
				      SRXL2_DEVICE_ID_FC_DEFAULT, packet.src_id,
				      10, 1, 0, device_uid());

			if (length > 0) {
				_uart->write(_tx_buf, length);
			}
		}

		if (packet.dest_id == SRXL2_DEVICE_ID_BROADCAST) {
			configure_baud(srxl2_baud_from_selector(packet.baud_support));
			_discovery_mode = false;
		}

		break;

	case SRXL2_PACKET_TYPE_CONTROL_DATA:
		if (packet.control_command == SRXL2_CONTROL_CMD_CHANNEL_DATA
		    || packet.control_command == SRXL2_CONTROL_CMD_FAILSAFE_CHANNEL_DATA) {
			process_control_packet(packet, now);
			_discovery_mode = false;
		}

		break;

	default:
		break;
	}
}

void Srxl2Rc::publish_rc(const hrt_abstime now)
{
	if (_last_publish != 0 && now - _last_publish < 4_ms) {
		return;
	}

	if (_last_rc_update != 0 && now - _last_rc_update > 500_ms) {
		_input_rc.rc_lost = true;
		_input_rc.rc_failsafe = true;
	}

	_input_rc.link_quality = -1;
	_input_rc.link_snr = -1;
	_input_rc.rc_ppm_frame_length = 0;
	_input_rc.timestamp = now;
	_input_rc_pub.publish(_input_rc);
	perf_count(_publish_interval_perf);
	_last_publish = now;
}

void Srxl2Rc::Run()
{
	if (should_exit()) {
		ScheduleClear();

		if (_uart) {
			_uart->close();
			delete _uart;
			_uart = nullptr;
		}

		exit_and_cleanup(desc);
		return;
	}

	if (!init_uart()) {
		ScheduleDelayed(100_ms);
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	perf_count_interval(_cycle_interval_perf, now);

	maybe_send_startup_handshake(now);

	const int new_bytes = _uart->readAtLeast(&_rcs_buf[0], sizeof(_rcs_buf), 1, 2);

	if (new_bytes > 0) {
		_bytes_rx += new_bytes;

		for (int i = 0; i < new_bytes; ++i) {
			srxl2_packet_t packet {};

			if (srxl2_parse_byte(_rcs_buf[i], &packet)) {
				process_packet(packet, now);
			}
		}
	}

	if (_discovery_mode && now - _baud_set_time > 100_ms) {
		const uint32_t next_baud = (current_baud() == SRXL2_BAUD_115200) ? SRXL2_BAUD_400000 : SRXL2_BAUD_115200;
		configure_baud(next_baud);
	}

	if (!_discovery_mode && _last_packet_seen != 0 && now - _last_packet_seen > 50_ms) {
		restart_discovery();
	}

	publish_rc(now);
	ScheduleDelayed(4_ms);
}

int Srxl2Rc::print_status()
{
	PX4_INFO("device: %s", _device);
	PX4_INFO("baudrate: %" PRIu32, _current_baudrate);
	PX4_INFO("bytes rx: %" PRIu32, _bytes_rx);
	PX4_INFO("discovery mode: %s", _discovery_mode ? "yes" : "no");
	return 0;
}

int Srxl2Rc::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Spektrum SRXL2 RC input driver.

This module implements the SRXL2 slave-side handshake and decodes Control Data
packets from an SRXL2 receiver to publish RC input.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("srxl2_rc", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "serial device", false);
	return 0;
}

extern "C" __EXPORT int srxl2_rc_main(int argc, char *argv[])
{
	return ModuleBase::main(Srxl2Rc::desc, argc, argv);
}
