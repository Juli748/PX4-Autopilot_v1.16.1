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

#include "srxl2.h"

#include <mathlib/mathlib.h>

#include <errno.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

namespace
{

struct srxl2_parser_state_t {
	uint8_t buffer[SRXL2_PACKET_LENGTH_MAX] {};
	uint8_t position{0};
	uint8_t expected_length{0};
};

srxl2_parser_state_t g_parser_state{};
srxl2_diagnostics_t g_diagnostics{};

constexpr uint16_t SRXL2_CENTER = 0x8000;
constexpr int32_t SRXL2_FULL_SCALE_DELTA = 21856;

static uint16_t read_le_u16(const uint8_t *buffer)
{
	return static_cast<uint16_t>(buffer[0]) | (static_cast<uint16_t>(buffer[1]) << 8);
}

static uint32_t read_le_u32(const uint8_t *buffer)
{
	return static_cast<uint32_t>(buffer[0])
	       | (static_cast<uint32_t>(buffer[1]) << 8)
	       | (static_cast<uint32_t>(buffer[2]) << 16)
	       | (static_cast<uint32_t>(buffer[3]) << 24);
}

static uint16_t read_be_u16(const uint8_t *buffer)
{
	return (static_cast<uint16_t>(buffer[0]) << 8) | static_cast<uint16_t>(buffer[1]);
}

static void write_le_u32(uint8_t *buffer, uint32_t value)
{
	buffer[0] = static_cast<uint8_t>(value & 0xFF);
	buffer[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
	buffer[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
	buffer[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

static void parser_resync_from_current_buffer()
{
	for (uint8_t i = 1; i < g_parser_state.position; ++i) {
		if (g_parser_state.buffer[i] == SRXL2_ID) {
			memmove(&g_parser_state.buffer[0], &g_parser_state.buffer[i], g_parser_state.position - i);
			g_parser_state.position -= i;
			g_parser_state.expected_length = (g_parser_state.position >= 3) ? g_parser_state.buffer[2] : 0;
			return;
		}
	}

	g_parser_state.position = 0;
	g_parser_state.expected_length = 0;
}

static bool parse_handshake_packet(const uint8_t *buffer, srxl2_packet_t *packet)
{
	if (buffer[2] != 14) {
		return false;
	}

	packet->src_id = buffer[3];
	packet->dest_id = buffer[4];
	packet->priority = buffer[5];
	packet->baud_support = buffer[6];
	packet->info = buffer[7];
	packet->uid = read_le_u32(&buffer[8]);
	g_diagnostics.last_handshake_src_id = packet->src_id;
	g_diagnostics.last_handshake_dest_id = packet->dest_id;
	g_diagnostics.last_handshake_baud_support = packet->baud_support;
	g_diagnostics.last_handshake_info = packet->info;
	return true;
}

static bool parse_control_packet(const uint8_t *buffer, srxl2_packet_t *packet)
{
	packet->control_command = buffer[3];
	packet->reply_id = buffer[4];

	if (packet->control_command != SRXL2_CONTROL_CMD_CHANNEL_DATA
	    && packet->control_command != SRXL2_CONTROL_CMD_FAILSAFE_CHANNEL_DATA) {
		return true;
	}

	const uint8_t control_payload_length = buffer[2] - 7;

	if (control_payload_length < 7 || ((control_payload_length - 7) % 2) != 0) {
		return false;
	}

	packet->is_failsafe = (packet->control_command == SRXL2_CONTROL_CMD_FAILSAFE_CHANNEL_DATA);
	packet->rssi = static_cast<int8_t>(buffer[5]);
	packet->frame_loss_count = read_le_u16(&buffer[6]);
	packet->channel_mask = read_le_u32(&buffer[8]);
	packet->num_channels = (control_payload_length - 7) / 2;

	uint8_t expected_channel_count = 0;

	for (uint32_t mask = packet->channel_mask; mask != 0; mask >>= 1) {
		expected_channel_count += mask & 1u;
	}

	if (expected_channel_count != packet->num_channels) {
		return false;
	}

	const uint8_t *channel_ptr = &buffer[12];

	for (uint8_t i = 0; i < packet->num_channels; ++i) {
		packet->channel_data[i] = read_le_u16(channel_ptr);
		channel_ptr += 2;
	}

	return true;
}

static bool parse_packet(const uint8_t *buffer, srxl2_packet_t *packet)
{
	memset(packet, 0, sizeof(*packet));
	packet->packet_type = buffer[1];
	packet->length = buffer[2];

	switch (packet->packet_type) {
	case SRXL2_PACKET_TYPE_HANDSHAKE:
		return parse_handshake_packet(buffer, packet);

	case SRXL2_PACKET_TYPE_CONTROL_DATA:
		return parse_control_packet(buffer, packet);

	case SRXL2_PACKET_TYPE_BIND_INFO:
	case SRXL2_PACKET_TYPE_PARAMETER_CONFIG:
	case SRXL2_PACKET_TYPE_SIGNAL_QUALITY:
	case SRXL2_PACKET_TYPE_TELEMETRY_DATA:
		return true;

	default:
		return false;
	}
}

} // namespace

int srxl2_config(int uart_fd, uint32_t baudrate)
{
	struct termios t {};

	if (baudrate != SRXL2_BAUD_115200 && baudrate != SRXL2_BAUD_400000) {
		return -EINVAL;
	}

	if (tcgetattr(uart_fd, &t) != 0) {
		return -errno;
	}

	cfsetspeed(&t, baudrate);
	t.c_cflag &= ~(CSTOPB | PARENB);
	t.c_cflag |= CS8;
	t.c_cflag |= CLOCAL | CREAD;
	t.c_iflag = 0;
	t.c_oflag = 0;
	t.c_lflag = 0;

	if (tcsetattr(uart_fd, TCSANOW, &t) != 0) {
		return -errno;
	}

	return 0;
}

void srxl2_reset_parser(void)
{
	g_parser_state = {};
}

void srxl2_reset_diagnostics(void)
{
	g_diagnostics = {};
}

void srxl2_get_diagnostics(srxl2_diagnostics_t *diagnostics)
{
	if (diagnostics != nullptr) {
		*diagnostics = g_diagnostics;
	}
}

bool srxl2_parse_byte(uint8_t byte, srxl2_packet_t *packet)
{
	if (g_parser_state.position == 0) {
		if (byte != SRXL2_ID) {
			return false;
		}

		g_diagnostics.sync_bytes++;
		g_parser_state.buffer[g_parser_state.position++] = byte;
		return false;
	}

	g_parser_state.buffer[g_parser_state.position++] = byte;

	if (g_parser_state.position == 3) {
		g_parser_state.expected_length = g_parser_state.buffer[2];

		if (g_parser_state.expected_length < SRXL2_PACKET_LENGTH_MIN
		    || g_parser_state.expected_length > SRXL2_PACKET_LENGTH_MAX) {
			g_diagnostics.invalid_length++;
			parser_resync_from_current_buffer();
		}

		return false;
	}

	if (g_parser_state.expected_length == 0 || g_parser_state.position < g_parser_state.expected_length) {
		if (g_parser_state.position >= SRXL2_PACKET_LENGTH_MAX) {
			g_diagnostics.oversize_resync++;
			parser_resync_from_current_buffer();
		}

		return false;
	}

	if (g_parser_state.position > g_parser_state.expected_length) {
		parser_resync_from_current_buffer();
		return false;
	}

	const uint16_t computed_crc = srxl2_crc16(g_parser_state.buffer, g_parser_state.expected_length - 2);
	const uint16_t received_crc = read_be_u16(&g_parser_state.buffer[g_parser_state.expected_length - 2]);
	g_diagnostics.packets_seen++;
	g_diagnostics.last_packet_type = g_parser_state.buffer[1];
	g_diagnostics.last_packet_length = g_parser_state.expected_length;
	g_diagnostics.last_crc_expected = computed_crc;
	g_diagnostics.last_crc_received = received_crc;

	bool success = false;

	if (computed_crc == received_crc) {
		success = parse_packet(g_parser_state.buffer, packet);

		if (success) {
			switch (packet->packet_type) {
			case SRXL2_PACKET_TYPE_HANDSHAKE:
				g_diagnostics.handshake_packets++;
				break;

			case SRXL2_PACKET_TYPE_CONTROL_DATA:
				g_diagnostics.control_packets++;

				if (packet->control_command == SRXL2_CONTROL_CMD_CHANNEL_DATA) {
					g_diagnostics.control_channel_packets++;

				} else if (packet->control_command == SRXL2_CONTROL_CMD_FAILSAFE_CHANNEL_DATA) {
					g_diagnostics.control_failsafe_packets++;
				}

				break;

			default:
				g_diagnostics.other_packets++;
				break;
			}

		} else {
			g_diagnostics.parse_failures++;
		}

	} else {
		g_diagnostics.crc_failures++;
	}

	parser_resync_from_current_buffer();
	return success;
}

uint16_t srxl2_crc16_update(uint16_t crc, uint8_t data)
{
	crc ^= static_cast<uint16_t>(data) << 8;

	for (int i = 0; i < 8; ++i) {
		if (crc & 0x8000) {
			crc = (crc << 1) ^ 0x1021;

		} else {
			crc = crc << 1;
		}
	}

	return crc;
}

uint16_t srxl2_crc16(const uint8_t *buffer, size_t length)
{
	uint16_t crc = 0;

	for (size_t i = 0; i < length; ++i) {
		crc = srxl2_crc16_update(crc, buffer[i]);
	}

	return crc;
}

uint16_t srxl2_raw_to_pwm(uint16_t raw_value)
{
	const int32_t delta = static_cast<int32_t>(raw_value) - SRXL2_CENTER;
	const int32_t scaled = 1500 + ((delta * 500) / SRXL2_FULL_SCALE_DELTA);
	return static_cast<uint16_t>(math::constrain(scaled, int32_t{800}, int32_t{2200}));
}

size_t srxl2_build_handshake_packet(uint8_t *buffer, size_t buffer_size,
				    uint8_t src_id, uint8_t dest_id, uint8_t priority, uint8_t baud_support,
				    uint8_t info, uint32_t uid)
{
	static constexpr size_t packet_size = 14;

	if (buffer == nullptr || buffer_size < packet_size) {
		return 0;
	}

	buffer[0] = SRXL2_ID;
	buffer[1] = SRXL2_PACKET_TYPE_HANDSHAKE;
	buffer[2] = packet_size;
	buffer[3] = src_id;
	buffer[4] = dest_id;
	buffer[5] = priority;
	buffer[6] = baud_support;
	buffer[7] = info;
	write_le_u32(&buffer[8], uid);

	const uint16_t crc = srxl2_crc16(buffer, packet_size - 2);
	buffer[12] = static_cast<uint8_t>(crc >> 8);
	buffer[13] = static_cast<uint8_t>(crc & 0xFF);
	return packet_size;
}

uint32_t srxl2_baud_from_selector(uint8_t baud_selector)
{
	return (baud_selector == 0) ? SRXL2_BAUD_115200 : SRXL2_BAUD_400000;
}
