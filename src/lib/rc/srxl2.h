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

/**
 * @file srxl2.h
 *
 * RC protocol definition for Spektrum SRXL2.
 *
 * The implementation in PX4 covers the FC-side slave behavior needed to
 * receive Control Data packets from an SRXL2 receiver and establish the
 * initial handshake specified by the protocol.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <px4_platform_common/defines.h>

__BEGIN_DECLS

#define SRXL2_ID 0xA6

#define SRXL2_PACKET_LENGTH_MIN 5
#define SRXL2_PACKET_LENGTH_MAX 80

#define SRXL2_BAUD_115200 115200u
#define SRXL2_BAUD_400000 400000u

#define SRXL2_DEVICE_ID_FC_DEFAULT 0x30
#define SRXL2_DEVICE_ID_BROADCAST 0xFF
#define SRXL2_DEVICE_ID_NONE 0x00

#define SRXL2_PACKET_TYPE_HANDSHAKE 0x21
#define SRXL2_PACKET_TYPE_BIND_INFO 0x41
#define SRXL2_PACKET_TYPE_PARAMETER_CONFIG 0x50
#define SRXL2_PACKET_TYPE_SIGNAL_QUALITY 0x55
#define SRXL2_PACKET_TYPE_TELEMETRY_DATA 0x80
#define SRXL2_PACKET_TYPE_CONTROL_DATA 0xCD

#define SRXL2_CONTROL_CMD_CHANNEL_DATA 0x00
#define SRXL2_CONTROL_CMD_FAILSAFE_CHANNEL_DATA 0x01
#define SRXL2_CONTROL_CMD_VTX_DATA 0x02

typedef struct {
	uint8_t packet_type;
	uint8_t length;

	/* Handshake fields */
	uint8_t src_id;
	uint8_t dest_id;
	uint8_t priority;
	uint8_t baud_support;
	uint8_t info;
	uint32_t uid;

	/* Control Data fields */
	uint8_t control_command;
	uint8_t reply_id;
	bool is_failsafe;
	int8_t rssi;
	uint16_t frame_loss_count;
	uint32_t channel_mask;
	uint16_t channel_data[32];
	uint8_t num_channels;
} srxl2_packet_t;

/**
 * Configure a UART port for SRXL2.
 *
 * @param uart_fd UART file descriptor
 * @param baudrate selected baudrate, must be 115200 or 400000
 * @return 0 on success, -errno otherwise
 */
__EXPORT int srxl2_config(int uart_fd, uint32_t baudrate);

/** Reset the internal byte-wise parser state. */
__EXPORT void srxl2_reset_parser(void);

/**
 * Parse a single byte from the SRXL2 stream.
 *
 * @param byte next byte from the UART stream
 * @param packet decoded packet when a full valid packet is received
 * @return true when a complete valid packet has been decoded
 */
__EXPORT bool srxl2_parse_byte(uint8_t byte, srxl2_packet_t *packet);

/**
 * CRC-16/XMODEM update helper used by SRXL2.
 */
__EXPORT uint16_t srxl2_crc16_update(uint16_t crc, uint8_t data);

/**
 * CRC-16/XMODEM helper for a contiguous byte buffer.
 */
__EXPORT uint16_t srxl2_crc16(const uint8_t *buffer, size_t length);

/**
 * Convert a raw SRXL2 16-bit channel value into the standard PX4 RC range.
 *
 * @param raw_value SRXL2 channel value
 * @return PWM-equivalent value in approximately [1000, 2000]
 */
__EXPORT uint16_t srxl2_raw_to_pwm(uint16_t raw_value);

/**
 * Build a Handshake packet.
 *
 * @return total packet size in bytes, or 0 on error
 */
__EXPORT size_t srxl2_build_handshake_packet(uint8_t *buffer, size_t buffer_size,
		uint8_t src_id, uint8_t dest_id, uint8_t priority, uint8_t baud_support,
		uint8_t info, uint32_t uid);

/**
 * Translate the protocol's baud selector into an actual baudrate.
 */
__EXPORT uint32_t srxl2_baud_from_selector(uint8_t baud_selector);

__END_DECLS
