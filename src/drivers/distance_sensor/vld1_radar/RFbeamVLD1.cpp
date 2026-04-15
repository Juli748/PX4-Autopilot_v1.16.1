#include "RFbeamVLD1.hpp"

// TODO: figure out why this is not in header file
#include <lib/drivers/device/Device.hpp>

namespace
{
constexpr uint32_t packet_payload_len(const uint8_t *buffer)
{
	return (uint32_t)buffer[4]
	       | ((uint32_t)buffer[5] << 8)
	       | ((uint32_t)buffer[6] << 16)
	       | ((uint32_t)buffer[7] << 24);
}

bool packet_has_header(const uint8_t *buffer, const char *header)
{
	return memcmp(buffer, header, PACKET_HEADER_BYTES) == 0;
}
}

RFbeamVLD1::RFbeamVLD1(const char *port, uint8_t rotation)
	: ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)), _px4_rangefinder(0, rotation)
{
	// Store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// Enforce null terminal
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]);  // assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_RFBEAM);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
}

RFbeamVLD1::~RFbeamVLD1()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int RFbeamVLD1::init()
{
	// Update parameter values (only this one time)
	ModuleParams::updateParams();

	// Open serial port before writing to it
	if (openSerialPort() != PX4_OK) {
		return PX4_ERROR;
	}

	tcflush(_fd, TCIFLUSH);

	/* ------------------------- Initialization command ------------------------- */

	if (sendCommandAndExpectResponse(_cmd_INIT_default, INIT_PACKET_BYTES, "INIT") != PX4_OK) {
		return PX4_ERROR;
	}

	if (readVersionMessage(RFBEAM_SETUP_CMD_WAIT_US) != PX4_OK) {
		return PX4_ERROR;
	}

	/* --------------------------- Target filter mode --------------------------- */

	uint8_t *TGFI_msg = _cmd_TGFI_default;
	int32_t tgfi_setting = _param_sensor_tgfi.get();

	switch (tgfi_setting) {

	case RFBEAM_PARAM_TGFI_DEFAULT:
		PX4_INFO("DEBUG: target filter setting: nearest (default)");
		break;

	case 0:
		TGFI_msg = _cmd_TGFI_strong;
		PX4_INFO("DEBUG: target filter setting: strongest");
		break;

	case 2:
		TGFI_msg = _cmdTGFI_FAR;
		PX4_INFO("DEBUG: target filter setting: farthest");
		break;

	default:
		PX4_ERR("invalid target filter setting: %" PRId32, tgfi_setting);
		return PX4_ERROR;
	}

	if (sendCommandAndExpectResponse(TGFI_msg, TGFI_PACKET_BYTES, "TGFI") != PX4_OK) {
		return PX4_ERROR;
	}

	/* ------------------------------- Range mode ------------------------------- */

	uint8_t *RRAI_msg = _cmd_RRAI_default;
	int32_t range_setting = _param_sensor_range.get();

	float range_resolution_cm = 0;

	switch (range_setting) {
	case RFBEAM_PARAM_RNG_DEFAULT:
		PX4_INFO("DEBUG: max range setting: 20 m (default)");
		range_resolution_cm = RFBEAM_RANGE_RESOLUTION_20M_CM;
		break;

	case 1:
		RRAI_msg = _cmd_RRAI_50;
		PX4_INFO("DEBUG: max range setting: 50 m");
		range_resolution_cm = RFBEAM_RANGE_RESOLUTION_50M_CM;
		break;

	default:
		PX4_ERR("invalid range setting: %" PRId32, range_setting);
		return PX4_ERROR;
	}

	if (sendCommandAndExpectResponse(RRAI_msg, RRAI_PACKET_BYTES, "RRAI") != PX4_OK) {
		return PX4_ERROR;
	}

	/* --------------------------- Short range filter --------------------------- */

	uint8_t *SRDF_msg = _cmd_SRDF_default;
	int32_t short_range_filter_enabled = _param_sensor_srng.get();

	switch (short_range_filter_enabled) {
	case RFBEAM_PARAM_SRNG_DEFAULT:
		PX4_INFO("DEBUG: short range filter: off (default)");
		break;

	case 1:
		SRDF_msg = _cmd_SRDF_on;
		PX4_INFO("DEBUG: short range filter: on");
		break;

	default:
		PX4_ERR("invalid short range filter setting: %" PRId32, short_range_filter_enabled);
		return PX4_ERROR;
	}

	if (sendCommandAndExpectResponse(SRDF_msg, SRDF_PACKET_BYTES, "SRDF") != PX4_OK) {
		return PX4_ERROR;
	}

	/* -------------------------- Minimum range filter -------------------------- */

	uint8_t *MIRA_msg = _cmd_MIRA_default;
	int32_t min_range_setting = _param_sensor_minf.get();

	// Calculate and set minimum detection distance (according to datasheet)
	float min_distance_m = (min_range_setting * range_resolution_cm + RFBEAM_INTERNAL_OFFSET_CM) / 100.0f;
	_px4_rangefinder.set_min_distance(min_distance_m);
	PX4_INFO("DEBUG: min distance [m]: %f", (double)min_distance_m);

	switch (min_range_setting) {
	case RFBEAM_PARAM_MINF_DEFAULT:
		PX4_INFO("DEBUG: min range filter: bin %" PRId32 " (default)", (uint32_t)RFBEAM_PARAM_MINF_DEFAULT);
		break;

	default:
		if (min_range_setting >= RFBEAM_PARAM_MINF_MIN && min_range_setting <= RFBEAM_PARAM_MINF_MAX) {
			uint16_t min_range = (uint16_t)min_range_setting;
			// Fill payload section (little endian) of MIRA packet
			MIRA_msg[PACKET_PAYLOAD_START_IDX] = (uint8_t)(min_range & 0xFF); // mask away the higher byte
			MIRA_msg[PACKET_PAYLOAD_START_IDX + 1] = (uint8_t)(min_range >> 8); // shift away the lower byte
			PX4_INFO("DEBUG: min range filter: bin %" PRId32, min_range_setting);
			break;

		} else {
			PX4_ERR("invalid min range filter setting: %" PRId32, min_range_setting);
			return PX4_ERROR;
		}
	}

	if (sendCommandAndExpectResponse(MIRA_msg, MIRA_PACKET_BYTES, "MIRA") != PX4_OK) {
		return PX4_ERROR;
	}

	/* -------------------------- Maximum range filter -------------------------- */

	uint8_t *MARA_msg = _cmd_MARA_default;
	int32_t max_range_setting = _param_sensor_maxf.get();

	// Calculate and set maximum detection distance (according to datasheet)
	float max_distance_m = (max_range_setting * range_resolution_cm + RFBEAM_INTERNAL_OFFSET_CM) / 100.0f;
	_px4_rangefinder.set_max_distance(max_distance_m);
	PX4_INFO("DEBUG: max distance [m]: %f", (double)max_distance_m);

	switch (max_range_setting) {
	case RFBEAM_PARAM_MAXF_DEFAULT:
		PX4_INFO("DEBUG: max range filter: bin %" PRId32 " (default)", (uint32_t)RFBEAM_PARAM_MAXF_DEFAULT);
		break;

	default:
		if (max_range_setting >= RFBEAM_PARAM_MAXF_MIN && max_range_setting <= RFBEAM_PARAM_MAXF_MAX) {
			uint16_t max_range = (uint16_t)max_range_setting;
			// Fill payload section (LSB first) of MARA packet
			MARA_msg[PACKET_PAYLOAD_START_IDX] = (uint8_t)(max_range & 0xFF); // mask away the higher byte
			MARA_msg[PACKET_PAYLOAD_START_IDX + 1] = (uint8_t)(max_range >> 8); // shift away the lower byte
			PX4_INFO("DEBUG: max range filter: bin %" PRId32, max_range_setting);
			break;

		} else {
			PX4_ERR("invalid max range filter setting: %" PRId32 ".", max_range_setting);
			return PX4_ERROR;
		}
	}

	if (sendCommandAndExpectResponse(MARA_msg, MARA_PACKET_BYTES, "MARA") != PX4_OK) {
		return PX4_ERROR;
	}

	/* ---------------------------- Threshold offset ---------------------------- */

	uint8_t *THOF_msg = _cmd_THOF_default;
	int32_t threshold_offset = _param_sensor_thrs.get();

	switch (threshold_offset) {
	case RFBEAM_PARAM_THRS_DEFAULT:
		PX4_INFO("DEBUG: threshold offset: %" PRId32 " (default)", (uint32_t)RFBEAM_PARAM_THRS_DEFAULT);
		break;

	default:
		if (threshold_offset >= RFBEAM_PARAM_THRS_MIN && threshold_offset <= RFBEAM_PARAM_THRS_MAX) {
			// Fill payload section (LSB first) of THOF packet
			THOF_msg[PACKET_PAYLOAD_START_IDX] = (uint8_t)threshold_offset;
			PX4_INFO("DEBUG: threshold offset: %" PRId32, threshold_offset);
			break;

		} else {
			PX4_ERR("invalid threshold offset: %" PRId32, threshold_offset);
			return PX4_ERROR;
		}
	}

	if (sendCommandAndExpectResponse(THOF_msg, THOF_PACKET_BYTES, "THOF") != PX4_OK) {
		return PX4_ERROR;
	}

	/* --------------------------- Distance precision --------------------------- */

	if (sendCommandAndExpectResponse(_cmd_PREC_default, PREC_PACKET_BYTES, "PREC") != PX4_OK) {
		return PX4_ERROR;
	}

	/* ---------------------------- Chirp integration --------------------------- */

	uint8_t *INTN_msg = _cmd_INTN_default;
	int32_t chirp_count = _param_sensor_chrp.get();

	switch (chirp_count) {
	case RFBEAM_PARAM_CHRP_DEFAULT:
		PX4_INFO("DEBUG: chirp integration count: %" PRId32 " (default)", (uint32_t)RFBEAM_PARAM_CHRP_DEFAULT);
		break;

	default:
		if (chirp_count >= RFBEAM_PARAM_CHRP_MIN && chirp_count <= RFBEAM_PARAM_CHRP_MAX) {
			// Fill payload section (LSB first) of INTN packet
			INTN_msg[PACKET_PAYLOAD_START_IDX] = (uint8_t)chirp_count;
			PX4_INFO("DEBUG: chirp integration count: %" PRId32, chirp_count);
			break;

		} else {
			PX4_ERR("invalid chirp integration count: %" PRId32, chirp_count);
			return PX4_ERROR;
		}
	}

	if (sendCommandAndExpectResponse(INTN_msg, INTN_PACKET_BYTES, "INTN") != PX4_OK) {
		return PX4_ERROR;
	}


	/* --------------------------- Distance averaging --------------------------- */

	uint8_t *RAVG_msg = _cmd_RAVG_default;
	int32_t average = _param_sensor_avg.get();

	switch (average) {
	case RFBEAM_PARAM_AVG_DEFAULT:
		PX4_INFO("DEBUG: measurements to average: %" PRId32 " (default)", (uint32_t)RFBEAM_PARAM_AVG_DEFAULT);
		break;

	default:
		if (average >= RFBEAM_PARAM_AVG_MIN && average <= RFBEAM_PARAM_AVG_MAX) {
			// Fill payload section (LSB first) of RAVG packet
			RAVG_msg[PACKET_PAYLOAD_START_IDX] = (uint8_t)average;
			PX4_INFO("DEBUG: measurements to average: %" PRId32, average);
			break;

		} else {
			PX4_ERR("invalid average setting: %" PRId32, average);
			return PX4_ERROR;
		}
	}

	if (sendCommandAndExpectResponse(RAVG_msg, RAVG_PACKET_BYTES, "RAVG") != PX4_OK) {
		return PX4_ERROR;
	}


	/* ----------------------------------- End ---------------------------------- */

	// Calculate update interval
	if (short_range_filter_enabled) {
		_interval_us = (RFBEAM_FRAME_PROC_TIME_HP_MS + (chirp_count - 1) * (RFBEAM_CHIRP_COUNT_DELTA_T_MS) + chirp_count *
				RFBEAM_SHORT_RNG_DELTA_T_MS) * RFBEAM_MEASURE_INTERVAL_MULT;
	}

	else {
		_interval_us = (RFBEAM_FRAME_PROC_TIME_HP_MS + (chirp_count - 1) * (RFBEAM_CHIRP_COUNT_DELTA_T_MS)) *
			       RFBEAM_MEASURE_INTERVAL_MULT;
	}

	// TODO: why not * 1000 (already prints in us even though values in ms)???
	PX4_INFO("DEBUG: update interval [us]: %d", _interval_us);

	// Warn user if settings result in too low of an update rate (slower than 10 Hz)
	if (_interval_us > (1e6 / 10)) {
		PX4_WARN("low update rate warning [Hz]: %f", (double)(1e6 / _interval_us));
	}

	// Close the file descriptor
	::close(_fd);
	_fd = -1;

	start();

	return PX4_OK;
}

int RFbeamVLD1::measure()
{
	int bytes_written = ::write(_fd, _cmd_GNFD_PDAT, GNFD_PACKET_BYTES);

	if (bytes_written != GNFD_PACKET_BYTES) {
		perf_count(_comms_errors);
		PX4_ERR("measure cmd write fail %d", bytes_written);
		return bytes_written;
	}

	_measurement_time = hrt_absolute_time();
	_read_buffer_len = 0;
	return PX4_OK;
}

int RFbeamVLD1::collect()
{
	perf_begin(_sample_perf);

	const ssize_t bytes_read = ::read(_fd, _read_buffer + _read_buffer_len, sizeof(_read_buffer) - _read_buffer_len);

	if (bytes_read < 0) {
		if ((errno == EAGAIN) || (errno == EWOULDBLOCK)) {
			perf_end(_sample_perf);
			return -EAGAIN;
		}

		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	if (bytes_read == 0) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	_read_buffer_len += bytes_read;

	size_t start = 0;

	while ((start + PACKET_HEADER_BYTES) <= _read_buffer_len) {
		if (packet_has_header(&_read_buffer[start], "RESP")) {
			break;
		}

		start++;
	}

	if (start > 0) {
		memmove(_read_buffer, _read_buffer + start, _read_buffer_len - start);
		_read_buffer_len -= start;
	}

	if (_read_buffer_len < RESP_PACKET_BYTES) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	if (!packet_has_header(_read_buffer, "RESP")) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	const uint32_t resp_payload_len = packet_payload_len(_read_buffer);

	if (resp_payload_len != 1) {
		perf_count(_comms_errors);
		memmove(_read_buffer, _read_buffer + 1, _read_buffer_len - 1);
		_read_buffer_len--;
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	const size_t resp_total_size = PACKET_PAYLOAD_START_IDX + resp_payload_len;

	if (_read_buffer_len < resp_total_size) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	if (_read_buffer_len < (resp_total_size + PACKET_PAYLOAD_START_IDX)) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	if (_read_buffer_len < (resp_total_size + PDAT_PACKET_BYTES)) {
		if (hrt_elapsed_time(&_measurement_time) > RFBEAM_READ_TIMEOUT_US) {
			_read_buffer_len = 0;
			perf_end(_sample_perf);
			return PX4_OK;
		}

		perf_end(_sample_perf);
		return -EAGAIN;
	}

	const uint8_t *pdat = _read_buffer + resp_total_size;

	if (!packet_has_header(pdat, "PDAT")) {
		perf_count(_comms_errors);
		memmove(_read_buffer, _read_buffer + 1, _read_buffer_len - 1);
		_read_buffer_len--;
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	const uint32_t pdat_payload_len = packet_payload_len(pdat);

	if ((pdat_payload_len != 0) && (pdat_payload_len != sizeof(PDAT_msg))) {
		perf_count(_comms_errors);
		memmove(_read_buffer, _read_buffer + 1, _read_buffer_len - 1);
		_read_buffer_len--;
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	if (pdat_payload_len == 0) {
		_read_buffer_len = 0;
		perf_end(_sample_perf);
		return PX4_OK;
	}

	if (_read_buffer_len < (resp_total_size + PACKET_PAYLOAD_START_IDX + pdat_payload_len)) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	float distance_m = NAN;
	memcpy(&distance_m, pdat + PACKET_PAYLOAD_START_IDX, sizeof(distance_m));

	_read_time = hrt_absolute_time();
	_px4_rangefinder.update(_read_time, distance_m);
	_read_buffer_len = 0;

	perf_end(_sample_perf);
	return PX4_OK;
}

int RFbeamVLD1::readPacket(const char *expected_header, uint8_t *buffer, size_t buffer_size, size_t &packet_size,
			   hrt_abstime timeout_us)
{
	packet_size = 0;
	size_t buffer_len = 0;
	const hrt_abstime start = hrt_absolute_time();

	while (hrt_elapsed_time(&start) < timeout_us) {
		const ssize_t bytes_read = ::read(_fd, buffer + buffer_len, buffer_size - buffer_len);

		if (bytes_read < 0) {
			if ((errno == EAGAIN) || (errno == EWOULDBLOCK)) {
				px4_usleep(1000);
				continue;
			}

			perf_count(_comms_errors);
			return PX4_ERROR;
		}

		if (bytes_read == 0) {
			px4_usleep(1000);
			continue;
		}

		buffer_len += bytes_read;

		size_t start_idx = 0;

		while ((start_idx + PACKET_HEADER_BYTES) <= buffer_len) {
			if (packet_has_header(&buffer[start_idx], expected_header)) {
				break;
			}

			start_idx++;
		}

		if (start_idx > 0) {
			memmove(buffer, buffer + start_idx, buffer_len - start_idx);
			buffer_len -= start_idx;
		}

		if (buffer_len < PACKET_PAYLOAD_START_IDX) {
			continue;
		}

		const uint32_t payload_len = packet_payload_len(buffer);
		const size_t total_size = PACKET_PAYLOAD_START_IDX + payload_len;

		if (total_size > buffer_size) {
			perf_count(_comms_errors);
			return PX4_ERROR;
		}

		if (buffer_len < total_size) {
			continue;
		}

		packet_size = total_size;
		return PX4_OK;
	}

	return PX4_ERROR;
}

int RFbeamVLD1::readResponse(uint8_t &error_code, hrt_abstime timeout_us)
{
	uint8_t buffer[RESP_PACKET_BYTES] {};
	size_t packet_size = 0;

	if (readPacket("RESP", buffer, sizeof(buffer), packet_size, timeout_us) != PX4_OK) {
		PX4_ERR("RESP read timeout");
		return PX4_ERROR;
	}

	if ((packet_size != RESP_PACKET_BYTES) || (packet_payload_len(buffer) != 1)) {
		perf_count(_comms_errors);
		PX4_ERR("RESP malformed");
		return PX4_ERROR;
	}

	error_code = buffer[PACKET_PAYLOAD_START_IDX];
	return PX4_OK;
}

int RFbeamVLD1::sendCommandAndExpectResponse(const uint8_t *command, size_t command_size, const char *command_name,
					     hrt_abstime timeout_us)
{
	const int bytes_written = ::write(_fd, command, command_size);

	if (bytes_written != (int)command_size) {
		perf_count(_comms_errors);
		PX4_ERR("%s cmd write fail %d", command_name, bytes_written);
		return PX4_ERROR;
	}

	uint8_t error_code = 0xff;

	if (readResponse(error_code, timeout_us) != PX4_OK) {
		PX4_ERR("%s cmd no RESP", command_name);
		return PX4_ERROR;
	}

	if (error_code != 0) {
		perf_count(_comms_errors);
		PX4_ERR("%s RESP error %u", command_name, error_code);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int RFbeamVLD1::readVersionMessage(hrt_abstime timeout_us)
{
	uint8_t buffer[64] {};
	size_t packet_size = 0;

	if (readPacket("VERS", buffer, sizeof(buffer), packet_size, timeout_us) != PX4_OK) {
		PX4_ERR("VERS read timeout");
		return PX4_ERROR;
	}

	if (packet_payload_len(buffer) == 0 || packet_size > sizeof(buffer)) {
		perf_count(_comms_errors);
		PX4_ERR("VERS malformed");
		return PX4_ERROR;
	}

	char version[64] {};
	const uint32_t payload_len = packet_payload_len(buffer);
	const size_t version_len = (payload_len < (sizeof(version) - 1)) ? payload_len : (sizeof(version) - 1);
	memcpy(version, buffer + PACKET_PAYLOAD_START_IDX, version_len);
	PX4_INFO("sensor version: %s", version);
	return PX4_OK;
}

int RFbeamVLD1::openSerialPort(const speed_t speed)
{
	// Skip the rest if  descriptor already initialized
	if (_fd > 0) {
		PX4_DEBUG("serial port already open");
		return PX4_OK;
	}

	// Configure port flags (read/write, non-controlling, non-blocking)
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port
	_fd = ::open(_port, flags);

	if (_fd < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	if (!isatty(_fd)) {
		PX4_WARN("not a serial device");
		return PX4_ERROR;
	}

	struct termios uart_config;

	int termios_state;

	// Store the current port configuration attributes
	if (tcgetattr(_fd, &uart_config) != 0) {
		PX4_ERR("Unable to get termios (%i)", errno);
		::close(_fd);
		_fd = -1;
		return PX4_ERROR;
	}

	/* Input flags (turn off input processing):
	Convert break to null byte, no CR to NL translation,
	No NL to CR translation, don't mark parity errors or breaks
	No input parity check, don't strip high bit off,
	No XON/XOFF software flow control */
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON | IXOFF | IXANY);

	// Output flags (turn off output processing):
	uart_config.c_oflag &= ~(ONLCR | OPOST);

	// 8 data bits
	uart_config.c_cflag &= ~CSIZE;
	uart_config.c_cflag |= CS8;

	// Enable receiver
	uart_config.c_cflag |= CREAD;

	// Ignore modem status lines
	uart_config.c_cflag |= CLOCAL;

	// One stop bit (clear 2 stop bits flag)
	uart_config.c_cflag &= ~CSTOPB;

	// Even parity
	uart_config.c_cflag &= ~PARODD;

	uart_config.c_cflag |= PARENB;

	// No flow control (clear flow control flag)
	uart_config.c_cflag &= ~CRTSCTS;

	/* Turn off line processing:
	Echo off, echo newline off, canonical mode off, extended input processing off, signal chars off
	*/
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// Set the input baud rate in the uart_config struct
	termios_state = cfsetispeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_fd);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct
	termios_state = cfsetospeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_fd);
		return PX4_ERROR;
	}

	// Apply the modified port attributes
	termios_state = tcsetattr(_fd, TCSANOW, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_fd);
		return PX4_ERROR;
	}

	// Flush the hardware buffers // TODO: not sure if this is necessary
	// tcflush(_fd, TCIOFLUSH);

	PX4_INFO("successfully opened UART port %s", _port);
	return PX4_OK;
}

void RFbeamVLD1::Run()
{
	// Ensure the serial port is open
	if (openSerialPort() != PX4_OK) {
		perf_count(_comms_errors);
		start();
		return;
	}

	// Collection phase
	if (_collect_phase) {
		int ret = collect();

		if (ret != PX4_OK) {
			// Case 1/2: try again upon incomplete/failed read
			// TODO: the code in collect() that would trigger this case is currently commented out
			if (ret == -EAGAIN) {
				PX4_DEBUG("trying read again");
				ScheduleDelayed(5_ms);
			}

			// Case 2/2: data collection error (timeout)
			else {
				PX4_ERR("collection error");
				// Restart the measurement state machine i.e. skip collection on next iteration
				start();
			}

			return;
		}

		// Next phase is measurement
		_collect_phase = false;
	}

	// Measurement phase
	if (measure() != PX4_OK) {
		PX4_ERR("measure error");
	}

	// Next phase is collection
	_collect_phase = true;

	// Schedule a fresh cycle call when the measurement is done
	ScheduleDelayed(_interval_us);
}

int RFbeamVLD1::requestSensorSettings()
{
	int bytes_written = ::write(_fd, _cmd_GRPS, GRPS_PACKET_BYTES);

	if (bytes_written != GRPS_PACKET_BYTES) {
		perf_count(_comms_errors);
		PX4_ERR("GRPS cmd write fail %d", bytes_written);
		return bytes_written;
	}

	return PX4_OK;
}

int RFbeamVLD1::restoreFactorySettings()
{
	int bytes_written = ::write(_fd, _cmd_RFSE, RFSE_PACKET_BYTES);

	if (bytes_written != RFSE_PACKET_BYTES) {
		perf_count(_comms_errors);
		PX4_ERR("RFSE cmd write fail %d", bytes_written);
		return bytes_written;
	}

	return PX4_OK;
}

void RFbeamVLD1::start()
{
	// Reset the report ring and state machine
	_collect_phase = false;

	// Schedule a cycle to start things
	ScheduleDelayed(5); // 5 us
}

void RFbeamVLD1::stop()
{
	// Ensure the serial port is closed
	::close(_fd);

	// Clear the work queue schedule
	ScheduleClear();
}

void RFbeamVLD1::printInfo()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
