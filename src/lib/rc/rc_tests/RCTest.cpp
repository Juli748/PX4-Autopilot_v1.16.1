#include <unit_test.h>

#include <systemlib/err.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>

#define DSM_DEBUG
#include <lib/rc/sbus.h>
#include <lib/rc/dsm.h>
#include <lib/rc/st24.h>
#include <lib/rc/sumd.h>
#include <lib/rc/crsf.h>
#include <lib/rc/ghst.hpp>
#include <lib/rc/srxl2.h>

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
#define TEST_DATA_PATH "./test_data/"
#else
#define TEST_DATA_PATH CONFIG_BOARD_ROOT_PATH
#endif

extern "C" __EXPORT int rc_tests_main(int argc, char *argv[]);

class RCTest : public UnitTest
{
public:
	bool run_tests() override;

private:
	bool crsfTest();
	bool ghstTest();
	bool srxl2Test();
	bool dsmTest(const char *filepath, unsigned expected_chancount, unsigned expected_dropcount, unsigned chan0);
	bool dsmTest10Ch();
	bool dsmTest16Ch();
	bool dsmTest22msDSMX16Ch();
	bool dsmTestOrangeDsmx();
	bool sbus2Test();
	bool st24Test();
	bool sumdTest();
};

bool RCTest::run_tests()
{
	ut_run_test(crsfTest);
	ut_run_test(ghstTest);
	ut_run_test(srxl2Test);
	ut_run_test(dsmTest10Ch);
	ut_run_test(dsmTest16Ch);
	ut_run_test(dsmTest22msDSMX16Ch);
	ut_run_test(dsmTestOrangeDsmx);
	ut_run_test(sbus2Test);
	ut_run_test(st24Test);
	ut_run_test(sumdTest);

	return (_tests_failed == 0);
}

bool RCTest::crsfTest()
{
	const char *filepath = TEST_DATA_PATH "crsf_rc_channels.txt";

	FILE *fp = fopen(filepath, "rt");

	ut_test(fp);
	//PX4_INFO("loading data from: %s", filepath);

	const int line_size = 500;
	char line[line_size];
	bool has_decoded_values = false;
	const int max_channels = 16;
	uint16_t rc_values[max_channels];
	uint16_t num_values = 0;
	int line_counter = 1;

	while (fgets(line, line_size, fp) != nullptr)  {

		if (strncmp(line, "INPUT ", 6) == 0) {

			if (has_decoded_values) {
				PX4_ERR("Parser decoded values that are not in the test file (line=%i)", line_counter);
				return false;
			}

			// read the values
			const char *file_buffer = line + 6;
			int frame_len = 0;
			uint8_t frame[300];
			int offset;
			int number;

			while (sscanf(file_buffer, "%x, %n", &number, &offset) > 0) {
				frame[frame_len++] = number;
				file_buffer += offset;
			}

			// Pipe the data into the parser
			hrt_abstime now = hrt_absolute_time();

			bool result = crsf_parse(now, frame, frame_len, rc_values, &num_values, max_channels);

			if (result) {
				has_decoded_values = true;
			}

		} else if (strncmp(line, "DECODED ", 8) == 0) {

			if (!has_decoded_values) {
				PX4_ERR("Test file contains decoded values but the parser did not decode anything (line=%i)", line_counter);
				return false;
			}

			// read the values
			const char *file_buffer = line + 8;
			int offset;
			int expected_rc_value;
			int expected_num_channels = 0;

			while (sscanf(file_buffer, "%x, %n", &expected_rc_value, &offset) > 0) {

				// allow a small difference
				if (abs(expected_rc_value - (int)rc_values[expected_num_channels]) > 10) {
					PX4_ERR("File line: %i, channel: %i", line_counter, expected_num_channels);
					ut_compare("Wrong decoded channel", expected_rc_value, rc_values[expected_num_channels]);
				}

				file_buffer += offset;
				++expected_num_channels;
			}

			if (expected_num_channels != num_values) {
				PX4_ERR("File line: %d", line_counter);
				ut_compare("Unexpected number of decoded channels", expected_num_channels, num_values);
			}

			has_decoded_values = false;
		}

		++line_counter;
	}

	return true;
}

bool RCTest::ghstTest()
{
	const char *filepath = TEST_DATA_PATH "ghst_rc_channels.txt";

	FILE *fp = fopen(filepath, "rt");

	ut_test(fp);

	int uart_fd = -1;
	const int line_size = 500;
	char line[line_size];
	bool has_decoded_values = false;
	const int max_channels = 16;
	uint16_t rc_values[max_channels];
	uint16_t num_values = 0;
	int line_counter = 1;
	ghstLinkStatistics_t link_stats;
	ghst_config(uart_fd);

	while (fgets(line, line_size, fp) != nullptr)  {

		if (strncmp(line, "INPUT ", 6) == 0) {

			if (has_decoded_values) {
				PX4_ERR("Parser decoded values that are not in the test file (line=%i)", line_counter);
				return false;
			}

			// read the values
			const char *file_buffer = line + 6;
			int frame_len = 0;
			uint8_t frame[300];
			int offset;
			int number;

			while (sscanf(file_buffer, "%x, %n", &number, &offset) > 0) {
				frame[frame_len++] = number;
				file_buffer += offset;
			}

			// Pipe the data into the parser
			hrt_abstime now = hrt_absolute_time();

			bool result = ghst_parse(now, frame, frame_len, rc_values, &link_stats, &num_values, max_channels);

			if (result) {
				has_decoded_values = true;
			}

		} else if (strncmp(line, "DECODED ", 8) == 0) {

			if (!has_decoded_values) {
				PX4_ERR("Test file contains decoded values but the parser did not decode anything (line=%i)", line_counter);
				return false;
			}

			// read the values
			const char *file_buffer = line + 8;
			int offset;
			int expected_rc_value;
			int expected_num_channels = 0;

			while (sscanf(file_buffer, "%x, %n", &expected_rc_value, &offset) > 0) {

				// allow a small difference
				if (abs(expected_rc_value - (int)rc_values[expected_num_channels]) > 10) {
					PX4_ERR("File line: %i, channel: %i", line_counter, expected_num_channels);
					ut_compare("Wrong decoded channel", expected_rc_value, rc_values[expected_num_channels]);
				}

				file_buffer += offset;
				++expected_num_channels;
			}

			if (expected_num_channels != num_values) {
				PX4_ERR("File line: %d", line_counter);
				ut_compare("Unexpected number of decoded channels", expected_num_channels, num_values);
			}

			has_decoded_values = false;
		}

		++line_counter;
	}

	return true;
}

bool RCTest::srxl2Test()
{
	{
		uint8_t packet_buf[SRXL2_PACKET_LENGTH_MAX] {};
		const uint32_t uid = 0x12345678;
		const size_t packet_len = srxl2_build_handshake_packet(packet_buf, sizeof(packet_buf),
					  SRXL2_DEVICE_ID_FC_DEFAULT, 0x21, 10, 1, 0x03, uid);

		ut_compare("Handshake packet length", 14, packet_len);

		srxl2_reset_parser();
		srxl2_packet_t packet {};
		bool parsed = false;

		for (size_t i = 0; i < packet_len; ++i) {
			if (srxl2_parse_byte(packet_buf[i], &packet)) {
				parsed = true;
			}
		}

		ut_test(parsed);
		ut_compare("Wrong packet type", SRXL2_PACKET_TYPE_HANDSHAKE, packet.packet_type);
		ut_compare("Wrong src id", SRXL2_DEVICE_ID_FC_DEFAULT, packet.src_id);
		ut_compare("Wrong dest id", 0x21, packet.dest_id);
		ut_compare("Wrong priority", 10, packet.priority);
		ut_compare("Wrong baud support", 1, packet.baud_support);
		ut_compare("Wrong info", 0x03, packet.info);
		ut_compare("Wrong uid", uid, packet.uid);
	}

	{
		uint8_t packet_buf[] {
			SRXL2_ID,
			SRXL2_PACKET_TYPE_CONTROL_DATA,
			0x1C,
			SRXL2_CONTROL_CMD_CHANNEL_DATA,
			SRXL2_DEVICE_ID_FC_DEFAULT,
			0x58,
			0x0B, 0x00,
			0x37, 0x06, 0x00, 0x00,
			0xA0, 0x2A,
			0x00, 0x80,
			0x04, 0x80,
			0xFC, 0x7F,
			0x54, 0xD5,
			0xA0, 0x2A,
			0xA0, 0x2A,
			0x00, 0x00
		};

		const uint16_t crc = srxl2_crc16(packet_buf, sizeof(packet_buf) - 2);
		packet_buf[sizeof(packet_buf) - 2] = crc >> 8;
		packet_buf[sizeof(packet_buf) - 1] = crc & 0xFF;

		srxl2_reset_parser();
		srxl2_packet_t packet {};
		bool parsed = false;

		for (size_t i = 0; i < sizeof(packet_buf); ++i) {
			if (srxl2_parse_byte(packet_buf[i], &packet)) {
				parsed = true;
			}
		}

		ut_test(parsed);
		ut_compare("Wrong packet type", SRXL2_PACKET_TYPE_CONTROL_DATA, packet.packet_type);
		ut_compare("Wrong command", SRXL2_CONTROL_CMD_CHANNEL_DATA, packet.control_command);
		ut_compare("Wrong reply id", SRXL2_DEVICE_ID_FC_DEFAULT, packet.reply_id);
		ut_compare("Wrong RSSI", 0x58, packet.rssi);
		ut_compare("Wrong frame loss count", 11, packet.frame_loss_count);
		ut_compare("Wrong channel mask", 0x00000637, packet.channel_mask);
		ut_compare("Wrong channel count", 7, packet.num_channels);
		ut_compare("Wrong channel 1", 0x2AA0, packet.channel_data[0]);
		ut_compare("Wrong channel 2", 0x8000, packet.channel_data[1]);
		ut_compare("Wrong channel 3", 0x8004, packet.channel_data[2]);
		ut_compare("Wrong channel 5", 0x7FFC, packet.channel_data[3]);
		ut_compare("Wrong channel 6", 0xD554, packet.channel_data[4]);
		ut_compare("Wrong channel 10", 0x2AA0, packet.channel_data[5]);
		ut_compare("Wrong channel 11", 0x2AA0, packet.channel_data[6]);
		ut_compare("Wrong PWM conversion", 1500, srxl2_raw_to_pwm(0x8000));
	}

	{
		uint8_t packet_buf[] {
			SRXL2_ID,
			SRXL2_PACKET_TYPE_CONTROL_DATA,
			0x06,
			SRXL2_CONTROL_CMD_CHANNEL_DATA,
			SRXL2_DEVICE_ID_FC_DEFAULT,
			0x00
		};

		srxl2_reset_parser();
		srxl2_packet_t packet {};
		bool parsed = false;

		for (size_t i = 0; i < sizeof(packet_buf); ++i) {
			if (srxl2_parse_byte(packet_buf[i], &packet)) {
				parsed = true;
			}
		}

		ut_test(!parsed);
	}

	return true;
}

bool RCTest::dsmTest10Ch()
{
	return dsmTest(TEST_DATA_PATH "dsm_x_data.txt", 10, 2, 1500);
}

bool RCTest::dsmTest16Ch()
{
	return dsmTest(TEST_DATA_PATH "dsm_x_dx9_data.txt", 16, 1, 1500);
}

bool RCTest::dsmTest22msDSMX16Ch()
{
	return dsmTest(TEST_DATA_PATH "dsm_x_dx9_px4_binding_data.txt", 16, 1, 1499);
}

bool RCTest::dsmTestOrangeDsmx()
{
	return dsmTest(TEST_DATA_PATH "orangerx_dsmx_12.txt", 12, 1, 1499);
}

bool RCTest::dsmTest(const char *filepath, unsigned expected_chancount, unsigned expected_dropcount, unsigned chan0)
{
	FILE *fp;
	fp = fopen(filepath, "rt");

	ut_test(fp);
	//PX4_INFO("loading data from: %s", filepath);

	float f;
	unsigned x;
	int ret;

	// Trash the first 20 lines
	for (unsigned i = 0; i < 20; i++) {
		char buf[200];
		(void)fgets(buf, sizeof(buf), fp);
	}

	// Init the parser
	uint8_t frame[30];
	uint16_t rc_values[18];
	uint16_t num_values;
	bool dsm_11_bit;
	unsigned dsm_frame_drops = 0;
	uint16_t max_channels = sizeof(rc_values) / sizeof(rc_values[0]);

	int count = 0;
	unsigned last_drop = 0;

	dsm_proto_init();

	while (EOF != (ret = fscanf(fp, "%f,%x,,", &f, &x))) {

		if (ret <= 0) {
			fclose(fp);
			ut_test(ret > 0);
		}

		frame[0] = x;
		unsigned len = 1;

		// Pipe the data into the parser
		bool result = dsm_parse(f * 1e6f, &frame[0], len, rc_values, &num_values,
					&dsm_11_bit, &dsm_frame_drops, nullptr, max_channels);

		if (result) {
			if (count > (16 * 20)) { // need to process enough data to have full channel count
				ut_compare("num_values == expected_chancount", num_values, expected_chancount);
			}

			ut_test(abs((int)chan0 - (int)rc_values[0]) < 30);

			//PX4_INFO("decoded packet with %d channels and %s encoding:", num_values, (dsm_11_bit) ? "11 bit" : "10 bit");

			for (unsigned i = 0; i < num_values; i++) {
				//PX4_INFO("chan #%u:\t%d", i, (int)rc_values[i]);
			}
		}

		if (last_drop != (dsm_frame_drops)) {
			//PX4_INFO("frame dropped, now #%d", (dsm_frame_drops));
			last_drop = dsm_frame_drops;
		}

		count++;
	}

	fclose(fp);

	ut_compare("num_values == expected_chancount", num_values, expected_chancount);

	ut_test(ret == EOF);
	//PX4_INFO("drop: %d", (int)last_drop);
	ut_compare("last_drop == expected_dropcount", last_drop, expected_dropcount);

	return true;
}

bool RCTest::sbus2Test()
{
	const char *filepath = TEST_DATA_PATH "sbus2_r7008SB.txt";

	FILE *fp;
	fp = fopen(filepath, "rt");

	ut_test(fp);
	//PX4_INFO("loading data from: %s", filepath);

	float f;
	unsigned x;
	int ret;

	// Trash the first 20 lines
	for (unsigned i = 0; i < 20; i++) {
		char buf[200];
		(void)fgets(buf, sizeof(buf), fp);
	}

	// Init the parser
	uint8_t frame[SBUS_BUFFER_SIZE];
	uint16_t rc_values[18];
	uint16_t num_values;
	unsigned sbus_frame_drops = 0;
	unsigned sbus_frame_resets = 0;
	bool sbus_failsafe;
	bool sbus_frame_drop;
	uint16_t max_channels = sizeof(rc_values) / sizeof(rc_values[0]);

	unsigned last_drop = 0;

	while (EOF != (ret = fscanf(fp, "%f,%x,,", &f, &x))) {

		if (ret <= 0) {
			fclose(fp);
			ut_test(ret > 0);
		}

		frame[0] = x;
		unsigned len = 1;

		// Pipe the data into the parser
		hrt_abstime now = hrt_absolute_time();

		// if (rate_limiter % byte_offset == 0) {
		bool result = sbus_parse(now, &frame[0], len, rc_values, &num_values,
					 &sbus_failsafe, &sbus_frame_drop, &sbus_frame_drops, max_channels);

		if (result) {
			//PX4_INFO("decoded packet");
		}

		// }

		if (last_drop != (sbus_frame_drops + sbus_frame_resets)) {
			PX4_WARN("frame dropped, now #%d", (sbus_frame_drops + sbus_frame_resets));
			last_drop = sbus_frame_drops + sbus_frame_resets;
		}

	}

	ut_test(ret == EOF);

	return true;
}

bool RCTest::st24Test()
{
	const char *filepath = TEST_DATA_PATH "st24_data.txt";

	//PX4_INFO("loading data from: %s", filepath);

	FILE *fp;

	fp = fopen(filepath, "rt");
	ut_test(fp);

	float f;
	unsigned x;
	int ret;

	// Trash the first 20 lines
	for (unsigned i = 0; i < 20; i++) {
		char buf[200];
		(void)fgets(buf, sizeof(buf), fp);
	}

	float last_time = 0;

	while (EOF != (ret = fscanf(fp, "%f,%x,,", &f, &x))) {

		if (ret <= 0) {
			fclose(fp);
			ut_test(ret > 0);
		}

		if (((f - last_time) * 1000 * 1000) > 3000) {
			// PX4_INFO("FRAME RESET\n\n");
		}

		uint8_t b = static_cast<uint8_t>(x);

		last_time = f;

		// Pipe the data into the parser
		//hrt_abstime now = hrt_absolute_time();

		uint8_t rssi;
		uint8_t rx_count;
		uint16_t channel_count;
		uint16_t channels[20];

		if (!st24_decode(b, &rssi, &rx_count, &channel_count, channels, sizeof(channels) / sizeof(channels[0]))) {
			//PX4_INFO("decoded: %u channels (converted to PPM range)", (unsigned)channel_count);

			for (unsigned i = 0; i < channel_count; i++) {
				//int16_t val = channels[i];
				//PX4_INFO("channel %u: %d 0x%03X", i, static_cast<int>(val), static_cast<int>(val));
			}
		}
	}

	ut_test(ret == EOF);

	return true;
}

bool RCTest::sumdTest()
{
	const char *filepath = TEST_DATA_PATH "sumd_data.txt";

	//PX4_INFO("loading data from: %s", filepath);

	FILE *fp;

	fp = fopen(filepath, "rt");
	ut_test(fp);

	float f;
	unsigned x;
	int ret;

	// Trash the first 20 lines
	for (unsigned i = 0; i < 20; i++) {
		char buf[200];
		(void)fgets(buf, sizeof(buf), fp);
	}

	float last_time = 0;

	while (EOF != (ret = fscanf(fp, "%f,%x,,", &f, &x))) {

		if (ret <= 0) {
			fclose(fp);
			ut_test(ret > 0);
		}

		if (((f - last_time) * 1000 * 1000) > 3000) {
			// PX4_INFO("FRAME RESET\n\n");
		}

		uint8_t b = static_cast<uint8_t>(x);

		last_time = f;

		// Pipe the data into the parser
		//hrt_abstime now = hrt_absolute_time();

		uint8_t rssi;
		uint8_t rx_count;
		uint16_t channel_count;
		uint16_t channels[32];
		bool sumd_failsafe;


		if (!sumd_decode(b, &rssi, &rx_count, &channel_count, channels, 32, &sumd_failsafe)) {
			//PX4_INFO("decoded: %u channels (converted to PPM range)", (unsigned)channel_count);

			for (unsigned i = 0; i < channel_count; i++) {
				//int16_t val = channels[i];
				//PX4_INFO("channel %u: %d 0x%03X", i, static_cast<int>(val), static_cast<int>(val));
			}
		}
	}

	ut_test(ret == EOF);

	return true;
}

ut_declare_test_c(rc_tests_main, RCTest)
