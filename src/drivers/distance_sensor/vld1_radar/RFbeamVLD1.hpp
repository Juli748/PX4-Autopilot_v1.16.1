#pragma once

#include <termios.h>
#include <unistd.h>

#include <lib/perf/perf_counter.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>
#include <px4_platform_common/module.h>
#include <fcntl.h>
#include <poll.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

using namespace time_literals;

/* -------------------------------------------------------------------------- */
/*                                   Defines                                  */
/* -------------------------------------------------------------------------- */

/* ---------------------------------- Basic --------------------------------- */
#define RFBEAM_STARTUP_TIME_MS		15_ms

// Internal offset needed for min and max range calculation
#define RFBEAM_INTERNAL_OFFSET_CM	-2.1f

// Range resolution in low precision mode and 20 m range setting
#define RFBEAM_RANGE_RESOLUTION_20M_CM	3.934f

// Range resolution in low precision mode and 50 m range setting
#define RFBEAM_RANGE_RESOLUTION_50M_CM	9.943f

// Additional time per chirp integration
#define RFBEAM_CHIRP_COUNT_DELTA_T_MS	3_ms

// Additional time if short range filter enabled (per chirp integration)
#define RFBEAM_SHORT_RNG_DELTA_T_MS	5_ms

// Processing time in high precision mode (we are always in that mode atm) and chirp integration count 1
#define RFBEAM_FRAME_PROC_TIME_HP_MS	21_ms

// Resolution in high precision mode (we are always in that mode atm)
#define RF_BEAM_RESOLUTION_HP_M		0.001f

// Time to wait after sending a command to change sensor settings
#define RFBEAM_SETUP_CMD_WAIT_US	50000

// Update interval above which a warning will be logged
#define RFBEAM_MAX_MEASURE_INTERVAL_MS	100_ms // 10 Hz

// Multiplicative factor to increase update interval so that sensor doesn't run at full capacity
#define RFBEAM_MEASURE_INTERVAL_MULT	1.2

/* ----------------------- Default, max, min settings ----------------------- */

// Target filter
#define RFBEAM_PARAM_TGFI_DEFAULT 	1

// Range mode
#define RFBEAM_PARAM_RNG_DEFAULT 	0

// Short range filter
#define RFBEAM_PARAM_SRNG_DEFAULT 	0

// Minimum range filter
#define RFBEAM_PARAM_MINF_DEFAULT 	5
#define RFBEAM_PARAM_MINF_MIN		1
#define RFBEAM_PARAM_MINF_MAX 		510

// Maximum range filter
#define RFBEAM_PARAM_MAXF_DEFAULT 	460
#define RFBEAM_PARAM_MAXF_MIN		2
#define RFBEAM_PARAM_MAXF_MAX 		511

// Threshold offset
#define RFBEAM_PARAM_THRS_DEFAULT 	40
#define RFBEAM_PARAM_THRS_MIN	 	20
#define RFBEAM_PARAM_THRS_MAX 		90

// Chirp integration
#define RFBEAM_PARAM_CHRP_DEFAULT 	1
#define RFBEAM_PARAM_CHRP_MIN 		1
#define RFBEAM_PARAM_CHRP_MAX 		100

// Distance averaging
#define RFBEAM_PARAM_AVG_DEFAULT 	5
#define RFBEAM_PARAM_AVG_MIN 		1
#define RFBEAM_PARAM_AVG_MAX	 	255

/**
 * Assume standard deviation to be equal to sensor resolution.
 * Static bench tests have shown that the sensor output does
 * not vary if the unit is not moved.
 */
#define SENS_VARIANCE			RF_BEAM_RESOLUTION_HP_M * RF_BEAM_RESOLUTION_HP_M

/* --------------------------------- Packets -------------------------------- */

#define PACKET_HEADER_BYTES                     4 // Number of bytes for  packet header
#define PACKET_PAYLOAD_LENGTH_BYTES             4 // Number of bytes to specify payload length

#define PACKET_PAYLOAD_START_IDX 		PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES

// Driver -> radar (requests)
#define INIT_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of INIT command
#define GNFD_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of GNFD command
#define GRPS_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 0	// Payload bytes of GRPS command
#define RFSE_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 0	// Payload bytes of RFSE command
#define GBYE_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 0	// Payload bytes of GBYE command
#define RRAI_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of RRAI command
#define THOF_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of THOF command
#define MIRA_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 2	// Payload bytes of MIRA command
#define MARA_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 2	// Payload bytes of MARA command
#define RAVG_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of RAVG command
#define TGFI_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of TGFI command
#define PREC_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of PREC command
#define TXPW_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of TXPW command
#define INTN_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of INTN command
#define SRDF_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of SRDF command
#define JBTL_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 0	// Payload bytes of JBTL command

// Radar -> driver (responses)
#define RESP_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1			// Payload bytes of RESP response
#define RADC_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 2048			// Payload bytes of RADC response
#define DONE_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 4			// Payload bytes of DONE response
#define PDAT_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + sizeof(PDAT_msg)	// Payload bytes of PDAT response (if a target is detected, otherwise zero payload)

struct __attribute__((__packed__)) PDAT_msg {
	float distance;
	uint16_t mag;
};

class RFbeamVLD1 : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	/**
	 * @brief Default constructor
	 *
	 * @param port The serial port to open for communicating with the sensor.
	 * @param rotation The sensor rotation relative to the vehicle body.
	 */
	RFbeamVLD1(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	~RFbeamVLD1() override;

	/**
	 * @brief Send INIT command to sensor
	 *
	 * @return int
	 */
	int init();

	/**
	 * @brief Print some basic information about the driver.
	 *
	 */
	void printInfo();

private:
	/**
	 * @brief Reads data from serial UART and places it into a buffer.
	 *
	 * @return int
	 */
	int collect();

	/**
	 * @brief Send command to obtain reading from sensor.
	 *
	 * @return int
	 */
	int measure();

	/**
	 * @brief Opens and configures the serial communications port.
	 *
	 * @param speed The baudrate (speed) to configure the serial UART port.
	 * @return int
	 */
	int openSerialPort(const speed_t speed = B115200);

	/**
	 * @brief Request or read measurement at regular intervals.
	 *
	 */
	void Run() override;

	/**
	 * @brief Initialise the automatic measurement state machine and start it.
	 *
	 */
	void start();

	/**
	 * @brief Stop the automatic measurement state machine.
	 *
	 */
	void stop();

	/**
	 * @brief Send a GRPS command to the sensor via UART (for debugging).
	 *
	 * @return int
	 */
	int requestSensorSettings();

	/**
	 * @brief Send a RFSE command to the sensor via UART.
	 *
	 * @return int
	 */
	int restoreFactorySettings();

	PX4Rangefinder _px4_rangefinder;

	char _port[20] = {};

	int _fd = -1;

	uint8_t _read_buffer[50] = {}; // TODO: don't hardcode size
	// uint8_t _read_buffer_len = 0;

	// hrt_abstime _last_read_time = 0;
	hrt_abstime _read_time = 0;

	int _interval_us = RFBEAM_MAX_MEASURE_INTERVAL_MS;

	bool _collect_phase = false;

	perf_counter_t _comms_errors = perf_alloc(PC_COUNT, MODULE_NAME ": comm_err");
	perf_counter_t _sample_perf = perf_alloc(PC_ELAPSED, MODULE_NAME ": read");

	/* -------------------------------------------------------------------------- */
	/*                    Message definitions (from datasheet)                    */
	/* -------------------------------------------------------------------------- */

	/* ---------------------------------- INIT ---------------------------------- */

	// Initialize with a baudrate of 115200 bit/s (default): {INIT, 1, 0}
	uint8_t _cmd_INIT_default[INIT_PACKET_BYTES] = {0x49, 0x4E, 0x49, 0x54, 0x01, 0x00, 0x00, 0x00, 0x00};

	/* ---------------------------------- GNFD ---------------------------------- */

	// Request distance to target: {GNFD, 1, 4}
	uint8_t _cmd_GNFD_PDAT[GNFD_PACKET_BYTES] = {0x47, 0x4E, 0x46, 0x44, 0x01, 0x00, 0x00, 0x00, 0x04};

	/* ---------------------------------- TGFI ---------------------------------- */

	// Target filter: strongest first: {TGFI, 1, 0}
	uint8_t _cmd_TGFI_strong[TGFI_PACKET_BYTES] = {0x54, 0x47, 0x46, 0x49, 0x01, 0x00, 0x00, 0x00, 0x00};

	// Target filter: nearest first (default): {TGFI, 1, 1}
	// TODO: use (uint8_t)RFBEAM_PARAM_TGFI_DEFAULT as payload
	uint8_t _cmd_TGFI_near[TGFI_PACKET_BYTES] = {0x54, 0x47, 0x46, 0x49, 0x01, 0x00, 0x00, 0x00, 0x01};

	// Target filter: farthest first: {TGFI, 1, 2}
	uint8_t _cmdTGFI_FAR[TGFI_PACKET_BYTES] = {0x54, 0x47, 0x46, 0x49, 0x01, 0x00, 0x00, 0x00, 0x02};

	uint8_t *_cmd_TGFI_default = _cmd_TGFI_near;

	/* ---------------------------------- RRAI ---------------------------------- */

	// Max range setting: 20 m (default): {RRAI, 1, 0}
	// TODO: use (uint8_t)RFBEAM_PARAM_RNG_DEFAULT as payload
	uint8_t _cmd_RRAI_20[RRAI_PACKET_BYTES] = {0x52, 0x52, 0x41, 0x49, 0x01, 0x00, 0x00, 0x00, 0x00};

	// Max range setting: 50 m: {RRAI, 1, 1}
	uint8_t _cmd_RRAI_50[RRAI_PACKET_BYTES] = {0x52, 0x52, 0x41, 0x49, 0x01, 0x00, 0x00, 0x00, 0x01};

	uint8_t *_cmd_RRAI_default = _cmd_RRAI_20;

	/* ---------------------------------- SRDF ---------------------------------- */

	// Short range filter off (default): {SRDF, 1, 0}
	// TODO: use (uint8_t)RFBEAM_PARAM_SRNG_DEFAULT as payload
	uint8_t _cmd_SRDF_off[SRDF_PACKET_BYTES] = {0x53, 0x52, 0x44, 0x46, 0x01, 0x00, 0x00, 0x00, 0x00};

	// Short range filter on: {SRDF, 1, 1}
	uint8_t _cmd_SRDF_on[SRDF_PACKET_BYTES] = {0x53, 0x52, 0x44, 0x46, 0x01, 0x00, 0x00, 0x00, 0x01};

	uint8_t *_cmd_SRDF_default = _cmd_SRDF_off;

	/* ---------------------------------- MIRA ---------------------------------- */

	// Minimum range filter (default): {MIRA, 2, 5}
	// TODO: use (uint16_t)RFBEAM_PARAM_MINF_DEFAULT as payload
	uint8_t _cmd_MIRA_default[MIRA_PACKET_BYTES] = {0x4D, 0x49, 0x52, 0x41, 0x02, 0x00, 0x00, 0x00, 0x05, 0x00};

	/* ---------------------------------- MARA ---------------------------------- */

	// Maximum range filter (default): {MARA, 2, 460}
	// TODO: use (uint16_t)RFBEAM_PARAM_MAXF_DEFAULT as payload
	uint8_t _cmd_MARA_default[MARA_PACKET_BYTES] = {0x4D, 0x41, 0x52, 0x41, 0x02, 0x00, 0x00, 0x00, 0xCC, 0x01};

	/* ---------------------------------- THOF ---------------------------------- */

	// Threshold offset (default): {THOF, 1, 40}
	// TODO: use (uint8_t)RFBEAM_PARAM_THRS_DEFAULT as payload
	uint8_t _cmd_THOF_default[THOF_PACKET_BYTES] = {0x54, 0x48, 0x4F, 0x46, 0x01, 0x00, 0x00, 0x00, 0x28};

	/* ---------------------------------- INTN ---------------------------------- */

	// Chirp integration count (default): {INTN, 1, 1}
	// TODO: use (uint8_t)RFBEAM_PARAM_CHRP_DEFAULT as payload
	uint8_t _cmd_INTN_default[INTN_PACKET_BYTES] = {0x49, 0x4E, 0x54, 0x4E, 0x01, 0x00, 0x00, 0x00, 0x01};

	/* ---------------------------------- RAVG ---------------------------------- */

	// Distance average count (default): {RAVG, 1, 5}
	// TODO: use (uint8_t)RFBEAM_PARAM_AVG_DEFAULT as payload
	uint8_t _cmd_RAVG_default[RAVG_PACKET_BYTES] = {0x52, 0x41, 0x56, 0x47, 0x01, 0x00, 0x00, 0x00, 0x05};


	/* -------------------------------------------------------------------------- */
	/*                          Other message definitions                         */
	/* -------------------------------------------------------------------------- */

	/* ---------------------------------- PREC ---------------------------------- */

	// High precision mode (default): {PREC, 1, 1}
	uint8_t _cmd_PREC_high[PREC_PACKET_BYTES] = {0x50, 0x52, 0x45, 0x43, 0x01, 0x00, 0x00, 0x00, 0x01};

	// Low precision mode: {PREC, 1, 0}
	uint8_t _cmd_PREC_low[PREC_PACKET_BYTES] = {0x50, 0x52, 0x45, 0x43, 0x01, 0x00, 0x00, 0x00, 0x00};

	uint8_t *_cmd_PREC_default = _cmd_PREC_high;

	/* ---------------------------------- RFSE ---------------------------------- */

	// Restore factory settings: {RFSE, 0}
	uint8_t _cmd_RFSE[RFSE_PACKET_BYTES] = {0x52, 0x46, 0x53, 0x45, 0x00, 0x00, 0x00, 0x00};

	/* ---------------------------------- GRPS ---------------------------------- */

	// Request parameter settings: {GRPS, 0}
	uint8_t _cmd_GRPS[GRPS_PACKET_BYTES] = {0x47, 0x52, 0x50, 0x53, 0x00, 0x00, 0x00, 0x00};

	/* -------------------------------------------------------------------------- */
	/*                                 Parameters                                 */
	/* -------------------------------------------------------------------------- */

	uORB::Subscription _parameter_update_sub = ORB_ID(parameter_update);

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_EN_VLD1>)    _param_sensor_enabled,
		(ParamInt<px4::params::SENS_VLD1_TGFI>)  _param_sensor_tgfi,
		(ParamInt<px4::params::SENS_VLD1_RNG>)  _param_sensor_range,
		(ParamInt<px4::params::SENS_VLD1_SRNG>)  _param_sensor_srng,
		(ParamInt<px4::params::SENS_VLD1_MINF>)  _param_sensor_minf,
		(ParamInt<px4::params::SENS_VLD1_MAXF>)  _param_sensor_maxf,
		(ParamInt<px4::params::SENS_VLD1_THRS>)  _param_sensor_thrs,
		(ParamInt<px4::params::SENS_VLD1_CHRP>)  _param_sensor_chrp,
		(ParamInt<px4::params::SENS_VLD1_AVG>)  _param_sensor_avg
	);
};
