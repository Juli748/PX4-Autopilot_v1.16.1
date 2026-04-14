#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include "RFbeamVLD1.hpp"

namespace vld1_radar
{
RFbeamVLD1 *g_dev { nullptr };

static int start(const char *port, uint8_t rotation)
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return PX4_ERROR;
	}

	if (port == nullptr) {
		PX4_ERR("serial port required");
		return PX4_ERROR;
	}

	// Instantiate the driver
	g_dev = new RFbeamVLD1(port, rotation);

	if (g_dev == nullptr) {
		PX4_ERR("driver start failed");
		return PX4_ERROR;
	}

	if (g_dev->init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

static int stop()
{
	if (g_dev != nullptr) {
		PX4_INFO("stopping driver");
		delete g_dev;
		g_dev = nullptr;
		PX4_INFO("driver stopped");

	} else {
		return PX4_ERROR;
	}

	return PX4_OK;
}

static int status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	g_dev->printInfo();

	return PX4_OK;
}

static int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the RFbeam V-LD1 radar.

Setup/usage information: TODO

### Examples

Attempt to start driver on a specified serial device.
$ vld1_radar start -d /dev/ttyS1
Stop driver
$ vld1_radar stop
)DESCR_STR");

        PRINT_MODULE_USAGE_NAME("vld1_radar", "driver");
        PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
        PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
        PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "Serial device", false);
        PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation - downward facing by default", true);
        PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
        return PX4_OK;
}

}  // namespace vld1_radar

extern "C" __EXPORT int vld1_radar_main(int argc, char *argv[]) {
        uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
        const char *device_path = nullptr;
        int ch;
        // Option index
        int myoptind = 1;
        // Option argument
        const char *myoptarg = nullptr;

        while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
                switch (ch) {
                        case 'R':
                                rotation = (uint8_t)atoi(myoptarg);
                                break;

                        case 'd':
                                device_path = myoptarg;
                                break;

                        default:
                                return vld1_radar::usage();
                }
        }

        if (myoptind >= argc) {
                return vld1_radar::usage();
        }

        if (!strcmp(argv[myoptind], "start")) {
                return vld1_radar::start(device_path, rotation);

        } else if (!strcmp(argv[myoptind], "stop")) {
                return vld1_radar::stop();

        } else if (!strcmp(argv[myoptind], "status")) {
                return vld1_radar::status();
        }

        return vld1_radar::usage();
}
