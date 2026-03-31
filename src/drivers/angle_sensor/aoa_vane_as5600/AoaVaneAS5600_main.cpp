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

#include "AoaVaneAS5600.hpp"

#include <cstring>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

void AoaVaneAS5600::print_usage()
{
	PRINT_MODULE_USAGE_NAME("aoa_vane_as5600", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("angle_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(AoaVaneAS5600::I2C_ADDRESS_DEFAULT);
	PRINT_MODULE_USAGE_PARAM_STRING('R', "aoa", "aoa|ssa", "Sensor role", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int aoa_vane_as5600_main(int argc, char *argv[])
{
	using ThisDriver = AoaVaneAS5600;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = ThisDriver::I2C_SPEED_DEFAULT;
	cli.i2c_address = ThisDriver::I2C_ADDRESS_DEFAULT;
	cli.custom1 = static_cast<int32_t>(ThisDriver::SensorRole::Aoa);

	int ch;

	while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			if (strcmp(cli.optArg(), "aoa") == 0) {
				cli.custom1 = static_cast<int32_t>(ThisDriver::SensorRole::Aoa);

			} else if ((strcmp(cli.optArg(), "ssa") == 0) || (strcmp(cli.optArg(), "sideslip") == 0)) {
				cli.custom1 = static_cast<int32_t>(ThisDriver::SensorRole::Sideslip);

			} else {
				PX4_ERR("unknown role");
				return -1;
			}

			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_SENS_DEVTYPE_AOA_VANE_AS5600);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
