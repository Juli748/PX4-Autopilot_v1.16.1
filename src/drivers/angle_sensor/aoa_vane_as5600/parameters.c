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
 * Enable AoA vane calibration table
 *
 * When enabled, the AS5600 raw angle count is converted to angle of attack
 * using piecewise-linear interpolation between the configured raw count points.
 *
 * @boolean
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_AOA_CAL_EN, 0);

/**
 * AoA vane output sign
 *
 * Multiplies the final AoA output angle after calibration and wrapping.
 * Use -1 to reverse the AoA sign convention.
 *
 * @value -1 Reverse sign
 * @value 1 Normal sign
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_AOA_SIGN, 1);

/**
 * AoA vane slow filter factor
 *
 * Sets the AS5600 slow filter level. Higher values smooth more but respond slower.
 *
 * @value 16 Strongest smoothing
 * @value 8 Medium smoothing
 * @value 4 Light smoothing
 * @value 2 Lightest smoothing
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_AOA_SF, 16);

/**
 * AoA vane fast filter threshold
 *
 * Sets the AS5600 fast filter threshold in LSB. Set to 0 to disable the fast threshold
 * and use only the slow filter. Lower values react sooner to fast angle changes.
 *
 * Allowed values: 0, 6, 7, 9, 10, 18, 21, 24
 *
 * @min 0
 * @max 24
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_AOA_FTH, 0);

/**
 * AoA vane raw count at 0 degrees
 *
 * Raw AS5600 reading with the vane fixed at 0 degrees angle of attack.
 *
 * @min 0
 * @max 4095
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_AOA_RAW_0, 0);

/**
 * AoA vane raw count at 5 degrees
 *
 * Raw AS5600 reading with the vane fixed at 5 degrees angle of attack.
 *
 * @min 0
 * @max 4095
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_AOA_RAW_5, 0);

/**
 * AoA vane raw count at 10 degrees
 *
 * Raw AS5600 reading with the vane fixed at 10 degrees angle of attack.
 *
 * @min 0
 * @max 4095
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_AOA_RAW_10, 0);

/**
 * AoA vane raw count at 15 degrees
 *
 * Raw AS5600 reading with the vane fixed at 15 degrees angle of attack.
 *
 * @min 0
 * @max 4095
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_AOA_RAW_15, 0);

/**
 * AoA vane raw count at 20 degrees
 *
 * Raw AS5600 reading with the vane fixed at 20 degrees angle of attack.
 *
 * @min 0
 * @max 4095
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_AOA_RAW_20, 0);

/**
 * AoA vane raw count at 45 degrees
 *
 * Raw AS5600 reading with the vane fixed at 45 degrees angle of attack.
 *
 * @min 0
 * @max 4095
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_AOA_RAW_45, 0);
