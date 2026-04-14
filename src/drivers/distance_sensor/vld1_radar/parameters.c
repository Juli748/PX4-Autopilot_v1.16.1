/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * Enable the RFbeam V-LD1 radar distance sensor (UART)
 *
 * @reboot_required true
 *
 * @boolean
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_EN_VLD1, 0);

/**
 * Configure the target filter of the RFbeam V-LD1 radar distance sensor
 *
 * This parameter configures which detected target's distance should be used.
 *
 * @reboot_required true
 * @min 0
 * @max 2
 * @group Sensors
 *
 * @value 0 Strongest target
 * @value 1 Nearest target
 * @value 2 Farthest target
 */
PARAM_DEFINE_INT32(SENS_VLD1_TGFI, 1);

/**
 * Set the maximum range of the RFbeam V-LD1 radar distance sensor
 *
 * This parameter sets the range mode in meters.
 *
 * @reboot_required true
 * @min 0
 * @max 1
 * @group Sensors
 *
 * @value 0 20 m
 * @value 1 50 m
 */
PARAM_DEFINE_INT32(SENS_VLD1_RNG, 0);

/**
 * Enable the short range filter of the RFbeam V-LD1 radar distance sensor
 *
 * This parameter enables/disables the short range filter.
 *
 * @reboot_required true
 *
 * @boolean
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_VLD1_SRNG, 0);

/**
 * Configure the minimum range filter of the RFbeam V-LD1 radar distance sensor
 *
 * This parameter sets the FFT frequency bin above which targets are detected.
 *
 * @reboot_required true
 * @min 1
 * @max 510
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_VLD1_MINF, 5);

/**
 * Configure the maximum range filter of the RFbeam V-LD1 radar distance sensor
 *
 * This parameter sets the FFT frequency bin below which targets are detected.
 *
 * @reboot_required true
 * @min 2
 * @max 511
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_VLD1_MAXF, 460);

/**
 * Configure the threshold offset of the RFbeam V-LD1 radar distance sensor
 *
 * This parameter sets the level (in dB) above which targets are detected.
 *
 * @reboot_required true
 * @min 20
 * @max 90
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_VLD1_THRS, 40);

/**
 * Configure the chirp integration of the RFbeam V-LD1 radar distance sensor
 *
 * This parameter sets the number of FMCW sweeps to integrate per measurement.
 *
 * @reboot_required true
 * @min 1
 * @max 100
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_VLD1_CHRP, 1);

/**
 * Configure the averaging of the RFbeam V-LD1 radar distance sensor
 *
 * This parameter sets the number of measurements to average before outputting.
 *
 * @reboot_required true
 * @min 1
 * @max 255
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_VLD1_AVG, 5);
