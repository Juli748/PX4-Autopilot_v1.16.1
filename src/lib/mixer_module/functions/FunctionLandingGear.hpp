/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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

#pragma once

#include "FunctionProviderBase.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/slew_rate/SlewRate.hpp>
#include <parameters/param.h>
#include <uORB/topics/landing_gear.h>

/**
 * Functions: Landing_Gear
 */
class FunctionLandingGear : public FunctionProviderBase
{
public:
	FunctionLandingGear() = default;
	static FunctionProviderBase *allocate(const Context &context) { return new FunctionLandingGear(); }

	void update() override
	{
		const hrt_abstime now = hrt_absolute_time();
		landing_gear_s landing_gear;

		if (_topic.update(&landing_gear)) {
			if (landing_gear.landing_gear == landing_gear_s::GEAR_DOWN) {
				_target = -1.f;

			} else if (landing_gear.landing_gear == landing_gear_s::GEAR_UP) {
				_target = 1.f;
			}
		}

		float slew_time_s = 0.f;

		if (_param_slew_time != PARAM_INVALID) {
			param_get(_param_slew_time, &slew_time_s);
		}

		if (slew_time_s > FLT_EPSILON) {
			_slew_rate.setSlewRate(2.f / slew_time_s);

			if (_last_update == 0) {
				_slew_rate.setForcedValue(_target);

			} else {
				_slew_rate.update(_target, hrt_elapsed_time(&_last_update) * 1e-6f);
			}

			_data = _slew_rate.getState();

		} else {
			_slew_rate.setForcedValue(_target);
			_data = _target;
		}

		_last_update = now;
	}

	float value(OutputFunction func) override { return _data; }

private:
	uORB::Subscription _topic{ORB_ID(landing_gear)};
	param_t _param_slew_time{param_find("CA_GEAR_SLEW")};
	SlewRate<float> _slew_rate{-1.f};
	hrt_abstime _last_update{0};
	float _data{-1.f};
	float _target{-1.f};
};
