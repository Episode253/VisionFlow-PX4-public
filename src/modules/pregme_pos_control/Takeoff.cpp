#include "Takeoff.hpp"

#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

void Takeoff::generateInitialRampValue(float velocity_p_gain)
{
	// Keep the parameter path intact, but make the ramp start from a conservative
	// zero upward-speed limit and grow toward the requested takeoff speed.
	velocity_p_gain = math::max(fabsf(velocity_p_gain), 0.01f);
	_takeoff_ramp_vz_init = CONSTANTS_ONE_G / velocity_p_gain;
}

void Takeoff::updateTakeoffState(const bool armed, const bool landed, const bool want_takeoff,
				 const float takeoff_desired_vz, const bool skip_takeoff, const hrt_abstime &now_us)
{
	_spoolup_time_hysteresis.set_state_and_update(armed, now_us);

	switch (_takeoff_state) {
	case TakeoffState::disarmed:
		if (armed) {
			_takeoff_state = TakeoffState::spoolup;
		} else {
			break;
		}
		// FALLTHROUGH
	case TakeoffState::spoolup:
		if (_spoolup_time_hysteresis.get_state()) {
			_takeoff_state = TakeoffState::ready_for_takeoff;
		} else {
			break;
		}
		// FALLTHROUGH
	case TakeoffState::ready_for_takeoff:
		if (want_takeoff) {
			_takeoff_state = TakeoffState::rampup;
			_takeoff_ramp_progress = 0.f;
		} else {
			break;
		}
		// FALLTHROUGH
	case TakeoffState::rampup:
		if (_takeoff_ramp_progress >= 1.f) {
			_takeoff_state = TakeoffState::flight;
		} else {
			break;
		}
		// FALLTHROUGH
	case TakeoffState::flight:
		// During the first part of takeoff the land detector can still report
		// landed/ground_contact. Do not drop back to ready_for_takeoff while
		// a takeoff request is still active, otherwise the ramp is repeatedly
		// restarted and the aircraft may stay on fast idle without leaving ground.
		if (landed && !want_takeoff) {
			_takeoff_state = TakeoffState::ready_for_takeoff;
			_takeoff_ramp_progress = 0.f;
		}
		break;
	default:
		break;
	}

	if (armed && skip_takeoff) {
		_takeoff_state = TakeoffState::flight;
	}

	if (!armed) {
		_takeoff_state = TakeoffState::disarmed;
		_takeoff_ramp_progress = 0.f;
	}
}

float Takeoff::updateRamp(const float dt, const float takeoff_desired_vz)
{
	float upwards_velocity_limit = 0.f;

	if (_takeoff_state >= TakeoffState::flight) {
		upwards_velocity_limit = takeoff_desired_vz;
		return math::max(upwards_velocity_limit, 0.f);
	}

	if (_takeoff_state == TakeoffState::rampup) {
		if (_takeoff_ramp_time > dt) {
			_takeoff_ramp_progress += dt / _takeoff_ramp_time;

		} else {
			_takeoff_ramp_progress = 1.f;
		}

		_takeoff_ramp_progress = math::constrain(_takeoff_ramp_progress, 0.f, 1.f);
		upwards_velocity_limit = _takeoff_ramp_progress * takeoff_desired_vz;
	}

	return math::max(upwards_velocity_limit, 0.f);
}
