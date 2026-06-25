#pragma once

#include <drivers/drv_hrt.h>
#include <lib/hysteresis/hysteresis.h>
#include <uORB/topics/takeoff_status.h>

using namespace time_literals;

enum class TakeoffState {
	disarmed = takeoff_status_s::TAKEOFF_STATE_DISARMED,
	spoolup = takeoff_status_s::TAKEOFF_STATE_SPOOLUP,
	ready_for_takeoff = takeoff_status_s::TAKEOFF_STATE_READY_FOR_TAKEOFF,
	rampup = takeoff_status_s::TAKEOFF_STATE_RAMPUP,
	flight = takeoff_status_s::TAKEOFF_STATE_FLIGHT
};

class Takeoff
{
public:
	Takeoff() = default;
	~Takeoff() = default;

	void setSpoolupTime(const float seconds) { _spoolup_time_hysteresis.set_hysteresis_time_from(false, seconds * 1_s); }
	void setTakeoffRampTime(const float seconds) { _takeoff_ramp_time = seconds; }

	void generateInitialRampValue(const float velocity_p_gain);

	void updateTakeoffState(const bool armed, const bool landed, const bool want_takeoff,
				const float takeoff_desired_vz, const bool skip_takeoff, const hrt_abstime &now_us);

	float updateRamp(const float dt, const float takeoff_desired_vz);

	TakeoffState getTakeoffState() const { return _takeoff_state; }

private:
	TakeoffState _takeoff_state = TakeoffState::disarmed;
	systemlib::Hysteresis _spoolup_time_hysteresis{false};
	float _takeoff_ramp_time{0.f};
	float _takeoff_ramp_vz_init{0.f};
	float _takeoff_ramp_progress{0.f};
};
