#include "PregmePositionControl.hpp"
#include "ControlMath.hpp"

#include <cmath>
#include <cstring>
#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/events.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/vehicle_command.h>

using namespace matrix;

namespace
{
uORB::Publication<vehicle_command_s> g_vehicle_command_pub{ORB_ID(vehicle_command)};
hrt_abstime g_touchdown_start_us{0};
hrt_abstime g_last_disarm_request_us{0};
bool g_seen_real_flight_after_arm{false};

void reset_post_landing_disarm_state()
{
	g_touchdown_start_us = 0;
	g_last_disarm_request_us = 0;
	g_seen_real_flight_after_arm = false;
}

void request_disarm_after_stable_touchdown(bool post_flight_touchdown,
		const PositionControlStates &states, hrt_abstime now_us)
{
	if (!post_flight_touchdown) {
		g_touchdown_start_us = 0;
		return;
	}

	if (g_touchdown_start_us == 0) {
		g_touchdown_start_us = now_us;
		return;
	}

	const bool touchdown_confirmed_long_enough = (now_us - g_touchdown_start_us) > 1200_ms;
	const bool vertical_speed_low = !PX4_ISFINITE(states.velocity(2)) || fabsf(states.velocity(2)) < 0.35f;
	const bool request_rate_limited = (g_last_disarm_request_us == 0) || ((now_us - g_last_disarm_request_us) > 2000_ms);

	if (!touchdown_confirmed_long_enough || !vertical_speed_low || !request_rate_limited) {
		return;
	}

	vehicle_command_s cmd{};
	cmd.timestamp = now_us;
	cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
	cmd.param1 = 0.f; // disarm
	cmd.param2 = 0.f; // do not force-disarm in the air
	cmd.target_system = 1;
	cmd.target_component = 1;
	cmd.source_system = 1;
	cmd.source_component = 1;
	cmd.from_external = false;

	g_vehicle_command_pub.publish(cmd);
	g_last_disarm_request_us = now_us;
}

trajectory_setpoint_s generate_hold_setpoint(const PositionControlStates &states)
{
	trajectory_setpoint_s setpoint{};
	setpoint.timestamp = hrt_absolute_time();

	setpoint.position[0] = NAN;
	setpoint.position[1] = NAN;
	setpoint.position[2] = NAN;
	setpoint.velocity[0] = NAN;
	setpoint.velocity[1] = NAN;
	setpoint.velocity[2] = NAN;
	setpoint.acceleration[0] = NAN;
	setpoint.acceleration[1] = NAN;
	setpoint.acceleration[2] = NAN;
	setpoint.yaw = NAN;
	setpoint.yawspeed = NAN;

	if (PX4_ISFINITE(states.position(0))) {
		setpoint.position[0] = states.position(0);
	}

	if (PX4_ISFINITE(states.position(1))) {
		setpoint.position[1] = states.position(1);
	}

	if (PX4_ISFINITE(states.position(2))) {
		setpoint.position[2] = states.position(2);
	}

	if (PX4_ISFINITE(states.yaw)) {
		setpoint.yaw = states.yaw;
	}

	setpoint.yawspeed = 0.f;
	return setpoint;
}

} // namespace

ModuleBase::Descriptor PregmePositionControl::desc{task_spawn, custom_command, print_usage};

PregmePositionControl::PregmePositionControl(bool vtol) :
	SuperBlock(nullptr, MODULE_NAME),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint))
{
	parameters_update(true);
	_failsafe_land_hysteresis.set_hysteresis_time_from(false, LOITER_TIME_BEFORE_DESCEND);
	_tilt_limit_slew_rate.setSlewRate(.2f);
	reset_setpoint_to_nan(_setpoint);
}

PregmePositionControl::~PregmePositionControl()
{
	perf_free(_cycle_perf);
}

bool PregmePositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("vehicle_local_position callback registration failed!");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();
	return true;
}

int PregmePositionControl::parameters_update(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s pupdate{};
		_parameter_update_sub.copy(&pupdate);

		ModuleParams::updateParams();
		SuperBlock::updateParams();

		_vel_x_deriv.setCutoff(_param_pregme_veld_lp.get());
		_vel_y_deriv.setCutoff(_param_pregme_veld_lp.get());
		_vel_z_deriv.setCutoff(_param_pregme_veld_lp.get());

		int num_changed = 0;

		if (_param_pregme_vehicle_resp.get() >= 0.f) {
			const float responsiveness = _param_pregme_vehicle_resp.get() * _param_pregme_vehicle_resp.get();
			num_changed += _param_pregme_acc_hor.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_pregme_acc_hor_max.commit_no_notification(math::lerp(2.f, 15.f, responsiveness));
			num_changed += _param_pregme_man_y_max.commit_no_notification(math::lerp(80.f, 450.f, responsiveness));

			if (responsiveness > 0.6f) {
				num_changed += _param_pregme_man_y_tau.commit_no_notification(0.f);
			} else {
				num_changed += _param_pregme_man_y_tau.commit_no_notification(math::lerp(0.5f, 0.f, responsiveness / 0.6f));
			}

			if (responsiveness < 0.5f) {
				num_changed += _param_pregme_tiltmax_air.commit_no_notification(45.f);
			} else {
				num_changed += _param_pregme_tiltmax_air.commit_no_notification(math::min(MAX_SAFE_TILT_DEG, math::lerp(45.f, 70.f,
					(responsiveness - 0.5f) * 2.f)));
			}

			num_changed += _param_pregme_acc_down_max.commit_no_notification(math::lerp(0.8f, 15.f, responsiveness));
			num_changed += _param_pregme_acc_up_max.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_pregme_jerk_max.commit_no_notification(math::lerp(2.f, 50.f, responsiveness));
			num_changed += _param_pregme_jerk_auto.commit_no_notification(math::lerp(1.f, 25.f, responsiveness));
		}

		if (_param_pregme_xy_vel_all.get() >= 0.f) {
			const float xy_vel = _param_pregme_xy_vel_all.get();

			num_changed += _param_pregme_vel_manual.commit_no_notification(xy_vel);
			num_changed += _param_pregme_xy_cruise.commit_no_notification(xy_vel);
			num_changed += _param_pregme_xy_vel_max.commit_no_notification(xy_vel);

			_param_pregme_xy_vel_all.set(-1.f);
			num_changed += _param_pregme_xy_vel_all.commit_no_notification();
		}

		if (_param_pregme_z_vel_all.get() >= 0.f) {
			const float z_vel = _param_pregme_z_vel_all.get();

			num_changed += _param_pregme_z_vel_max_up.commit_no_notification(z_vel);
			num_changed += _param_pregme_z_vel_max_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_pregme_tko_speed.commit_no_notification(z_vel * 0.6f);
			num_changed += _param_pregme_land_speed.commit_no_notification(z_vel * 0.5f);

			_param_pregme_z_vel_all.set(-1.f);
			num_changed += _param_pregme_z_vel_all.commit_no_notification();
		}

		if (num_changed > 0) {
			param_notify_changes();
		}

		if (_param_pregme_tiltmax_air.get() > MAX_SAFE_TILT_DEG) {
			_param_pregme_tiltmax_air.set(MAX_SAFE_TILT_DEG);
			_param_pregme_tiltmax_air.commit();
			mavlink_log_critical(nullptr, "Tilt constrained to safe value");
		}

		if (_param_pregme_tiltmax_lnd.get() > _param_pregme_tiltmax_air.get()) {
			_param_pregme_tiltmax_lnd.set(_param_pregme_tiltmax_air.get());
			_param_pregme_tiltmax_lnd.commit();
			mavlink_log_critical(nullptr, "Land tilt has been constrained by max tilt");
		}

		_control.setControlParas(Vector3f(_param_pregme_lambda_p_x.get(), _param_pregme_lambda_p_y.get(), _param_pregme_lambda_p_z.get()),
					 Vector3f(_param_pregme_k_p_x.get(), _param_pregme_k_p_y.get(), _param_pregme_k_p_z.get()));
		_control.setCESOParas(Vector3f(_param_eso_v_l1_x.get(), _param_eso_v_l1_y.get(), _param_eso_v_l1_z.get()),
				      Vector3f(_param_eso_v_l2_x.get(), _param_eso_v_l2_y.get(), _param_eso_v_l2_z.get()),
				      _param_eso_v_epsi.get(), _param_eso_v_c1.get(), _param_eso_v_c2.get());
		_control.setPresetTrajParas(_param_PresetTraj_l.get(), _param_PresetTraj_w.get(),
					    _param_PresetTraj_epsilon.get(), _param_PresetTraj_k.get());

		if (_param_pregme_xy_cruise.get() > _param_pregme_xy_vel_max.get()) {
			_param_pregme_xy_cruise.set(_param_pregme_xy_vel_max.get());
			_param_pregme_xy_cruise.commit();
			mavlink_log_critical(nullptr, "Cruise speed has been constrained by max speed");
		}

		if (_param_pregme_vel_manual.get() > _param_pregme_xy_vel_max.get()) {
			_param_pregme_vel_manual.set(_param_pregme_xy_vel_max.get());
			_param_pregme_vel_manual.commit();
			mavlink_log_critical(nullptr, "Manual speed has been constrained by max speed");
		}

		if (_param_pregme_thr_hover.get() > _param_pregme_thr_max.get() ||
		    _param_pregme_thr_hover.get() < _param_pregme_thr_min.get()) {
			_param_pregme_thr_hover.set(math::constrain(_param_pregme_thr_hover.get(), _param_pregme_thr_min.get(), _param_pregme_thr_max.get()));
			_param_pregme_thr_hover.commit();
			mavlink_log_critical(nullptr, "Hover thrust has been constrained by min/max");
		}

		if (!_param_pregme_use_hte.get() || !_hover_thrust_initialized) {
			_control.setHoverThrust(_param_pregme_thr_hover.get());
			_hover_thrust_initialized = true;
		}

		_param_pregme_tko_speed.set(math::min(_param_pregme_tko_speed.get(), _param_pregme_z_vel_max_up.get()));
		_param_pregme_land_speed.set(math::min(_param_pregme_land_speed.get(), _param_pregme_z_vel_max_dn.get()));

		_takeoff.setSpoolupTime(_param_pregme_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_pregme_tko_ramp_t.get());
		_takeoff.generateInitialRampValue(_param_pregme_k_p_z.get());
	}

	return OK;
}

PositionControlStates PregmePositionControl::set_vehicle_states(const vehicle_local_position_s &local_pos, float dt)
{
	PositionControlStates states{};

	if (PX4_ISFINITE(local_pos.x) && PX4_ISFINITE(local_pos.y) && local_pos.xy_valid) {
		states.position(0) = local_pos.x;
		states.position(1) = local_pos.y;
	} else {
		states.position(0) = NAN;
		states.position(1) = NAN;
	}

	if (PX4_ISFINITE(local_pos.z) && local_pos.z_valid) {
		states.position(2) = local_pos.z;
	} else {
		states.position(2) = NAN;
	}

	if (PX4_ISFINITE(local_pos.vx) && PX4_ISFINITE(local_pos.vy) && local_pos.v_xy_valid) {
		states.velocity(0) = local_pos.vx;
		states.velocity(1) = local_pos.vy;
		states.acceleration(0) = _vel_x_deriv.update(local_pos.vx, dt);
		states.acceleration(1) = _vel_y_deriv.update(local_pos.vy, dt);
	} else {
		states.velocity(0) = NAN;
		states.velocity(1) = NAN;
		states.acceleration(0) = NAN;
		states.acceleration(1) = NAN;
		_vel_x_deriv.reset();
		_vel_y_deriv.reset();
	}

	if (PX4_ISFINITE(local_pos.vz) && local_pos.v_z_valid) {
		states.velocity(2) = local_pos.vz;
		states.acceleration(2) = _vel_z_deriv.update(states.velocity(2), dt);
	} else {
		states.velocity(2) = NAN;
		states.acceleration(2) = NAN;
		_vel_z_deriv.reset();
	}

	states.yaw = local_pos.heading;
	return states;
}

void PregmePositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	ScheduleDelayed(100_ms);
	parameters_update(false);

	perf_begin(_cycle_perf);

	vehicle_local_position_s local_pos{};

	if (_local_pos_sub.update(&local_pos)) {
		const hrt_abstime time_stamp_now = local_pos.timestamp;
		const float dt = math::constrain(((time_stamp_now - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = time_stamp_now;
		setDt(dt);

		const bool was_in_failsafe = _in_failsafe;
		_in_failsafe = false;

		_vehicle_control_mode_sub.update(&_vehicle_control_mode);
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);

		if (!_vehicle_control_mode.flag_armed) {
			reset_post_landing_disarm_state();
		}

		PositionControlStates states{set_vehicle_states(local_pos, dt)};

		if (_vehicle_control_mode.flag_multicopter_position_control_enabled) {
			_trajectory_setpoint_sub.update(&_setpoint);

			if (_setpoint.timestamp == 0) {
				_setpoint = generate_hold_setpoint(states);

			} else if (hrt_elapsed_time(&_setpoint.timestamp) > 1_s && !_vehicle_control_mode.flag_armed) {
				_setpoint = generate_hold_setpoint(states);
			}

			if (_setpoint.timestamp < local_pos.timestamp) {
				if (local_pos.vxy_reset_counter != _vxy_reset_counter) {
					_setpoint.velocity[0] += local_pos.delta_vxy[0];
					_setpoint.velocity[1] += local_pos.delta_vxy[1];
				}

				if (local_pos.vz_reset_counter != _vz_reset_counter) {
					_setpoint.velocity[2] += local_pos.delta_vz;
				}

				if (local_pos.xy_reset_counter != _xy_reset_counter) {
					_setpoint.position[0] += local_pos.delta_xy[0];
					_setpoint.position[1] += local_pos.delta_xy[1];
				}

				if (local_pos.z_reset_counter != _z_reset_counter) {
					_setpoint.position[2] += local_pos.delta_z;
				}

				if (local_pos.heading_reset_counter != _heading_reset_counter) {
					_setpoint.yaw += local_pos.delta_heading;
				}
			}

			_vehicle_constraints_sub.update(&_vehicle_constraints);

			if (!PX4_ISFINITE(_vehicle_constraints.speed_up) || (_vehicle_constraints.speed_up > _param_pregme_z_vel_max_up.get())) {
				_vehicle_constraints.speed_up = _param_pregme_z_vel_max_up.get();
			}

			const bool has_recent_setpoint = (_setpoint.timestamp != 0) && (hrt_elapsed_time(&_setpoint.timestamp) < 1_s);

			const bool setpoint_requests_descent = has_recent_setpoint
				&& ((PX4_ISFINITE(_setpoint.velocity[2]) && (_setpoint.velocity[2] > 0.05f))
				    || (PX4_ISFINITE(_setpoint.acceleration[2]) && (_setpoint.acceleration[2] > 0.1f)));

			const bool setpoint_requests_takeoff = has_recent_setpoint
				&& !setpoint_requests_descent
				&& PX4_ISFINITE(states.position(2))
				&& ((PX4_ISFINITE(_setpoint.position[2]) && (_setpoint.position[2] < states.position(2) - 0.03f))
				    || (PX4_ISFINITE(_setpoint.velocity[2]) && (_setpoint.velocity[2] < -0.05f))
				    || (PX4_ISFINITE(_setpoint.acceleration[2]) && (_setpoint.acceleration[2] < -0.1f)));

			const bool land_detector_contact = _vehicle_land_detected.landed
				|| _vehicle_land_detected.ground_contact
				|| _vehicle_land_detected.maybe_landed;

			const bool was_flying = (_takeoff.getTakeoffState() >= TakeoffState::flight);


			const bool landing_contact_after_flight = _vehicle_control_mode.flag_armed
				&& was_flying
				&& land_detector_contact;

			const bool landing_contact_without_takeoff = landing_contact_after_flight;

			if (_vehicle_control_mode.flag_armed && (_vehicle_land_detected.landed || landing_contact_without_takeoff)) {
				if (landing_contact_without_takeoff) {
					_vehicle_constraints.want_takeoff = false;
					reset_setpoint_to_nan(_setpoint);
					_setpoint.timestamp = local_pos.timestamp;
					_setpoint.acceleration[2] = 100.f;

				} else {
					_vehicle_constraints.want_takeoff = _vehicle_constraints.want_takeoff || setpoint_requests_takeoff;
				}

				const float takeoff_speed = math::min(_param_pregme_tko_speed.get(), _param_pregme_z_vel_max_up.get());
				_vehicle_constraints.speed_up = PX4_ISFINITE(takeoff_speed) ? takeoff_speed : _param_pregme_z_vel_max_up.get();
				_vehicle_constraints.speed_down = _param_pregme_z_vel_max_dn.get();
			}

			const bool takeoff_state_landed = _vehicle_land_detected.landed || landing_contact_without_takeoff;

			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, takeoff_state_landed,
						    _vehicle_constraints.want_takeoff, _vehicle_constraints.speed_up,
						    false, time_stamp_now);

			const bool not_taken_off = (_takeoff.getTakeoffState() < TakeoffState::rampup);
			const bool flying = (_takeoff.getTakeoffState() >= TakeoffState::flight);
			const bool flying_but_landing_contact = flying && landing_contact_without_takeoff;

			if (_vehicle_control_mode.flag_armed && flying && !land_detector_contact) {
				g_seen_real_flight_after_arm = true;
			}

			const bool post_flight_touchdown_for_disarm = landing_contact_after_flight
				|| (_vehicle_control_mode.flag_armed
				    && g_seen_real_flight_after_arm
				    && land_detector_contact);

			request_disarm_after_stable_touchdown(post_flight_touchdown_for_disarm, states, time_stamp_now);

			if (!flying) {
				// Keep takeoff deterministic: do not let hover-thrust estimator alter the thrust scale before flight.
				_control.setHoverThrust(_param_pregme_thr_hover.get());

			} else if (_param_pregme_use_hte.get()
				   && !_vehicle_land_detected.landed
				   && !_vehicle_land_detected.ground_contact) {
				hover_thrust_estimate_s hte{};

				if (_hover_thrust_estimate_sub.update(&hte) && hte.valid) {
					_control.updateHoverThrust(hte.hover_thrust);
				}
			}

			if ((_takeoff.getTakeoffState() == TakeoffState::rampup) && PX4_ISFINITE(_setpoint.velocity[2])) {
				_setpoint.acceleration[2] = NAN;
			}

			if (not_taken_off || flying_but_landing_contact) {
				reset_setpoint_to_nan(_setpoint);
				_setpoint.timestamp = local_pos.timestamp;
				_setpoint.acceleration[2] = 100.f;
				_control.resetIntegral();
				_control.resetESO();
				_control.resetPresetTraj();
			}

			const float tilt_limit_deg = (_takeoff.getTakeoffState() < TakeoffState::flight)
						     ? _param_pregme_tiltmax_lnd.get() : _param_pregme_tiltmax_air.get();
			_control.setTiltLimit(_tilt_limit_slew_rate.update(math::radians(tilt_limit_deg), dt));

			const float speed_up = _takeoff.updateRamp(dt,
				PX4_ISFINITE(_vehicle_constraints.speed_up) ? _vehicle_constraints.speed_up : _param_pregme_z_vel_max_up.get());
			const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ? _vehicle_constraints.speed_down : _param_pregme_z_vel_max_dn.get();
			const float speed_horizontal = _param_pregme_xy_vel_max.get();

			const float minimum_thrust = (flying && !flying_but_landing_contact) ? _param_pregme_thr_min.get() : 0.f;

			_control.setThrustLimits(minimum_thrust, _param_pregme_thr_max.get());
			_control.setVelocityLimits(math::constrain(speed_horizontal, 0.f, _param_pregme_xy_vel_max.get()),
						   math::min(speed_up, _param_pregme_z_vel_max_up.get()),
						   math::constrain(speed_down, 0.f, _param_pregme_z_vel_max_dn.get()));

			if (_vehicle_control_mode.flag_armed
			    && _vehicle_constraints.want_takeoff
			    && (_takeoff.getTakeoffState() >= TakeoffState::rampup)
			    && _vehicle_land_detected.landed
			    && PX4_ISFINITE(states.position(2))) {
				const float takeoff_speed_sp = math::max(speed_up, TAKEOFF_SPEED_SP_MIN);

				if (!PX4_ISFINITE(_setpoint.velocity[2]) || (_setpoint.velocity[2] > -0.01f)) {
					_setpoint.velocity[2] = -takeoff_speed_sp;
				}

				const float nominal_takeoff_speed = math::max(math::min(_param_pregme_tko_speed.get(),
									     _param_pregme_z_vel_max_up.get()), 0.1f);
				const float ramp_ratio = math::constrain(speed_up / nominal_takeoff_speed, 0.f, 1.f);
				const float altitude_step = math::max(TAKEOFF_ALTITUDE_STEP_MIN, ALTITUDE_THRESHOLD * ramp_ratio);

				if (!PX4_ISFINITE(_setpoint.position[2]) || (_setpoint.position[2] > states.position(2) - TAKEOFF_ALTITUDE_STEP_MIN)) {
					_setpoint.position[2] = states.position(2) - altitude_step;
				}
			}

			_control.setInputSetpoint(_setpoint);

			if (!PX4_ISFINITE(_setpoint.position[2])
			    && PX4_ISFINITE(_setpoint.velocity[2]) && (fabsf(_setpoint.velocity[2]) > FLT_EPSILON)
			    && PX4_ISFINITE(local_pos.z_deriv) && local_pos.z_valid && local_pos.v_z_valid) {
				const float weighting = fminf(fabsf(_setpoint.velocity[2]) / _param_pregme_land_speed.get(), 1.f);
				states.velocity(2) = local_pos.z_deriv * weighting + local_pos.vz * (1.f - weighting);
			}

			_control.setState(states);

			if (_control.update(dt)) {
				_failsafe_land_hysteresis.set_state_and_update(false, time_stamp_now);

			} else {
				if ((time_stamp_now - _last_warn) > 2_s) {
					PX4_WARN("invalid setpoints");
					_last_warn = time_stamp_now;
				}

				vehicle_local_position_setpoint_s failsafe_setpoint{};
				failsafe(time_stamp_now, failsafe_setpoint, states, !was_in_failsafe);

				_vehicle_constraints.timestamp = 0;
				_vehicle_constraints.speed_up = NAN;
				_vehicle_constraints.speed_down = NAN;
				_vehicle_constraints.want_takeoff = false;

				_control.setInputSetpoint(failsafe_setpoint);
				_control.setVelocityLimits(_param_pregme_xy_vel_max.get(), _param_pregme_z_vel_max_up.get(), _param_pregme_z_vel_max_dn.get());
				_control.update(dt);
			}

			vehicle_local_position_setpoint_s local_pos_sp{};
			_control.getLocalPositionSetpoint(local_pos_sp);
			local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp_pub.publish(local_pos_sp);

			vehicle_attitude_setpoint_s attitude_setpoint{};
			_control.getAttitudeSetpoint(attitude_setpoint);

			limit_thrust_during_landing(attitude_setpoint, _takeoff.getTakeoffState(), dt, landing_contact_after_flight);

			attitude_setpoint.timestamp = hrt_absolute_time();
			_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

		} else {
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed, false, 10.f, true,
						    time_stamp_now);
		}

		const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());

		if (takeoff_state != _takeoff_status_pub.get().takeoff_state
		    || !isEqualF(_tilt_limit_slew_rate.getState(), _takeoff_status_pub.get().tilt_limit)) {
			_takeoff_status_pub.get().takeoff_state = takeoff_state;
			_takeoff_status_pub.get().tilt_limit = _tilt_limit_slew_rate.getState();
			_takeoff_status_pub.get().timestamp = hrt_absolute_time();
			_takeoff_status_pub.update();
		}

		_vxy_reset_counter = local_pos.vxy_reset_counter;
		_vz_reset_counter = local_pos.vz_reset_counter;
		_xy_reset_counter = local_pos.xy_reset_counter;
		_z_reset_counter = local_pos.z_reset_counter;
		_heading_reset_counter = local_pos.heading_reset_counter;
	}

	perf_end(_cycle_perf);
}

void PregmePositionControl::failsafe(const hrt_abstime &now, vehicle_local_position_setpoint_s &setpoint,
				     const PositionControlStates &states, bool warn)
{
	if (!_vehicle_control_mode.flag_armed) {
		warn = false;
	}

	_failsafe_land_hysteresis.set_state_and_update(true, now);

	if (_failsafe_land_hysteresis.get_state()) {
		reset_setpoint_to_nan(setpoint);

		if (PX4_ISFINITE(states.velocity(0)) && PX4_ISFINITE(states.velocity(1))) {
			setpoint.vx = setpoint.vy = 0.f;
			if (warn) {
				PX4_WARN("Failsafe: stop and wait");
			}

		} else {
			setpoint.acceleration[0] = setpoint.acceleration[1] = 0.f;
			setpoint.vz = _param_pregme_land_speed.get();
			if (warn) {
				PX4_WARN("Failsafe: blind land");
			}
		}

		if (PX4_ISFINITE(states.velocity(2))) {
			if (!PX4_ISFINITE(setpoint.vz)) {
				setpoint.vz = 0.f;
			}

		} else {
			setpoint.vz = NAN;
			setpoint.acceleration[2] = .3f;
			if (warn) {
				PX4_WARN("Failsafe: blind descend");
			}
		}

		_in_failsafe = true;
	}
}

void PregmePositionControl::reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint)
{
	setpoint.x = setpoint.y = setpoint.z = NAN;
	setpoint.vx = setpoint.vy = setpoint.vz = NAN;
	setpoint.yaw = setpoint.yawspeed = NAN;
	setpoint.acceleration[0] = setpoint.acceleration[1] = setpoint.acceleration[2] = NAN;
	setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = NAN;
}


void PregmePositionControl::reset_setpoint_to_nan(trajectory_setpoint_s &setpoint)
{
	setpoint.position[0] = setpoint.position[1] = setpoint.position[2] = NAN;
	setpoint.velocity[0] = setpoint.velocity[1] = setpoint.velocity[2] = NAN;
	setpoint.acceleration[0] = setpoint.acceleration[1] = setpoint.acceleration[2] = NAN;
	setpoint.yaw = NAN;
	setpoint.yawspeed = NAN;
}
float PregmePositionControl::slew_thrust_z(float target_thrust_z, float dt, float slew_rate)
{
	if (!PX4_ISFINITE(target_thrust_z)) {
		target_thrust_z = 0.f;
	}

	if (!PX4_ISFINITE(dt) || dt <= 0.f) {
		dt = 0.01f;
	}

	if (!PX4_ISFINITE(slew_rate) || slew_rate <= 0.f) {
		slew_rate = THRUST_SLEW_FLIGHT;
	}

	if (!_last_thrust_body_z_valid) {
		_last_thrust_body_z = target_thrust_z;
		_last_thrust_body_z_valid = true;
	}

	const float max_delta = math::max(slew_rate * dt, 0.f);
	target_thrust_z = math::constrain(target_thrust_z,
					    _last_thrust_body_z - max_delta,
					    _last_thrust_body_z + max_delta);

	_last_thrust_body_z = target_thrust_z;
	return target_thrust_z;
}

void PregmePositionControl::limit_thrust_during_landing(vehicle_attitude_setpoint_s &setpoint,
		const TakeoffState takeoff_state, float dt, bool landing_contact_after_flight)
{
	const bool armed = _vehicle_control_mode.flag_armed;
	const bool active_takeoff = _vehicle_constraints.want_takeoff && (takeoff_state >= TakeoffState::rampup);
	const bool land_detector_contact = _vehicle_land_detected.landed
		|| _vehicle_land_detected.ground_contact
		|| _vehicle_land_detected.maybe_landed;

	const bool ground_without_takeoff_request = landing_contact_after_flight
		|| (!_vehicle_constraints.want_takeoff
		    && land_detector_contact
		    && ((takeoff_state < TakeoffState::rampup) || (takeoff_state >= TakeoffState::flight)));

	if (!armed) {
		setpoint.thrust_body[0] = 0.f;
		setpoint.thrust_body[1] = 0.f;
		setpoint.thrust_body[2] = 0.f;
		_last_thrust_body_z = 0.f;
		_last_thrust_body_z_valid = false;
		return;
	}

	float target_thrust_z = PX4_ISFINITE(setpoint.thrust_body[2]) ? setpoint.thrust_body[2] : 0.f;
	target_thrust_z = math::constrain(target_thrust_z, -_param_pregme_thr_max.get(), 0.f);

	float slew_rate = THRUST_SLEW_FLIGHT;

	if (ground_without_takeoff_request) {
		target_thrust_z = 0.f;
		setpoint.thrust_body[0] = 0.f;
		setpoint.thrust_body[1] = 0.f;
		slew_rate = THRUST_SLEW_FLIGHT;

	} else if (active_takeoff || (takeoff_state < TakeoffState::flight)) {
		slew_rate = THRUST_SLEW_TAKEOFF;
	}

	if (!_last_thrust_body_z_valid) {
		_last_thrust_body_z = (takeoff_state < TakeoffState::rampup || ground_without_takeoff_request)
					    ? 0.f : target_thrust_z;
		_last_thrust_body_z_valid = true;
	}

	setpoint.thrust_body[2] = slew_thrust_z(target_thrust_z, dt, slew_rate);

	if (ground_without_takeoff_request && fabsf(setpoint.thrust_body[2]) < THRUST_ZERO_EPS) {
		setpoint.thrust_body[0] = 0.f;
		setpoint.thrust_body[1] = 0.f;
		setpoint.thrust_body[2] = 0.f;
		_last_thrust_body_z = 0.f;
	}
}

int PregmePositionControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1 && strcmp(argv[1], "vtol") == 0) {
		vtol = true;
	}

	auto *instance = new PregmePositionControl(vtol);

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int PregmePositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PregmePositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Custom multicopter position controller migrated to a PX4 v1.17-style architecture.

The outer module is responsible for subscriptions, parameter updates, takeoff state,
failsafe logic, EKF reset compensation and publication. The inner controller only
computes trajectory corrections, thrust and attitude setpoints.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pregme_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int pregme_pos_control_v2_main(int argc, char *argv[])
{
	return ModuleBase::main(PregmePositionControl::desc, argc, argv);
}
