#include "pregme_att_control.hpp"

#include <circuit_breaker/circuit_breaker.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <mathlib/math/Functions.hpp>
#include <mathlib/math/Limits.hpp>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

using namespace matrix;

ModuleBase::Descriptor UserAttitudeControl::desc{task_spawn, custom_command, print_usage};

// The system is configured for multicopter operation by default, with all VTOL-related logic removed.
UserAttitudeControl::UserAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	_vehicle_status.is_vtol = false;
	_vehicle_status.in_transition_mode = false;
	_vehicle_type_rotary_wing = true;
	_vtol_in_transition_mode = false;
	_vtol_tailsitter = false;
	_spooled_up = false;

	parameters_updated();
}

UserAttitudeControl::~UserAttitudeControl()
{
	perf_free(_loop_perf);
}

bool UserAttitudeControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed ！");
		return false;
	}

	return true;
}

void UserAttitudeControl::parameters_updated()
{
	_attitude_control.setControllerGain(
		Vector3f(_param_usr_lambda_q_x.get(), _param_usr_lambda_q_y.get(), _param_usr_lambda_q_z.get()),
		Vector3f(_param_usr_k_q_x.get(), _param_usr_k_q_y.get(), _param_usr_k_q_z.get()));

	_attitude_control.setCESOParas(
		Vector3f(_param_usr_eso_l_x.get(), _param_usr_eso_l_y.get(), _param_usr_eso_l_z.get()),
		_param_usr_eso_epsi.get(),
		_param_usr_eso_c1.get(),
		_param_usr_eso_c2.get());

	_attitude_control.setPresetTrajParas(
		_param_PresetTraj_l.get(),
		_param_PresetTraj_w.get(),
		_param_PresetTraj_epsilon.get(),
		_param_PresetTraj_k.get());


	float inertia_b[9] = {
		_param_usr_i_xx.get(), -_param_usr_i_xy.get(), -_param_usr_i_xz.get(),
		-_param_usr_i_xy.get(), _param_usr_i_yy.get(), -_param_usr_i_yz.get(),
		-_param_usr_i_xz.get(), -_param_usr_i_yz.get(), _param_usr_i_zz.get()
	};

	SquareMatrix<float, 3> inertia_matrix(inertia_b);
	_attitude_control.setInertiaMatrix(inertia_matrix);

	using math::radians;

	_attitude_control.setRateLimit(Vector3f(
		radians(_param_usr_rollrate_max.get()),
		radians(_param_usr_pitchrate_max.get()),
		radians(_param_usr_yawrate_max.get())));

	_man_tilt_max = math::radians(_param_usr_man_tilt_max.get());

	// Since the CBRK_RATE_CTRL_KEY parameter is not available in PX4 v1.17, a default constant value of 121212 is used here.
	_actuators_0_circuit_breaker_enabled = (_param_cbrk_rate_ctrl.get() == 121212);
}

// Adapt to the current PX4 manual_control_setpoint.throttle [-1, 1] input specification and implement throttle safety clamping.
float UserAttitudeControl::throttle_curve(float throttle_stick_input)
{
	// PX4 manual_control_setpoint.throttle is in range [-1, 1].
	const float stick = math::constrain(throttle_stick_input, -1.f, 1.f);

	const float throttle_min = _landed ? 0.f : _param_usr_manthr_min.get();

	float thrust = 0.f;

	switch (_param_usr_thr_curve.get()) {
	case 1:
		// Linear mapping without hover-throttle rescaling.
		thrust = math::interpolate(stick, -1.f, 1.f, throttle_min, _param_usr_thr_max.get());
		break;

	default:
		// Rescale to hover throttle at zero stick, matching the current PX4 manual-control convention.
		thrust = math::interpolateNXY(stick,
					       {-1.f, 0.f, 1.f},
					       {throttle_min, _param_usr_thr_hover.get(), _param_usr_thr_max.get()});
		break;
	}

	// Missing ETH throttle estimation logic.
	return math::constrain(thrust, 0.f, _param_usr_thr_max.get());
}

void UserAttitudeControl::generate_attitude_setpoint(const Quatf &q, float dt, bool reset_yaw_sp)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};

	const float yaw = Eulerf(q).psi();

	// Avoid continuous integral calculation when _man_yaw_sp is NaN or invalid.
	if (reset_yaw_sp || !PX4_ISFINITE(_man_yaw_sp)) {
		_man_yaw_sp = yaw;

	} else if (_manual_control_setpoint.throttle > -0.9f || _param_usr_airmode.get() == 2) {
		const float yaw_rate = math::radians(_param_usr_man_y_max.get());

		// Adapt to the new manual control setpoint field naming.
		attitude_setpoint.yaw_sp_move_rate = _manual_control_setpoint.yaw * yaw_rate;

		_man_yaw_sp = wrap_pi(_man_yaw_sp + attitude_setpoint.yaw_sp_move_rate * dt);
	}

	_man_x_input_filter.setParameters(dt, _param_usr_man_tilt_tau.get());
	_man_y_input_filter.setParameters(dt, _param_usr_man_tilt_tau.get());

	_man_x_input_filter.update(_manual_control_setpoint.roll * _man_tilt_max);
	_man_y_input_filter.update(_manual_control_setpoint.pitch * _man_tilt_max);

	const float x = _man_x_input_filter.getState();
	const float y = _man_y_input_filter.getState();

	Vector2f v(y, -x);
	const float v_norm = v.norm();

	if (v_norm > _man_tilt_max) {
		v *= _man_tilt_max / v_norm;
	}

	// Add const to improve code quality and variable safety.
	const Quatf q_sp_rp = AxisAnglef(v(0), v(1), 0.f);
	const Eulerf euler_sp = q_sp_rp;

	const float roll_sp = euler_sp(0);
	const float pitch_sp = euler_sp(1);
	// Add safety clamping.
	const float yaw_sp = wrap_pi(_man_yaw_sp + euler_sp(2));

	// Quaternions are free from gimbal lock and offer better numerical stability.
	const Quatf q_sp = Eulerf(roll_sp, pitch_sp, yaw_sp);
	q_sp.copyTo(attitude_setpoint.q_d);

	// Explicitly set thrust in X and Y directions to 0, and only keep thrust in Z direction.
	attitude_setpoint.thrust_body[0] = 0.f;
	attitude_setpoint.thrust_body[1] = 0.f;
	attitude_setpoint.thrust_body[2] = -throttle_curve(_manual_control_setpoint.throttle);
	attitude_setpoint.timestamp = hrt_absolute_time();

	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

	// Update variables in real-time without transmitting via uORB messages.
	_attitude_control.setAttitudeSetpoint(q_sp, attitude_setpoint.yaw_sp_move_rate);
	_thrust_setpoint_body = Vector3f(attitude_setpoint.thrust_body);
	_last_attitude_setpoint = attitude_setpoint.timestamp;
}

void UserAttitudeControl::update_vehicle_status()
{
	if (_vehicle_status_sub.updated()) {
		// Save drone state for easy retrieval / calling.
		vehicle_status_s vehicle_status{};

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_vehicle_status = vehicle_status;
			_vehicle_type_rotary_wing =
				(vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

			_vtol_in_transition_mode = vehicle_status.in_transition_mode;
			_vtol_tailsitter = vehicle_status.is_vtol_tailsitter;

			// Add safety protection logic for arming/unlocking.
			const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
			_spooled_up = armed && hrt_elapsed_time(&vehicle_status.armed_time) >
				_param_com_spoolup_time.get() * 1_s;
		}
	}
}

void UserAttitudeControl::update_landed_state()
{
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected{};

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
			_maybe_landed = vehicle_land_detected.maybe_landed;
		}
	}
}

void UserAttitudeControl::update_landing_gear()
{
	if (_landing_gear_sub.updated()) {
		landing_gear_s landing_gear{};

		if (_landing_gear_sub.copy(&landing_gear)
		    && landing_gear.landing_gear != landing_gear_s::GEAR_KEEP) {
			_landing_gear = landing_gear.landing_gear;
		}
	}
}

void UserAttitudeControl::publish_torque_thrust_setpoint(const vehicle_attitude_s &v_att)
{
	(void)v_att;

	if (_actuators_0_circuit_breaker_enabled) {
		return;
	}

	const float tau_coe = math::max(_param_usr_tau_coe.get(), 0.001f);

	Vector3f torque_normalized = _torque * (1.f / tau_coe);

	for (int i = 0; i < 3; i++) {
		torque_normalized(i) = PX4_ISFINITE(torque_normalized(i))
				       ? math::constrain(torque_normalized(i), -1.f, 1.f)
				       : 0.f;
	}

	_thrust_sp = -_thrust_setpoint_body(2);
	_thrust_sp = PX4_ISFINITE(_thrust_sp) ? math::constrain(_thrust_sp, 0.f, 1.f) : 0.f;

	if (!_vehicle_control_mode.flag_armed) {
		// Do not publish any effective torque or thrust when disarmed.
		torque_normalized.zero();
		_thrust_sp = 0.f;

	} else if (!_spooled_up) {
		// Match PX4 spoolup behavior: after arming, keep the controller output at zero until the configured motor spoolup time has elapsed.
		torque_normalized.zero();
		_thrust_sp = 0.f;

	} else if (_landed) {
		torque_normalized.zero();
	}

	if (_param_usr_bat_scale_en.get()) {
		battery_status_s battery_status{};

		if (_battery_status_sub.update(&battery_status)
		    && battery_status.connected
		    && battery_status.scale > 0.f) {
			_battery_status_scale = battery_status.scale;
		}

		if (_battery_status_scale > 0.f) {
			torque_normalized *= _battery_status_scale;
			_thrust_sp *= _battery_status_scale;
		}
	}

	const hrt_abstime now = hrt_absolute_time();

	vehicle_torque_setpoint_s torque_sp{};
	torque_sp.xyz[0] = torque_normalized(0);
	torque_sp.xyz[1] = torque_normalized(1);
	torque_sp.xyz[2] = torque_normalized(2);
	torque_sp.timestamp = now;

	vehicle_thrust_setpoint_s thrust_sp{};
	thrust_sp.xyz[0] = 0.f;
	thrust_sp.xyz[1] = 0.f;
	thrust_sp.xyz[2] = -_thrust_sp;
	thrust_sp.timestamp = now;

	_vehicle_torque_setpoint_pub.publish(torque_sp);
	_vehicle_thrust_setpoint_pub.publish(thrust_sp);
}

void UserAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update{};
		_parameter_update_sub.copy(&param_update);
		updateParams();
		parameters_updated();
	}

	vehicle_angular_velocity_s angular_velocity{};

	if (!_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		perf_end(_loop_perf);
		return;
	}

	float dt = 0.004f;

	if (_last_run != 0) {
		dt = math::constrain(
			(angular_velocity.timestamp_sample - _last_run) * 1e-6f,
			0.0002f,
			0.02f);
	}

	_last_run = angular_velocity.timestamp_sample;

	vehicle_attitude_s v_att{};

	if (!_vehicle_attitude_sub.update(&v_att)) {
		_vehicle_attitude_sub.copy(&v_att);
	}

	if (v_att.timestamp == 0) {
		perf_end(_loop_perf);
		return;
	}

	vehicle_local_position_s local_pos{};

	if (_local_pos_sub.update(&local_pos) && PX4_ISFINITE(local_pos.z)) {
		pos_z = local_pos.z;
	}

	_manual_control_setpoint_sub.update(&_manual_control_setpoint);
	_v_control_mode_sub.update(&_vehicle_control_mode);

	update_vehicle_status();
	update_landed_state();
	update_landing_gear();

	const bool is_hovering = _vehicle_type_rotary_wing && !_vtol_in_transition_mode;

	const bool run_att_ctrl =
		_vehicle_control_mode.flag_control_attitude_enabled && is_hovering;

	if (!run_att_ctrl) {
		_man_x_input_filter.reset(0.f);
		_man_y_input_filter.reset(0.f);
		_reset_yaw_sp = true;
		_thrust_setpoint_body.zero();
		_torque.zero();

		_attitude_control.resetESO();
		_attitude_control.resetPresetTraj();

		if (_vehicle_control_mode.flag_control_termination_enabled) {
			vehicle_torque_setpoint_s torque_sp{};
			vehicle_thrust_setpoint_s thrust_sp{};

			const hrt_abstime now = hrt_absolute_time();
			torque_sp.timestamp = now;
			thrust_sp.timestamp = now;

			_vehicle_torque_setpoint_pub.publish(torque_sp);
			_vehicle_thrust_setpoint_pub.publish(thrust_sp);
		}

		perf_end(_loop_perf);
		return;
	}

	if (!_vehicle_control_mode.flag_armed
	    || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		_attitude_control.resetESO();
		_attitude_control.resetPresetTraj();
	}

	const Quatf q{v_att.q};
	bool attitude_setpoint_generated = false;

	const bool manual_stabilized =
		_vehicle_control_mode.flag_control_manual_enabled
		&& !_vehicle_control_mode.flag_control_altitude_enabled
		&& !_vehicle_control_mode.flag_control_velocity_enabled
		&& !_vehicle_control_mode.flag_control_position_enabled;

	if (manual_stabilized) {
		generate_attitude_setpoint(q, dt, _reset_yaw_sp);
		attitude_setpoint_generated = true;

	} else {
		_man_x_input_filter.reset(0.f);
		_man_y_input_filter.reset(0.f);

		if (_vehicle_attitude_setpoint_sub.updated()) {
			vehicle_attitude_setpoint_s vehicle_attitude_setpoint{};

			if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)
			    && vehicle_attitude_setpoint.timestamp > _last_attitude_setpoint) {

				_attitude_control.setAttitudeSetpoint(
					Quatf(vehicle_attitude_setpoint.q_d),
					vehicle_attitude_setpoint.yaw_sp_move_rate);

				_thrust_setpoint_body = Vector3f(vehicle_attitude_setpoint.thrust_body);
				_last_attitude_setpoint = vehicle_attitude_setpoint.timestamp;
			}
		}
	}

	if (_quat_reset_counter != v_att.quat_reset_counter) {
		const Quatf delta_q_reset(v_att.delta_q_reset);
		const float delta_psi = Eulerf(delta_q_reset).psi();

		if (PX4_ISFINITE(_man_yaw_sp)) {
			_man_yaw_sp = wrap_pi(_man_yaw_sp + delta_psi);
		}

		if (v_att.timestamp > _last_attitude_setpoint) {
			_attitude_control.adaptAttitudeSetpoint(delta_q_reset);
		}

		_quat_reset_counter = v_att.quat_reset_counter;
	}

	const Vector3f rates{angular_velocity.xyz};
	Vector3f rates_sp{0.f, 0.f, 0.f};

	_attitude_control.update(q, rates, dt, _landed, _torque, rates_sp, pos_z);

	rate_ctrl_status_s rate_ctrl_status{};
	_attitude_control.getRateControlStatus(rate_ctrl_status);
	rate_ctrl_status.timestamp = hrt_absolute_time();
	_controller_status_pub.publish(rate_ctrl_status);

	publish_torque_thrust_setpoint(v_att);

	_reset_yaw_sp = !attitude_setpoint_generated || _landed;

	perf_end(_loop_perf);
}

int UserAttitudeControl::task_spawn(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	UserAttitudeControl *instance = new UserAttitudeControl();

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

int UserAttitudeControl::print_status()
{
	perf_print_counter(_loop_perf);
	return 0;
}

int UserAttitudeControl::custom_command(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	return print_usage("unknown command");
}

int UserAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Custom multicopter attitude controller for PX4 Control Allocation architecture.

This module runs on vehicle_angular_velocity updates, generates manual stabilized attitude setpoints when required,
uses the custom Att_Control backend, and publishes vehicle_torque_setpoint and vehicle_thrust_setpoint.

This version is only for multicopter platforms. VTOL and legacy actuator_controls output logic have been removed.

Important: this module publishes vehicle_torque_setpoint and vehicle_thrust_setpoint directly.
Do not run it together with the stock mc_rate_control module.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("usr_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int pregme_att_control_main(int argc, char *argv[])
{
	return ModuleBase::main(UserAttitudeControl::desc, argc, argv);
}
