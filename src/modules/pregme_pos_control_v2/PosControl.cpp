#include "PosControl.hpp"

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

using namespace matrix;
using namespace time_literals;

namespace
{

// Vertical CESO injection protection for OFFBOARD altitude hold.
// The ESO still estimates Z disturbance, but the value injected into the
// thrust channel is bounded and low-pass filtered to avoid thrust pumping.
static constexpr float CESO_Z_DISTURBANCE_LIMIT = 2.0f;      // m/s^2 equivalent, about 0.2 g
static constexpr float CESO_Z_LPF_CUTOFF_HZ = 1.5f;          // conservative vertical disturbance bandwidth
static constexpr float CESO_Z_DEADZONE = 0.03f;              // suppress tiny estimator noise around hover

float g_delta_v_z_filtered = 0.f;
bool g_delta_v_z_filter_initialized = false;

float updateVerticalCESOInjection(float delta_v_z_raw, bool vertical_control_enabled, float dt)
{
	if (!vertical_control_enabled || !PX4_ISFINITE(delta_v_z_raw)) {
		g_delta_v_z_filtered = 0.f;
		g_delta_v_z_filter_initialized = false;
		return 0.f;
	}

	dt = (PX4_ISFINITE(dt) && dt > 0.f) ? math::constrain(dt, 0.001f, 0.04f) : 0.01f;

	const float delta_v_z_limited = math::constrain(delta_v_z_raw,
		-CESO_Z_DISTURBANCE_LIMIT, CESO_Z_DISTURBANCE_LIMIT);

	if (!g_delta_v_z_filter_initialized) {
		// Start from zero after reset/takeoff transition and let the estimate fade in.
		g_delta_v_z_filtered = 0.f;
		g_delta_v_z_filter_initialized = true;
	}

	const float alpha = expf(-6.28318530718f * CESO_Z_LPF_CUTOFF_HZ * dt);
	g_delta_v_z_filtered = alpha * g_delta_v_z_filtered + (1.f - alpha) * delta_v_z_limited;

	if (fabsf(g_delta_v_z_filtered) < CESO_Z_DEADZONE) {
		return 0.f;
	}

	return g_delta_v_z_filtered;
}

} // namespace


void PosControl::setControlParas(const Vector3f &bm_lambda_p, const Vector3f &bm_K_p)
{
	_contolParas.bm_lambda_p(0, 0) = bm_lambda_p(0);
	_contolParas.bm_lambda_p(1, 1) = bm_lambda_p(1);
	_contolParas.bm_lambda_p(2, 2) = bm_lambda_p(2);

	_contolParas.bm_Kp(0, 0) = bm_K_p(0);
	_contolParas.bm_Kp(1, 1) = bm_K_p(1);
	_contolParas.bm_Kp(2, 2) = bm_K_p(2);
}

void PosControl::setCESOParas(const Vector3f &CESO_l1, const Vector3f &CESO_l2,
			      const float &CESO_EPSI, const float &CESO_c1, const float &CESO_c2)
{
	_pregme_eso.L1 = CESO_l1;
	_pregme_eso.L2 = CESO_l2;
	_pregme_eso.EPSI = CESO_EPSI;
	_pregme_eso.c1 = CESO_c1;
	_pregme_eso.c2 = CESO_c2;
}

void PosControl::setPresetTrajParas(const float &PresetTraj_l, const float &PresetTraj_w,
				    const float &PresetTraj_epsilon, const float &PresetTraj_k)
{
	_preset_traj.l = PresetTraj_l;
	_preset_traj.w = PresetTraj_w;
	_preset_traj.epsilon = PresetTraj_epsilon;
	_preset_traj.k = PresetTraj_k;
}

void PosControl::setPresetTraj(const Vector3f e0, const Vector3f ev0)
{
	const float absolute_time = hrt_absolute_time() * 1.e-6f;

	ControlMath::setZeroIfNanVector3f(_preset_traj.e0_last);
	ControlMath::setZeroIfNanVector3f(_preset_traj.ev0_last);

	for (int i = 0; i < 3; i++) {
		if (!PX4_ISFINITE(_pos_sp_last(i))) {
			// V1.13-compatible initialization: force the first cycle after reset
			// to enter the preset-trajectory refresh branch below. If this is
			// initialized exactly to _pos_sp(i), e0_last/ev0_last can remain zero
			// after reset and the transient trajectory is not initialized from
			// the current position/velocity error.
			_pos_sp_last(i) = _pos_sp(i) + _preset_traj.epsilon;
		}
	}

	for (int i = 0; i < 3; i++) {
		_preset_traj.e0(i) = _preset_traj.w * e0(i);
		_preset_traj.ev0(i) = _preset_traj.w * ev0(i);

		if (fabsf(_pos_sp_last(i) - _pos_sp(i)) >= _preset_traj.epsilon) {
			_preset_traj.time(i) = 0.f;
			_preset_traj.time_last(i) = absolute_time;
			_preset_traj.e0_last(i) = _preset_traj.w * e0(i);
			_preset_traj.ev0_last(i) = _preset_traj.w * ev0(i);

		} else {
			_preset_traj.time(i) = absolute_time - _preset_traj.time_last(i);
			_preset_traj.e0(i) = _preset_traj.e0_last(i);
			_preset_traj.ev0(i) = _preset_traj.ev0_last(i);
		}

		_preset_traj.b = _preset_traj.l * _preset_traj.e0(i) + _preset_traj.ev0(i);
		_preset_traj.c = fabsf(_preset_traj.b) / 2.0f + _preset_traj.epsilon;

		_preset_traj.ed(i) = _preset_traj.e0(i) * expf(-_preset_traj.l * _preset_traj.time(i))
				   + _preset_traj.b / _preset_traj.c
				     * (1.f - expf(-_preset_traj.c * _preset_traj.time(i)))
				     * expf(-_preset_traj.l * _preset_traj.time(i));

		_preset_traj.ed_dot(i) = -_preset_traj.l * _preset_traj.ed(i)
				       + _preset_traj.b * expf(-(_preset_traj.l + _preset_traj.c) * _preset_traj.time(i));

		_preset_traj.ed_ddot(i) = -_preset_traj.l * _preset_traj.ed_dot(i)
					- _preset_traj.b * (_preset_traj.l + _preset_traj.c)
					  * expf(-(_preset_traj.l + _preset_traj.c) * _preset_traj.time(i));

		_pos_sp_last(i) = _pos_sp(i);
	}
}

void PosControl::setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PosControl::setThrustLimits(const float min, const float max)
{
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PosControl::updateHoverThrust(const float hover_thrust_new)
{
	if (!PX4_ISFINITE(hover_thrust_new)) {
		return;
	}

	const float hover_thrust_limited = math::constrain(hover_thrust_new, _lim_thr_min, _lim_thr_max);
	setHoverThrust(hover_thrust_limited);
}

void PosControl::setState(const PositionControlStates &states)
{
	_states = states;
	_pos = states.position;
	_vel = states.velocity;
	_vel_dot = states.acceleration;
	if (!PX4_ISFINITE(_yaw_sp)) {
		_yaw_sp = states.yaw;
	}
}

void PosControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position);
	_vel_sp = Vector3f(setpoint.velocity);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

void PosControl::setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

bool PosControl::update(const float dt)
{
	_prepareSetpointCompatibility(dt);

	const bool valid = (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)))
			   && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)))
			   && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	if (valid) {
		_positionControl(dt);

		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _states.yaw;
	}

	return valid && _updateSuccessful();
}

void PosControl::_prepareSetpointCompatibility(const float dt)
{
	for (int i = 0; i < 3; i++) {
		if (!PX4_ISFINITE(_pos_sp(i)) && PX4_ISFINITE(_vel_sp(i)) && PX4_ISFINITE(_pos(i))) {
			_pos_sp(i) = _pos(i) + _vel_sp(i) * dt;
		}
	}
}

void PosControl::_positionControl(const float dt)
{
	_autopilot.pos_err = _pos - _pos_sp;
	ControlMath::setZeroIfNanVector3f(_autopilot.pos_err);
	ControlMath::setZeroIfNanVector3f(_vel_sp);
	_vel_sp(0) = math::constrain(_vel_sp(0), -_lim_vel_horizontal, _lim_vel_horizontal);
	_vel_sp(1) = math::constrain(_vel_sp(1), -_lim_vel_horizontal, _lim_vel_horizontal);
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);

	_autopilot.vel_err = _vel - _vel_sp;
	ControlMath::setZeroIfNanVector3f(_autopilot.vel_err);

	const bool vertical_control_enabled = PX4_ISFINITE(_pos_sp(2)) || PX4_ISFINITE(_vel_sp(2));

	delta_v = _pregme_eso.delta_est;
	ControlMath::setZeroIfNanVector3f(delta_v);

	// Keep Z-axis CESO disturbance rejection, but do not inject the raw observer
	// output directly into thrust. The protected injection preserves the anti-
	// disturbance path while avoiding vertical thrust pumping in OFFBOARD hover.
	delta_v(2) = updateVerticalCESOInjection(delta_v(2), vertical_control_enabled, dt);

	setPresetTraj(_autopilot.pos_err, _autopilot.vel_err);

	_autopilot.zp = _autopilot.pos_err - _preset_traj.k * _preset_traj.ed;
	_autopilot.zp_dot = _autopilot.vel_err - _preset_traj.k * _preset_traj.ed_dot;
	_autopilot.slide_mode = _autopilot.zp_dot + _contolParas.bm_lambda_p * _autopilot.zp;

	_autopilot.f_iusl = _contolParas.bm_Kp * _autopilot.slide_mode + g + delta_v
		+ _contolParas.bm_lambda_p * _autopilot.zp_dot - _preset_traj.k * _preset_traj.ed_ddot;

	const Vector3f u_v = -_autopilot.f_iusl + g;
	PositionCESO(_pos, u_v, dt);

	_acc_cal = -_autopilot.f_iusl + g;

	Vector3f body_z = Vector3f(-_acc_cal(0), -_acc_cal(1), G).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);

	float collective_thrust = _acc_cal(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	collective_thrust /= math::max(Vector3f(0, 0, 1).dot(body_z), 0.1f);

	if (!PX4_ISFINITE(collective_thrust)) {
		collective_thrust = 0.f;
	}

	collective_thrust = math::min(collective_thrust, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;

	_thr_sp(2) = math::max(_thr_sp(2), -_lim_thr_max);

	const float thrust_max_squared = _lim_thr_max * _lim_thr_max;
	const float thrust_z_squared = _thr_sp(2) * _thr_sp(2);
	const float thrust_max_xy_squared = thrust_max_squared - thrust_z_squared;
	float thrust_max_xy = 0.f;

	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	Vector2f thrust_sp_xy(_thr_sp(0), _thr_sp(1));
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();

	if (thrust_sp_xy_norm > thrust_max_xy && thrust_sp_xy_norm > 1e-6f) {
		_thr_sp(0) = thrust_sp_xy(0) / thrust_sp_xy_norm * thrust_max_xy;
		_thr_sp(1) = thrust_sp_xy(1) / thrust_sp_xy_norm * thrust_max_xy;
	}
}

bool PosControl::_updateSuccessful()
{
	bool valid = true;

	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	valid = valid && PX4_ISFINITE(_acc_cal(0)) && PX4_ISFINITE(_acc_cal(1)) && PX4_ISFINITE(_acc_cal(2));
	valid = valid && PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1)) && PX4_ISFINITE(_thr_sp(2));
	return valid;
}

void PosControl::PositionCESO(matrix::Vector3f pos_in, matrix::Vector3f u, float dt)
{
	if (!_is_initialized) {
		_pregme_eso.xi1 = pos_in;
		_is_initialized = true;
	}

	ControlMath::setZeroIfNanVector3f(_pregme_eso.xi1);
	ControlMath::setZeroIfNanVector3f(_pregme_eso.xi2);
	ControlMath::setZeroIfNanVector3f(_pregme_eso.vel_est);
	ControlMath::setZeroIfNanVector3f(_pregme_eso.delta_est);
	ControlMath::setZeroIfNanVector3f(pos_in);
	ControlMath::setZeroIfNanVector3f(u);
	dt = (PX4_ISFINITE(dt) && dt > 0.f) ? math::constrain(dt, 0.001f, 0.04f) : 0.0f;

	const float eso_epsi = (PX4_ISFINITE(_pregme_eso.EPSI) && fabsf(_pregme_eso.EPSI) > 1e-4f)
			       ? _pregme_eso.EPSI : 1e-4f;

	for (int i = 0; i < 3; i++) {
		const float error1 = pos_in(i) - _pregme_eso.xi1(i);
		const float function_g1 = PositionCESO_function_g(error1, _pregme_eso.L1(i));

		const float xi1_dot = function_g1 / eso_epsi;
		_pregme_eso.vel_est(i) = xi1_dot;
		_pregme_eso.xi1(i) = _pregme_eso.xi1(i) + xi1_dot * dt;

		const float error2 = _pregme_eso.vel_est(i) - _pregme_eso.xi2(i);
		const float function_g2 = PositionCESO_function_g(error2, _pregme_eso.L2(i));

		const float xi2_dot = function_g2 / eso_epsi + u(i);

		_pregme_eso.delta_est(i) = function_g2 / eso_epsi;
		_pregme_eso.xi2(i) = _pregme_eso.xi2(i) + xi2_dot * dt;
	}
}

float PosControl::PositionCESO_function_g(float error, float l)
{
	if (!PX4_ISFINITE(error) || !PX4_ISFINITE(l)) {
		return 0.f;
	}

	const float error_limited = math::constrain(error, -20.f, 20.f);
	const float exp_sum = expf(error_limited) + expf(-error_limited);
	const float denominator = _pregme_eso.c1 * exp_sum + _pregme_eso.c2;

	if (!PX4_ISFINITE(denominator) || fabsf(denominator) < 1e-6f) {
		return 0.f;
	}

	return l * error_limited * exp_sum / denominator;
}

void PosControl::resetESO()
{
	_is_initialized = false;
	_pregme_eso.delta_est = {0.0f, 0.0f, 0.0f};
	_pregme_eso.vel_est = {0.0f, 0.0f, 0.0f};
	_pregme_eso.xi1 = {NAN, NAN, NAN};
	_pregme_eso.xi2 = {0.0f, 0.0f, 0.0f};

	// Reset the protected Z-CESO injection path together with the observer.
	g_delta_v_z_filtered = 0.f;
	g_delta_v_z_filter_initialized = false;
}

void PosControl::resetPresetTraj()
{
	_pos_sp_last(0) = NAN;
	_pos_sp_last(1) = NAN;
	_pos_sp_last(2) = NAN;

	_preset_traj.time_last(0) = hrt_absolute_time() / 1.e6f;
	_preset_traj.time_last(1) = hrt_absolute_time() / 1.e6f;
	_preset_traj.time_last(2) = hrt_absolute_time() / 1.e6f;

	for (int i = 0; i < 3; i++) {
		_preset_traj.e0_last(i) = NAN;
		_preset_traj.ev0_last(i) = NAN;
	}
}

void PosControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void PosControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
