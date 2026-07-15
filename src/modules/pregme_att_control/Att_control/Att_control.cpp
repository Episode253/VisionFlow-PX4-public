#include "Att_control.hpp"

#include <drivers/drv_hrt.h>
#include <math.h>
#include <mathlib/math/Limits.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;

namespace
{
constexpr float kMinDt = 0.0002f;
constexpr float kMaxDt = 0.02f;
constexpr float kMinEsoEpsilon = 1.0e-3f;
constexpr float kMinEsoDenominator = 1.0e-6f;
constexpr float kMinPresetEpsilon = 1.0e-4f;
constexpr float kMinQuatScalarForInverse = 1.0e-3f;
constexpr float kEsoEnableZ = -1.0f; // PX4 local position is NED: z < -1 means above roughly 1 m.

using Matrix3fLocal = SquareMatrix<float, 3>;

bool isFiniteVector3(const Vector3f &v)
{
	return PX4_ISFINITE(v(0)) && PX4_ISFINITE(v(1)) && PX4_ISFINITE(v(2));
}

bool isFiniteQuat(const Quatf &q)
{
	return PX4_ISFINITE(q(0)) && PX4_ISFINITE(q(1)) && PX4_ISFINITE(q(2)) && PX4_ISFINITE(q(3));
}

void zeroVector3(Vector3f &v)
{
	v(0) = 0.f;
	v(1) = 0.f;
	v(2) = 0.f;
}

void setNanVector4(Vector<float, 4> &v)
{
	for (int i = 0; i < 4; i++) {
		v(i) = NAN;
	}
}

void zeroMatrix3(Matrix3fLocal &m)
{
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			m(r, c) = 0.f;
		}
	}
}

void identityMatrix3(Matrix3fLocal &m)
{
	zeroMatrix3(m);
	m(0, 0) = 1.f;
	m(1, 1) = 1.f;
	m(2, 2) = 1.f;
}

void identitySquareMatrix3(SquareMatrix<float, 3> &m)
{
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			m(r, c) = (r == c) ? 1.f : 0.f;
		}
	}
}

Matrix3fLocal skewSymmetric(const Vector3f &v)
{
	Matrix3fLocal m;
	zeroMatrix3(m);
	m(0, 1) = -v(2);
	m(0, 2) =  v(1);
	m(1, 0) =  v(2);
	m(1, 2) = -v(0);
	m(2, 0) = -v(1);
	m(2, 1) =  v(0);
	return m;
}

void sanitizeVector3(Vector3f &v)
{
	for (int i = 0; i < 3; i++) {
		if (!PX4_ISFINITE(v(i))) {
			v(i) = 0.f;
		}
	}
}

float safeExp(float x)
{
	return expf(math::constrain(x, -50.f, 50.f));
}

} // namespace

Att_Control::Att_Control()
{
	Controllerinit();
}

void Att_Control::Controllerinit()
{
	zeroVector3(_rate_limit);
	zeroVector3(_tau);

	_attitude_setpoint_q(0) = 1.f;
	_attitude_setpoint_q(1) = 0.f;
	_attitude_setpoint_q(2) = 0.f;
	_attitude_setpoint_q(3) = 0.f;
	_yawspeed_setpoint = 0.f;

	setNanVector4(_attitude_setpoint_q_last);
	identityMatrix3(_eye_3);

	zeroVector3(_usr_eso.xi);
	zeroVector3(_usr_eso.xi_dot);
	zeroVector3(_usr_eso.delta_esti);
	zeroVector3(_usr_eso.delta_esti_dot);
	zeroVector3(_usr_eso.L);
	_usr_eso.EPSI = 1.f;
	_usr_eso.c1 = 0.3f;
	_usr_eso.c2 = 0.5f;

	zeroMatrix3(_controller_param.lambda_q);
	zeroMatrix3(_controller_param.K_q);

	identitySquareMatrix3(_I_b);
	identitySquareMatrix3(_I_b_inve);

	_preset_traj.l = 1.f;
	_preset_traj.c = 1.f;
	_preset_traj.k = 1.f;
	_preset_traj.b = 0.f;
	_preset_traj.w = 0.1f;
	_preset_traj.epsilon = 0.05f;

	zeroVector3(_preset_traj.time);
	zeroVector3(_preset_traj.time_last);
	zeroVector3(_preset_traj.ed);
	zeroVector3(_preset_traj.ed_dot);
	zeroVector3(_preset_traj.ed_ddot);
	zeroVector3(_preset_traj.e0);
	zeroVector3(_preset_traj.ev0);
	zeroVector3(_preset_traj.e0_last);
	zeroVector3(_preset_traj.ev0_last);

	resetPresetTraj();
}

void Att_Control::setControllerGain(const Vector3f &lambda_q, const Vector3f &k_q)
{
	zeroMatrix3(_controller_param.lambda_q);
	zeroMatrix3(_controller_param.K_q);

	for (int i = 0; i < 3; i++) {
		_controller_param.lambda_q(i, i) = PX4_ISFINITE(lambda_q(i)) ? math::max(lambda_q(i), 0.f) : 0.f;
		_controller_param.K_q(i, i) = PX4_ISFINITE(k_q(i)) ? math::max(k_q(i), 0.f) : 0.f;
	}
}

void Att_Control::setInertiaMatrix(const SquareMatrix<float, 3> &Ib)
{
	bool valid = true;

	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			valid = valid && PX4_ISFINITE(Ib(r, c));
		}
	}

	valid = valid && (Ib(0, 0) > 0.f) && (Ib(1, 1) > 0.f) && (Ib(2, 2) > 0.f);

	if (valid) {
		_I_b = Ib;
		_I_b_inve = inv(Ib);

	} else {
		identitySquareMatrix3(_I_b);
		identitySquareMatrix3(_I_b_inve);
	}
}

void Att_Control::setRateLimit(const Vector3f &rate_limit)
{
	for (int i = 0; i < 3; i++) {
		_rate_limit(i) = (PX4_ISFINITE(rate_limit(i)) && rate_limit(i) > 0.f) ? rate_limit(i) : 0.f;
	}
}

void Att_Control::setCESOParas(const Vector3f &CESO_l, float CESO_EPSI, float CESO_c1, float CESO_c2)
{
	for (int i = 0; i < 3; i++) {
		_usr_eso.L(i) = PX4_ISFINITE(CESO_l(i)) ? math::max(CESO_l(i), 0.f) : 0.f;
	}

	_usr_eso.EPSI = PX4_ISFINITE(CESO_EPSI) ? math::max(CESO_EPSI, kMinEsoEpsilon) : 1.f;
	_usr_eso.c1 = PX4_ISFINITE(CESO_c1) ? math::max(CESO_c1, kMinEsoDenominator) : 0.3f;
	_usr_eso.c2 = PX4_ISFINITE(CESO_c2) ? math::max(CESO_c2, kMinEsoDenominator) : 0.5f;
}

void Att_Control::setPresetTrajParas(float PresetTraj_l, float PresetTraj_w, float PresetTraj_epsilon, float PresetTraj_k)
{
	_preset_traj.l = PX4_ISFINITE(PresetTraj_l) ? math::max(PresetTraj_l, 0.f) : 1.f;
	_preset_traj.w = PX4_ISFINITE(PresetTraj_w) ? math::max(PresetTraj_w, 0.f) : 0.1f;
	_preset_traj.epsilon = PX4_ISFINITE(PresetTraj_epsilon) ? math::max(PresetTraj_epsilon, kMinPresetEpsilon) : 0.05f;
	_preset_traj.k = PX4_ISFINITE(PresetTraj_k) ? math::max(PresetTraj_k, 0.f) : 1.f;
}

void Att_Control::setAttitudeSetpoint(const Quatf &qd, float yawspeed_setpoint)
{
	if (isFiniteQuat(qd)) {
		_attitude_setpoint_q = qd;
		_attitude_setpoint_q.normalize();
	}

	_yawspeed_setpoint = PX4_ISFINITE(yawspeed_setpoint) ? yawspeed_setpoint : 0.f;
}

void Att_Control::adaptAttitudeSetpoint(const Quatf &q_delta)
{
	if (isFiniteQuat(q_delta)) {
		_attitude_setpoint_q = q_delta * _attitude_setpoint_q;
		_attitude_setpoint_q.normalize();
	}
}

void Att_Control::setPresetTraj(const Vector3f qv_error, const Vector3f qv_error_dot)
{
	const float absolute_time = hrt_absolute_time() * 1.e-6f;

	for (int i = 1; i < 4; i++) {
		if (!PX4_ISFINITE(_attitude_setpoint_q_last(i))) {
			_attitude_setpoint_q_last(i) = _attitude_setpoint_q(i) + _preset_traj.epsilon;
		}
	}

	for (int i = 0; i < 3; i++) {
		_preset_traj.e0(i) = _preset_traj.w * qv_error(i);
		_preset_traj.ev0(i) = _preset_traj.w * qv_error_dot(i);

		if (fabsf(_attitude_setpoint_q_last(i + 1) - _attitude_setpoint_q(i + 1)) >= _preset_traj.epsilon) {
			_preset_traj.time(i) = 0.f;
			_preset_traj.time_last(i) = absolute_time;
			_preset_traj.e0_last(i) = _preset_traj.e0(i);
			_preset_traj.ev0_last(i) = _preset_traj.ev0(i);

		} else {
			_preset_traj.time(i) = math::max(absolute_time - _preset_traj.time_last(i), 0.f);
			_preset_traj.e0(i) = PX4_ISFINITE(_preset_traj.e0_last(i)) ? _preset_traj.e0_last(i) : _preset_traj.e0(i);
			_preset_traj.ev0(i) = PX4_ISFINITE(_preset_traj.ev0_last(i)) ? _preset_traj.ev0_last(i) : _preset_traj.ev0(i);
		}

		_preset_traj.b = _preset_traj.l * _preset_traj.e0(i) + _preset_traj.ev0(i);
		_preset_traj.c = fabsf(_preset_traj.b) / 0.1f + _preset_traj.epsilon;

		const float exp_l = safeExp(-_preset_traj.l * _preset_traj.time(i));
		const float exp_c = safeExp(-_preset_traj.c * _preset_traj.time(i));
		const float exp_lc = safeExp(-(_preset_traj.l + _preset_traj.c) * _preset_traj.time(i));

		_preset_traj.ed(i) = _preset_traj.e0(i) * exp_l
				       + _preset_traj.b / _preset_traj.c * (1.f - exp_c) * exp_l;

		_preset_traj.ed_dot(i) = -_preset_traj.l * _preset_traj.ed(i) + _preset_traj.b * exp_lc;

		_preset_traj.ed_ddot(i) = -_preset_traj.l * _preset_traj.ed_dot(i)
					 - _preset_traj.b * (_preset_traj.l + _preset_traj.c) * exp_lc;

		_attitude_setpoint_q_last(i + 1) = _attitude_setpoint_q(i + 1);
	}
}

void Att_Control::update(const Quatf &q,
				 const Vector3f &rate,
				 float dt,
				 bool landed,
				 Vector3f &torque,
				 Vector3f &rates_sp,
				 float pos_z)
{
	if (landed) {
		resetESO();
		resetPresetTraj();
	}

	runAttitudeControl(q, rate, dt, torque, rates_sp, pos_z);
}

void Att_Control::runAttitudeControl(const Quatf &q,
					     const Vector3f &rate,
					     float dt,
					     Vector3f &torque,
					     Vector3f &rates_sp,
					     float pos_z)
{
	zeroVector3(torque);
	zeroVector3(rates_sp);

	if (!isFiniteQuat(q) || !isFiniteQuat(_attitude_setpoint_q)) {
		resetESO();
		resetPresetTraj();
		return;
	}

	dt = PX4_ISFINITE(dt) ? math::constrain(dt, kMinDt, kMaxDt) : kMinDt;

	Vector3f current_rate = rate;
	sanitizeVector3(current_rate);

	Quatf q_current = q;
	q_current.normalize();

	// 缓存 body→world 旋转矩阵, 供 updateCouplingCompensation 使用
	_R_body_to_world = Dcmf(q_current);

	Quatf q_error = _attitude_setpoint_q.inversed() * q_current;
	q_error.normalize();

	// Use the shortest quaternion representation to avoid unwinding.
	if (q_error(0) < 0.f) {
		for (int i = 0; i < 4; i++) {
			q_error(i) = -q_error(i);
		}
	}

	const Vector3f qv_error = q_error.imag();
	const float q0_error = q_error(0);

	if (!PX4_ISFINITE(q0_error) || fabsf(q0_error) < kMinQuatScalarForInverse) {
		resetESO();
		resetPresetTraj();
		return;
	}

	const Matrix3fLocal q_skew_symm = skewSymmetric(qv_error);

	Vector3f omega_r;
	zeroVector3(omega_r);

	if (PX4_ISFINITE(_yawspeed_setpoint)) {
		omega_r += q_current.inversed().dcm_z() * _yawspeed_setpoint;
	}

	for (int i = 0; i < 3; i++) {
		if (_rate_limit(i) > 0.f) {
			omega_r(i) = math::constrain(omega_r(i), -_rate_limit(i), _rate_limit(i));
		}
	}

	rates_sp = omega_r;

	const Vector3f omega_error = current_rate - omega_r;
	const float q0_error_dot = -0.5f * (qv_error * omega_error);

	Matrix3fLocal Q = _eye_3 * q0_error + q_skew_symm;
	Matrix3fLocal Q_inv = inv(Q);

	const Vector3f qv_error_dot = Q * omega_error * 0.5f;
	const Matrix3fLocal qv_error_dot_skew_symm = skewSymmetric(qv_error_dot);
	const Matrix3fLocal Q_dot = _eye_3 * q0_error_dot + qv_error_dot_skew_symm;

	setPresetTraj(qv_error, qv_error_dot);

	Vector3f zq = qv_error - _preset_traj.k * _preset_traj.ed;
	Vector3f zq_dot = qv_error_dot - _preset_traj.k * _preset_traj.ed_dot;
	const Vector3f slide_mode_q = zq_dot + _controller_param.lambda_q * zq;

	if (PX4_ISFINITE(pos_z) && pos_z < kEsoEnableZ) {
		sanitizeVector3(_usr_eso.delta_esti);

	} else {
		zeroVector3(_usr_eso.delta_esti);
	}

	// 机械臂质心耦合补偿 (从 ArmJointSubscriber 读取最新 CoM)
	updateCouplingCompensation();

	const Vector3f control_vec = (-1.f) * (_controller_param.K_q * slide_mode_q)
					   + (-0.5f) * (Q_dot * omega_error)
					   + (-1.f) * (_controller_param.lambda_q * zq_dot)
					   + _preset_traj.k * _preset_traj.ed_ddot;

	const Vector3f gyro_comp = current_rate % (_I_b * current_rate);

	// 机械臂质心偏移补偿: Δω_comp = I⁻¹[m_total · p_C^B × R^T·g]
	// ESO 只估计残余未知扰动, 控制端显式加上已知 Δω_comp
	const Vector3f delta_omega_total = _usr_eso.delta_esti + _delta_omega_comp;

	torque = 2.f * (_I_b * (Q_inv * control_vec)) + gyro_comp - _I_b * delta_omega_total;

	if (!isFiniteVector3(torque)) {
		zeroVector3(torque);
		resetESO();
		resetPresetTraj();
		return;
	}

	_tau = torque;
	const Vector3f u_w = _I_b_inve * (_tau - gyro_comp);

	// 把已知耦合补偿项并入系统输入, ESO 只估计残差
	const Vector3f u_w_for_eso = u_w + _delta_omega_comp;
	UsrAttitudeESO(current_rate, u_w_for_eso, dt);

	for (int i = 0; i < 3; i++) {
		_usr_eso.delta_esti(i) = PX4_ISFINITE(_usr_eso.delta_esti(i)) ?
					 math::constrain(_usr_eso.delta_esti(i), -20.f, 20.f) : 0.f;
	}
}

void Att_Control::UsrAttitudeESO(Vector3f bm_omega, Vector3f u, float dt)
{
	sanitizeVector3(_usr_eso.delta_esti);
	sanitizeVector3(_usr_eso.xi);
	sanitizeVector3(bm_omega);
	sanitizeVector3(u);

	dt = PX4_ISFINITE(dt) ? math::constrain(dt, kMinDt, kMaxDt) : kMinDt;
	const float eps = math::max(_usr_eso.EPSI, kMinEsoEpsilon);

	for (int i = 0; i < 3; i++) {
		const float error = bm_omega(i) - _usr_eso.xi(i);
		const float function_g = CESO_function_g(error, _usr_eso.L(i));
		const float xi_dot = function_g / eps + u(i);

		_usr_eso.xi_dot(i) = xi_dot;
		_usr_eso.delta_esti(i) = function_g / eps;
		_usr_eso.xi(i) += xi_dot * dt;
	}
}

void Att_Control::resetESO()
{
	zeroVector3(_usr_eso.delta_esti);
	zeroVector3(_usr_eso.xi);
	zeroVector3(_usr_eso.xi_dot);
	zeroVector3(_usr_eso.delta_esti_dot);
	zeroVector3(_tau);
}

void Att_Control::resetPresetTraj()
{
	setNanVector4(_attitude_setpoint_q_last);

	const float now = hrt_absolute_time() * 1.e-6f;

	for (int i = 0; i < 3; i++) {
		_preset_traj.time(i) = 0.f;
		_preset_traj.time_last(i) = now;
		_preset_traj.ed(i) = 0.f;
		_preset_traj.ed_dot(i) = 0.f;
		_preset_traj.ed_ddot(i) = 0.f;
		_preset_traj.e0(i) = 0.f;
		_preset_traj.ev0(i) = 0.f;
		_preset_traj.e0_last(i) = NAN;
		_preset_traj.ev0_last(i) = NAN;
	}
}

float Att_Control::CESO_function_g(float error, float l) const
{
	const float safe_error = PX4_ISFINITE(error) ? math::constrain(error, -20.f, 20.f) : 0.f;
	const float safe_l = PX4_ISFINITE(l) ? math::max(l, 0.f) : 0.f;
	const float ch = safeExp(safe_error) + safeExp(-safe_error);
	const float denominator = math::max(_usr_eso.c1 * ch + _usr_eso.c2, kMinEsoDenominator);

	return safe_l * safe_error * ch / denominator;
}

void Att_Control::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status) const
{
	rate_ctrl_status.rollspeed_integ = PX4_ISFINITE(_usr_eso.delta_esti(0)) ? _usr_eso.delta_esti(0) : 0.f;
	rate_ctrl_status.pitchspeed_integ = PX4_ISFINITE(_usr_eso.delta_esti(1)) ? _usr_eso.delta_esti(1) : 0.f;
	rate_ctrl_status.yawspeed_integ = PX4_ISFINITE(_usr_eso.delta_esti(2)) ? _usr_eso.delta_esti(2) : 0.f;
}

// 机械臂质心耦合补偿
//
// Δω_comp = I_b⁻¹ [m_total · p_C^B × (R^T · g)]
//
// 物理含义:
//   p_C^B: 系统总质心在机体系下的偏移 (来自 ArmJointSubscriber)
//   R^T·g: 重力在机体系下的方向
//   p_C^B × (R^T·g): 质心偏移产生的重力矩臂
//   乘以 m_total 得到力矩, 除以 I_b 得到角加速度补偿量

void Att_Control::updateCouplingCompensation()
{
	auto *arm = ArmJointSubscriber::instance();

	_p_c_b = arm->getSystemCom();
	_coupling_total_mass = arm->getTotalMass();

	// 将 ArmJointSubscriber 的质心数据传递给姿态环
	// 如果质心全零 (机械臂未运动), 不产生补偿
	const float com_norm = _p_c_b.norm();

	if (com_norm < 1e-6f || _coupling_total_mass < 0.1f) {
		zeroVector3(_delta_omega_comp);
		return;
	}

	// g_n 在世界 NED 系下为 (0, 0, 9.8066)
	constexpr float G = 9.8066f;
	const Vector3f g_n(0.f, 0.f, G);

	// 重力在机体系下的方向
	const Vector3f body_g_dir = _R_body_to_world.transpose() * g_n;

	// 重力矩: τ_g = m_total · p_C^B × (R^T · g)
	const Vector3f gravity_torque = _coupling_total_mass * (_p_c_b.cross(body_g_dir));

	// 角加速度补偿: Δω = I⁻¹ · τ_g
	_delta_omega_comp = _I_b_inve * gravity_torque;

	// NaN 保护
	for (int i = 0; i < 3; ++i) {
		if (!PX4_ISFINITE(_delta_omega_comp(i))) {
			_delta_omega_comp(i) = 0.f;
		}
	}
}
