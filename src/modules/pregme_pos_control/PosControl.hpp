#pragma once

#include "ControlMath.hpp"

#include <cmath>
#include <float.h>
#include <matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/gamma_arm_dynamics/ArmJointSubscriber.hpp>

#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

#define G 9.8066f
#define SUM_max_x 0.25f
#define SUM_max_y 0.25f
#define SUM_max_z 0.25f
#define Pi 3.1415926f

struct PositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw{};
};

struct ControlParas {
	matrix::Matrix<float, 3, 3> bm_lambda_p;
	matrix::Matrix<float, 3, 3> bm_Kp;
};

struct Autopilot {
	matrix::Vector3f pos_err;
	matrix::Vector3f vel_err;
	matrix::Vector3f slide_mode;
	matrix::Vector3f zp;
	matrix::Vector3f zp_dot;
	matrix::Vector3f pos_err_integ;
	matrix::Vector3f f_iusl;
	matrix::Vector3f bm_sv;
	matrix::Vector3f bm_vr;
	matrix::Vector3f bm_vr_deriv;
};

class PosControl
{
public:
	PosControl() = default;
	~PosControl() = default;


	void setControlParas(const matrix::Vector3f &bm_lambda_p, const matrix::Vector3f &bm_K_p);
	void setCESOParas(const matrix::Vector3f &CESO_l1, const matrix::Vector3f &CESO_l2,
			  const float &CESO_EPSI, const float &CESO_c1, const float &CESO_c2);
	void setPresetTrajParas(const float &PresetTraj_l, const float &PresetTraj_w,
				const float &PresetTraj_epsilon, const float &PresetTraj_k);

	float PositionCESO_function_g(float error, float l);
	void resetESO();
	void resetPresetTraj();

	void setPresetTraj(const matrix::Vector3f e0, const matrix::Vector3f ev0);

	void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);
	void setThrustLimits(const float min, const float max);
	void setTiltLimit(const float tilt) { _lim_tilt = tilt; }
	void setHoverThrust(const float hover_thrust) { _hover_thrust = math::constrain(hover_thrust, 0.1f, 0.9f); }
	void updateHoverThrust(const float hover_thrust_new);

	void setState(const PositionControlStates &states);
	void setInputSetpoint(const trajectory_setpoint_s &setpoint);
	void setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint);

	/** 设置机体状态 (位姿 + 角速度), 用于机械臂耦合补偿。 */
	void setBodyState(const matrix::Dcmf &R_body_to_world, const matrix::Vector3f &omega_body);

	bool update(const float dt);

	void resetIntegral() { _autopilot.pos_err_integ.setZero(); }

	void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;
	void getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const;

private:
	bool _updateSuccessful();
	void _positionControl(const float dt);
	void PositionCESO(matrix::Vector3f pos_in, matrix::Vector3f f, float dt);

	ControlParas _contolParas{};

	Autopilot _autopilot{};

	matrix::Vector3f g = matrix::Vector3f(0.f, 0.f, G);

	PositionControlStates _states{};

	float _lim_vel_horizontal{};
	float _lim_vel_up{};
	float _lim_vel_down{};
	float _lim_thr_min{};
	float _lim_thr_max{};
	float _lim_tilt{};
	float _hover_thrust{0.5f};

	matrix::Vector3f _pos{};
	matrix::Vector3f _vel{};
	matrix::Vector3f _vel_dot{};
	matrix::Vector3f delta_v{};

	matrix::Vector3f _pos_sp{};
	matrix::Vector3f _vel_sp{};
	matrix::Vector3f _acc_sp{};
	matrix::Vector3f _acc_cal{};
	matrix::Vector3f _thr_sp{};

	float _yaw_sp{};
	float _yawspeed_sp{};

	// 机械臂耦合补偿 (CoM offset → velocity compensation)
	matrix::Dcmf _R_body_to_world;             // 世界系到机体系的旋转矩阵
	matrix::Vector3f _omega_body{};            // 机体角速度 [rad/s]
	matrix::Vector3f _p_c_b{};                 // 系统总质心 (机体系 NED), 从 ArmJointSubscriber 读取
	matrix::Vector3f _delta_v_comp{};          // Δv = -R·[ω×(ω×p_C^B)]

	struct pregme_ESO {
		float EPSI{1.f};
		float c1{0.3f};
		float c2{0.5f};

		matrix::Vector3f xi1{};
		matrix::Vector3f xi2{};
		matrix::Vector3f vel_est{};
		matrix::Vector3f delta_est{};
		matrix::Vector3f L1{};
		matrix::Vector3f L2{};
	} _pregme_eso{};

	matrix::Vector3f _pos_sp_last{};

	struct preset_traj {
		float l{1.f};
		float c{0.f};
		float k{1.f};
		float b{0.f};
		float w{0.1f};
		float epsilon{0.05f};

		matrix::Vector3f time{};
		matrix::Vector3f time_last{};
		matrix::Vector3f ed{};
		matrix::Vector3f ed_dot{};
		matrix::Vector3f ed_ddot{};
		matrix::Vector3f e0{};
		matrix::Vector3f ev0{};
		matrix::Vector3f e0_last{};
		matrix::Vector3f ev0_last{};
	} _preset_traj{};

	bool _is_initialized{false};

	void _prepareSetpointCompatibility(const float dt);
};
