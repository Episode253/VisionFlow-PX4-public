#pragma once

#include <matrix/matrix/math.hpp>
#include <uORB/topics/rate_ctrl_status.h>

class Att_Control
{
public:
	Att_Control();
	~Att_Control() = default;

	void Controllerinit();

	/** Set attitude error and sliding-mode gains. */
	void setControllerGain(const matrix::Vector3f &lambda_q, const matrix::Vector3f &k_q);

	/** Set body inertia matrix and pre-compute its inverse. */
	void setInertiaMatrix(const matrix::SquareMatrix<float, 3> &Ib);

	/** Set hard limit for reported/body-frame rate setpoints [rad/s]. */
	void setRateLimit(const matrix::Vector3f &rate_limit);

	void resetESO();
	void resetPresetTraj();

	void setCESOParas(const matrix::Vector3f &CESO_l, float CESO_EPSI, float CESO_c1, float CESO_c2);
	float CESO_function_g(float error, float l) const;

	/**
	 * Set COM offset compensation state for angular rate coupling.
	 * @param R_body_to_world rotation from body to world frame
	 * @param p_c_b COM offset expressed in body frame [m]
	 * @param total_mass total vehicle mass [kg] (UAV + manipulator)
	 */
	void setCouplingCompState(const matrix::Dcmf &R_body_to_world,
				  const matrix::Vector3f &p_c_b,
				  float total_mass);

	void setPresetTraj(const matrix::Vector3f qv_error, const matrix::Vector3f qv_error_dot);
	void setPresetTrajParas(float PresetTraj_l, float PresetTraj_w, float PresetTraj_epsilon, float PresetTraj_k);

	/**
	 * Set a new desired attitude.
	 * @param qd desired vehicle attitude setpoint
	 * @param yawspeed_setpoint yaw feed-forward angular rate in world frame [rad/s]
	 */
	void setAttitudeSetpoint(const matrix::Quatf &qd, float yawspeed_setpoint);

	/** Adapt attitude setpoint after estimator heading/reference reset. */
	void adaptAttitudeSetpoint(const matrix::Quatf &q_delta);

	/**
	 * Run one control cycle.
	 * @param q current attitude quaternion
	 * @param rate measured body angular rate [rad/s]
	 * @param dt control period [s]
	 * @param landed true when vehicle is landed; controller outputs are reset to zero
	 * @param torque output body torque command [N*m before external normalization]
	 * @param rates_sp output body rate setpoint for logging/compatibility [rad/s]
	 * @param pos_z local position z in PX4 NED frame [m]
	 */
	void update(const matrix::Quatf &q,
		    const matrix::Vector3f &rate,
		    float dt,
		    bool landed,
		    matrix::Vector3f &torque,
		    matrix::Vector3f &rates_sp,
		    float pos_z);

	void getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status) const;

private:
	void runAttitudeControl(const matrix::Quatf &q,
				const matrix::Vector3f &rate,
				float dt,
				matrix::Vector3f &torque,
				matrix::Vector3f &rates_sp,
				float pos_z);

	void UsrAttitudeESO(matrix::Vector3f bm_omega, matrix::Vector3f u, float dt);

	using Vector4f = matrix::Vector<float, 4>;
	using Matrix3f = matrix::SquareMatrix<float, 3>;

	matrix::Vector3f _rate_limit;

	matrix::Quatf _attitude_setpoint_q;
	float _yawspeed_setpoint{0.f};

	Vector4f _attitude_setpoint_q_last;
	Matrix3f _eye_3;

	struct usr_ESO {
		matrix::Vector3f xi;
		matrix::Vector3f xi_dot;
		matrix::Vector3f delta_esti;
		matrix::Vector3f delta_esti_dot;
		matrix::Vector3f L;
		float EPSI{1.f};
		float c1{0.3f};
		float c2{0.5f};
	} _usr_eso;

	struct usr_att_controller {
		Matrix3f lambda_q;
		Matrix3f K_q;
	} _controller_param;

	matrix::Vector3f _tau;
	matrix::SquareMatrix<float, 3> _I_b;
	matrix::SquareMatrix<float, 3> _I_b_inve;

	// COM offset coupling compensation
	matrix::Dcmf _R_body_to_world{};
	matrix::Vector3f _p_c_b{};
	float _total_mass{0.f};
	matrix::Vector3f _delta_omega_comp{};

	struct preset_traj {
		float l{1.f};
		float c{1.f};
		float k{1.f};
		float b{0.f};
		float w{0.1f};
		float epsilon{0.05f};

		matrix::Vector3f time;
		matrix::Vector3f time_last;
		matrix::Vector3f ed;
		matrix::Vector3f ed_dot;
		matrix::Vector3f ed_ddot;
		matrix::Vector3f e0;
		matrix::Vector3f ev0;
		matrix::Vector3f e0_last;
		matrix::Vector3f ev0_last;
	} _preset_traj;
};
