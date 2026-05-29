#pragma once

#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>

#include <lib/mathlib/math/filter/AlphaFilter.hpp>

#include <uORB/topics/battery_status.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

#include <Att_control.hpp>

#include <drivers/drv_hrt.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace time_literals;

class UserAttitudeControl : public ModuleBase, public ModuleParams,
	public px4::WorkItem
{
public:
	static Descriptor desc;

	UserAttitudeControl();
	~UserAttitudeControl() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	void parameters_updated();

	float throttle_curve(float throttle_stick_input);

	void generate_attitude_setpoint(const matrix::Quatf &q, float dt, bool reset_yaw_sp);

	void update_vehicle_status();
	void update_landed_state();
	void update_landing_gear();
	void publish_torque_thrust_setpoint(const vehicle_attitude_s &v_att);

	Att_Control _attitude_control;

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _v_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};

	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _landing_gear_sub{ORB_ID(landing_gear)};
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::PublicationMulti<rate_ctrl_status_s> _controller_status_pub{ORB_ID(rate_ctrl_status)};

	uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};

	bool _actuators_0_circuit_breaker_enabled{false};

	vehicle_control_mode_s _vehicle_control_mode{};
	vehicle_status_s _vehicle_status{};

	float _battery_status_scale{0.0f};

	perf_counter_t _loop_perf{nullptr};

	float _thrust_sp{0.0f};

	hrt_abstime _last_run{0};
	hrt_abstime _last_attitude_setpoint{0};

	int8_t _landing_gear{landing_gear_s::GEAR_DOWN};

	manual_control_setpoint_s _manual_control_setpoint{};

	matrix::Vector3f _thrust_setpoint_body{0.f, 0.f, 0.f};

	float _man_yaw_sp{0.f};
	float _man_tilt_max{0.f};
	AlphaFilter<float> _man_x_input_filter;
	AlphaFilter<float> _man_y_input_filter;

	bool _landed{true};
	bool _maybe_landed{true};
	bool _reset_yaw_sp{true};
	bool _vehicle_type_rotary_wing{true};
	bool _vtol_in_transition_mode{false};
	bool _vtol_tailsitter{false};
	bool _spooled_up{false};

	uint8_t _quat_reset_counter{0};

	matrix::Vector3f _torque{0.f, 0.f, 0.f};
	float pos_z{0.f};


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::USR_TAU_COE>) _param_usr_tau_coe,

		(ParamFloat<px4::params::USR_I_XX>) _param_usr_i_xx,
		(ParamFloat<px4::params::USR_I_YY>) _param_usr_i_yy,
		(ParamFloat<px4::params::USR_I_ZZ>) _param_usr_i_zz,
		(ParamFloat<px4::params::USR_I_XY>) _param_usr_i_xy,
		(ParamFloat<px4::params::USR_I_XZ>) _param_usr_i_xz,
		(ParamFloat<px4::params::USR_I_YZ>) _param_usr_i_yz,


		(ParamFloat<px4::params::USR_LAMBDA_Q_X>) _param_usr_lambda_q_x,
		(ParamFloat<px4::params::USR_LAMBDA_Q_Y>) _param_usr_lambda_q_y,
		(ParamFloat<px4::params::USR_LAMBDA_Q_Z>) _param_usr_lambda_q_z,

		(ParamFloat<px4::params::USR_K_Q_X>) _param_usr_k_q_x,
		(ParamFloat<px4::params::USR_K_Q_Y>) _param_usr_k_q_y,
		(ParamFloat<px4::params::USR_K_Q_Z>) _param_usr_k_q_z,

		(ParamFloat<px4::params::USR_A_PRESET_L>) _param_PresetTraj_l,
		(ParamFloat<px4::params::USR_A_PRESET_W>) _param_PresetTraj_w,
		(ParamFloat<px4::params::USR_A_PRESET_EP>) _param_PresetTraj_epsilon,
		(ParamFloat<px4::params::USR_A_PRESET_K>) _param_PresetTraj_k,

		(ParamFloat<px4::params::USR_ESO_L_X>) _param_usr_eso_l_x,
		(ParamFloat<px4::params::USR_ESO_L_Y>) _param_usr_eso_l_y,
		(ParamFloat<px4::params::USR_ESO_L_Z>) _param_usr_eso_l_z,
		(ParamFloat<px4::params::USR_ESO_EPSI>) _param_usr_eso_epsi,
		(ParamFloat<px4::params::USR_ESO_C1>) _param_usr_eso_c1,
		(ParamFloat<px4::params::USR_ESO_C2>) _param_usr_eso_c2,

		(ParamFloat<px4::params::USR_ROLLRATE_MAX>) _param_usr_rollrate_max,
		(ParamFloat<px4::params::USR_PITRATE_MAX>) _param_usr_pitchrate_max,
		(ParamFloat<px4::params::USR_YAWRATE_MAX>) _param_usr_yawrate_max,

		(ParamFloat<px4::params::USR_MAN_Y_MAX>) _param_usr_man_y_max,

		(ParamInt<px4::params::USR_AIRMODE>) _param_usr_airmode,
		(ParamFloat<px4::params::USR_MAN_TILT_TAU>) _param_usr_man_tilt_tau,
		(ParamFloat<px4::params::USR_MAN_TILT_MAX>) _param_usr_man_tilt_max,
		(ParamFloat<px4::params::USR_MANTHR_MIN>) _param_usr_manthr_min,
		(ParamFloat<px4::params::USR_THR_MAX>) _param_usr_thr_max,
		(ParamFloat<px4::params::USR_THR_HOVER>) _param_usr_thr_hover,
		(ParamInt<px4::params::USR_THR_CURVE>) _param_usr_thr_curve,
		(ParamBool<px4::params::USR_BAT_SCALE_EN>) _param_usr_bat_scale_en,

		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time,
		(ParamInt<px4::params::CBRK_RATE_CTRL>) _param_cbrk_rate_ctrl
	)
};
