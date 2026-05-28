#pragma once

#include "PosControl.hpp"
#include "Takeoff.hpp"

#include <drivers/drv_hrt.h>
#include <lib/controllib/blocks.hpp>
#include <lib/hysteresis/hysteresis.h>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRateYaw.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

using namespace time_literals;

class PregmePositionControl : public ModuleBase, public control::SuperBlock,
	public ModuleParams, public px4::ScheduledWorkItem
{
public:
	PregmePositionControl(bool vtol = false);
	~PregmePositionControl() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static ModuleBase::Descriptor desc;

	bool init();

private:
	void Run() override;

	int parameters_update(bool force);
	PositionControlStates set_vehicle_states(const vehicle_local_position_s &local_pos, float dt);
	void failsafe(const hrt_abstime &now, vehicle_local_position_setpoint_s &setpoint,
		      const PositionControlStates &states, bool warn);
	void reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint);
	void reset_setpoint_to_nan(trajectory_setpoint_s &setpoint);

	void limit_thrust_during_landing(vehicle_attitude_setpoint_s &setpoint, const TakeoffState takeoff_state, float dt);
	float slew_thrust_z(float target_thrust_z, float dt, float slew_rate);

	Takeoff _takeoff;

	uORB::PublicationData<takeoff_status_s> _takeoff_status_pub{ORB_ID(takeoff_status)};
	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_constraints_sub{ORB_ID(vehicle_constraints)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	hrt_abstime _time_stamp_last_loop{0};
	hrt_abstime _last_warn{0};

	trajectory_setpoint_s _setpoint{};
	vehicle_control_mode_s _vehicle_control_mode{};
	vehicle_constraints_s _vehicle_constraints{
		.timestamp = 0,
		.speed_up = NAN,
		.speed_down = NAN,
		.want_takeoff = false,
	};
	vehicle_land_detected_s _vehicle_land_detected{
		.timestamp = 0,
		.ground_contact = true,
		.maybe_landed = true,
		.landed = true,
	};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::PREGME_MUAV>) _param_pregme_mass_uav,
		(ParamFloat<px4::params::PREGME_MMANIP>) _param_pregme_mass_manipulator,
		(ParamFloat<px4::params::PREGME_LPX>) _param_pregme_lambda_p_x,
		(ParamFloat<px4::params::PREGME_LPY>) _param_pregme_lambda_p_y,
		(ParamFloat<px4::params::PREGME_LPZ>) _param_pregme_lambda_p_z,
		(ParamFloat<px4::params::PREGME_KPX>) _param_pregme_k_p_x,
		(ParamFloat<px4::params::PREGME_KPY>) _param_pregme_k_p_y,
		(ParamFloat<px4::params::PREGME_KPZ>) _param_pregme_k_p_z,
		(ParamFloat<px4::params::PREGME_EV1X>) _param_eso_v_l1_x,
		(ParamFloat<px4::params::PREGME_EV1Y>) _param_eso_v_l1_y,
		(ParamFloat<px4::params::PREGME_EV1Z>) _param_eso_v_l1_z,
		(ParamFloat<px4::params::PREGME_EV2X>) _param_eso_v_l2_x,
		(ParamFloat<px4::params::PREGME_EV2Y>) _param_eso_v_l2_y,
		(ParamFloat<px4::params::PREGME_EV2Z>) _param_eso_v_l2_z,
		(ParamFloat<px4::params::PREGME_EVEPS>) _param_eso_v_epsi,
		(ParamFloat<px4::params::PREGME_EVC1>) _param_eso_v_c1,
		(ParamFloat<px4::params::PREGME_EVC2>) _param_eso_v_c2,
		(ParamFloat<px4::params::PREGME_PSL>) _param_PresetTraj_l,
		(ParamFloat<px4::params::PREGME_PSW>) _param_PresetTraj_w,
		(ParamFloat<px4::params::PREGME_PSEPS>) _param_PresetTraj_epsilon,
		(ParamFloat<px4::params::PREGME_PSK>) _param_PresetTraj_k,
		(ParamFloat<px4::params::PREGME_XYVMAX>) _param_pregme_xy_vel_max,
		(ParamFloat<px4::params::PREGME_ZVUP>) _param_pregme_z_vel_max_up,
		(ParamFloat<px4::params::PREGME_ZVDN>) _param_pregme_z_vel_max_dn,
		(ParamFloat<px4::params::PREGME_TILTAIR>) _param_pregme_tiltmax_air,
		(ParamFloat<px4::params::PREGME_THRHOV>) _param_pregme_thr_hover,
		(ParamBool<px4::params::PREGME_UHTE>) _param_pregme_use_hte,
		(ParamFloat<px4::params::PREGME_SPUPTIME>) _param_pregme_spoolup_time,
		(ParamFloat<px4::params::PREGME_TKORAMP>) _param_pregme_tko_ramp_t,
		(ParamFloat<px4::params::PREGME_TKOSPD>) _param_pregme_tko_speed,
		(ParamFloat<px4::params::PREGME_LANDSPD>) _param_pregme_land_speed,
		(ParamFloat<px4::params::PREGME_VMAN>) _param_pregme_vel_manual,
		(ParamFloat<px4::params::PREGME_XYCRS>) _param_pregme_xy_cruise,
		(ParamFloat<px4::params::PREGME_LALT2>) _param_pregme_land_alt2,
		(ParamInt<px4::params::PREGME_POSMOD>) _param_pregme_pos_mode,
		(ParamInt<px4::params::PREGME_ALTMOD>) _param_pregme_alt_mode,
		(ParamFloat<px4::params::PREGME_TILTLND>) _param_pregme_tiltmax_lnd,
		(ParamFloat<px4::params::PREGME_THRMIN>) _param_pregme_thr_min,
		(ParamFloat<px4::params::PREGME_THRMAX>) _param_pregme_thr_max,
		(ParamFloat<px4::params::PREGME_VRESP>) _param_pregme_vehicle_resp,
		(ParamFloat<px4::params::PREGME_AHOR>) _param_pregme_acc_hor,
		(ParamFloat<px4::params::PREGME_ADNMAX>) _param_pregme_acc_down_max,
		(ParamFloat<px4::params::PREGME_AUPMAX>) _param_pregme_acc_up_max,
		(ParamFloat<px4::params::PREGME_AHMAX>) _param_pregme_acc_hor_max,
		(ParamFloat<px4::params::PREGME_JAUTO>) _param_pregme_jerk_auto,
		(ParamFloat<px4::params::PREGME_JMAX>) _param_pregme_jerk_max,
		(ParamFloat<px4::params::PREGME_MYMAX>) _param_pregme_man_y_max,
		(ParamFloat<px4::params::PREGME_MYTAU>) _param_pregme_man_y_tau,
		(ParamFloat<px4::params::PREGME_XYVALL>) _param_pregme_xy_vel_all,
		(ParamFloat<px4::params::PREGME_ZVALL>) _param_pregme_z_vel_all,
		(ParamFloat<px4::params::PREGME_VELD_LP>) _param_pregme_veld_lp
	);

	struct VelocityDerivativeFilter {
		float cutoff_hz{5.f};
		float prev_input{NAN};
		float estimate{0.f};

		void setCutoff(float hz)
		{
			cutoff_hz = math::max(hz, 0.f);
		}

		void reset()
		{
			prev_input = NAN;
			estimate = 0.f;
		}

		float update(float input, float dt)
		{
			if (!PX4_ISFINITE(input) || !PX4_ISFINITE(dt) || dt <= 0.f) {
				return estimate;
			}

			if (!PX4_ISFINITE(prev_input)) {
				prev_input = input;
				estimate = 0.f;
				return estimate;
			}

			const float raw_derivative = (input - prev_input) / dt;
			prev_input = input;

			if (cutoff_hz <= 0.f) {
				estimate = raw_derivative;
				return estimate;
			}

			const float alpha = expf(-2.f * Pi * cutoff_hz * dt);
			estimate = alpha * estimate + (1.f - alpha) * raw_derivative;
			return estimate;
		}
	};

	VelocityDerivativeFilter _vel_x_deriv;
	VelocityDerivativeFilter _vel_y_deriv;
	VelocityDerivativeFilter _vel_z_deriv;

	PosControl _control;

	bool _in_failsafe{false};
	bool _hover_thrust_initialized{false};

	float _last_thrust_body_z{0.f};
	bool _last_thrust_body_z_valid{false};

	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;
	static constexpr uint64_t LOITER_TIME_BEFORE_DESCEND = 200_ms;
	static constexpr float ALTITUDE_THRESHOLD = 0.3f;
	static constexpr float MAX_SAFE_TILT_DEG = 89.f;

	static constexpr float THRUST_SLEW_TAKEOFF = 0.25f;	// 0 -> hover in about 1.0~1.5 s for typical hover thrust
	static constexpr float THRUST_SLEW_LANDING = 0.65f;	// smooth but not too slow after ground contact
	static constexpr float THRUST_SLEW_FLIGHT = 8.0f;	// almost transparent during normal flight
	static constexpr float TAKEOFF_SPEED_SP_MIN = 0.01f;	// avoid the previous 0.15 m/s step at ramp start
	static constexpr float TAKEOFF_ALTITUDE_STEP_MIN = 0.05f;	// avoid an abrupt 0.3 m position step at ramp start
	static constexpr float THRUST_ZERO_EPS = 0.002f;

	systemlib::Hysteresis _failsafe_land_hysteresis{false};
	SlewRate<float> _tilt_limit_slew_rate;

	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _heading_reset_counter{0};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle time")};
};
