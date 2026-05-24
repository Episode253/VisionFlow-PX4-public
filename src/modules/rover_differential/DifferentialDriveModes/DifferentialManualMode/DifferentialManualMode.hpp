#pragma once

// PX4 includes
#include <px4_platform_common/module_params.h>

// Libraries
#include <math.h>
#include <matrix/matrix/math.hpp>

// uORB includes
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/rover_throttle_setpoint.h>
#include <uORB/topics/rover_steering_setpoint.h>
#include <uORB/topics/rover_rate_setpoint.h>
#include <uORB/topics/rover_attitude_setpoint.h>
#include <uORB/topics/rover_speed_setpoint.h>
#include <uORB/topics/rover_position_setpoint.h>

using namespace matrix;

/**
 * @brief Class for differential manual mode.
 */
class DifferentialManualMode : public ModuleParams
{
public:
	/**
	 * @brief Constructor for DifferentialManualMode.
	 * @param parent The parent ModuleParams object.
	 */
	DifferentialManualMode(ModuleParams *parent);
	~DifferentialManualMode() = default;

	/**
	 * @brief Publish roverThrottleSetpoint and roverSteeringSetpoint from manualControlSetpoint.
	 */
	void manual();

	/**
	 * @brief Generate and publish roverThrottleSetpoint/RoverRateSetpoint from manualControlSetpoint.
	 */
	void acro();

	/**
	 * @brief Generate and publish roverSetpoints from manualControlSetpoint.
	 */
	void stab();

	/**
	 * @brief Generate and publish roverSetpoints from manualControlSetpoint.
	 */
	void position();

	/**
	 * @brief Reset manual mode variables.
	 */
	void reset();

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	// uORB subscriptions
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	// uORB publications
	uORB::Publication<rover_throttle_setpoint_s> _rover_throttle_setpoint_pub{ORB_ID(rover_throttle_setpoint)};
	uORB::Publication<rover_steering_setpoint_s> _rover_steering_setpoint_pub{ORB_ID(rover_steering_setpoint)};
	uORB::Publication<rover_rate_setpoint_s>     _rover_rate_setpoint_pub{ORB_ID(rover_rate_setpoint)};
	uORB::Publication<rover_attitude_setpoint_s> _rover_attitude_setpoint_pub{ORB_ID(rover_attitude_setpoint)};
	uORB::Publication<rover_speed_setpoint_s> _rover_speed_setpoint_pub{ORB_ID(rover_speed_setpoint)};
	uORB::Publication<rover_position_setpoint_s> _rover_position_setpoint_pub{ORB_ID(rover_position_setpoint)};

	// Variables
	Vector2f _pos_ctl_course_direction{NAN, NAN};
	Vector2f _pos_ctl_start_position_ned{NAN, NAN};
	Vector2f _curr_pos_ned{NAN, NAN};
	float _stab_yaw_setpoint{NAN};
	float _vehicle_yaw{NAN};
	float _max_yaw_rate{NAN};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RO_YAW_RATE_LIM>)  _param_ro_yaw_rate_limit,
		(ParamFloat<px4::params::RO_YAW_STICK_DZ>)  _param_ro_yaw_stick_dz,
		(ParamFloat<px4::params::RO_YAW_EXPO>)      _param_ro_yaw_expo,
		(ParamFloat<px4::params::RO_YAW_SUPEXPO>)   _param_ro_yaw_supexpo,
		(ParamFloat<px4::params::RD_YAW_STK_GAIN>)  _param_rd_yaw_stk_gain,
		(ParamFloat<px4::params::PP_LOOKAHD_MAX>)   _param_pp_lookahd_max,
		(ParamFloat<px4::params::RO_SPEED_LIM>)     _param_ro_speed_limit
	)
};
