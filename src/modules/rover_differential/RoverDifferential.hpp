#pragma once

// PX4 includes
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// Library includes
#include <math.h>

// uORB includes
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>

// Local includes
#include "DifferentialActControl/DifferentialActControl.hpp"
#include "DifferentialRateControl/DifferentialRateControl.hpp"
#include "DifferentialAttControl/DifferentialAttControl.hpp"
#include "DifferentialSpeedControl/DifferentialSpeedControl.hpp"
#include "DifferentialPosControl/DifferentialPosControl.hpp"
#include "DifferentialDriveModes/DifferentialAutoMode/DifferentialAutoMode.hpp"
#include "DifferentialDriveModes/DifferentialManualMode/DifferentialManualMode.hpp"
#include "DifferentialDriveModes/DifferentialOffboardMode/DifferentialOffboardMode.hpp"

class RoverDifferential : public ModuleBase, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	/**
	 * @brief Constructor for RoverDifferential
	 */
	RoverDifferential();
	~RoverDifferential() override = default;

	/** @see ModuleBase */
	static Descriptor desc;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	void Run() override;

	/**
	 * @brief Generate rover setpoints from supported PX4 internal modes
	 */
	void generateSetpoints();

	/**
	 * @brief Update the controllers
	 */
	void updateControllers();

	/**
	 * @brief Check proper parameter setup for the controllers
	 *
	 * Modifies:
	 *
	 *   - _sanity_checks_passed: true if checks for all active controllers pass
	 */
	void runSanityChecks();

	/**
	 * @brief Reset controllers and manual mode variables.
	 */
	void reset();

	// uORB subscriptions
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	vehicle_control_mode_s _vehicle_control_mode{};

	// Class instances
	DifferentialActControl   _differential_act_control{this};
	DifferentialRateControl  _differential_rate_control{this};
	DifferentialAttControl   _differential_att_control{this};
	DifferentialSpeedControl   _differential_speed_control{this};
	DifferentialPosControl   _differential_pos_control{this};
	DifferentialAutoMode	 _auto_mode{this};
	DifferentialManualMode 	 _manual_mode{this};
	DifferentialOffboardMode _offboard_mode{this};

	// Variables
	bool _sanity_checks_passed{true}; // True if checks for all active controllers pass
	bool _was_armed{false}; // True if the vehicle was armed before the last reset
};
