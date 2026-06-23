/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file usr_pos_control_params.c
 * User define position controller parameters.
 *
 * @author Huazi Cao <caohuazi@westlake.edu.cn>
 */

/**
 * mass of UAV
 *
 * @min 0.0
 * @max 40.0
 * @decimal 2
 * @group  User Position Control
 */

PARAM_DEFINE_FLOAT(USR_MASS_UAV, 5.0f);


/**
 * mass of manipulator
 *
 * @min 0.0
 * @max 40.0
 * @decimal 2
 * @group  User Position Control
 */

PARAM_DEFINE_FLOAT(USR_MASS_MANIP, 2.3f);


/**
 * position control parameters of x
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group  User Position Control
 */

PARAM_DEFINE_FLOAT(USR_ESO_V_L1_X, 0.0f);

/**
 * position control parameters of x
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group  User Position Control
 */

PARAM_DEFINE_FLOAT(USR_ESO_V_L1_Y, 0.0f);

/**
 * position control parameters of x
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group  User Position Control
 */

PARAM_DEFINE_FLOAT(USR_ESO_V_L1_Z, 0.0f);

/**
 * gains of ESO,  gazebo 0 feihu 2
 *
 * @min 0.0
 * @max 40.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_ESO_V_L2_X, 0.0f);
/**
 * gains of ESO,  gazebo 0 feihu 2
 *
 * @min 0.0
 * @max 40.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_ESO_V_L2_Y, 0.0f);
/**
 * gains of ESO,  gazebo 0 feihu 2
 *
 * @min 0.0
 * @max 40.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_ESO_V_L2_Z, 0.0f);

/**
 * gains of ESO,  gazebo 0 feihu 2
 *
 * @min 0.0
 * @max 40.0
 * @decimal 2
 * @group  User Position Control
 */

PARAM_DEFINE_FLOAT(USR_ESO_V_EPSI, 1.0f);

/**
 * gains of ESO,  gazebo 0 feihu 2
 *
 * @min 0.0
 * @max 40.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_ESO_V_C1, 0.3f);

/**
 * gains of ESO,  gazebo 0 feihu 2
 *
 * @min 0.0
 * @max 40.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_ESO_V_C2, 0.5f);

/**
 * gains of ESO,  gazebo 0 feihu 2
 *
 * @min 0.0
 * @max 40.0
 * @decimal 2
 * @group  User Position Control
 */

// PARAM_DEFINE_FLOAT(USR_LAMBDA_P_X, 1.5f);

PARAM_DEFINE_FLOAT(USR_LAMBDA_P_X, 2.0f);


/**
 * position control parameters of y
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */
// PARAM_DEFINE_FLOAT(USR_LAMBDA_P_Y, 1.5f);

PARAM_DEFINE_FLOAT(USR_LAMBDA_P_Y, 2.0f);

/**
 * position control parameters of z
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */

PARAM_DEFINE_FLOAT(USR_LAMBDA_P_Z, 2.0f);


/**
 * position control parameters of p_x
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */

PARAM_DEFINE_FLOAT(USR_K_P_X, 2.0f);

/**
 * position control parameters of p_y
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_K_P_Y, 2.0f);


/**
 * position control parameters of p_z
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_K_P_Z, 1.0f);


/**
 * preset tracking error parameters
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_PRESET_L, 1.0f);

/**
 * preset tracking error parameters
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_PRESET_W, 0.1f);


/**
 * preset tracking error parameters
 *
 * @min 0.0
 * @max 0.5
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_PRESET_EPSI, 0.05f);

/**
 * preset tracking error parameters
 *
 * @min 0.0
 * @max 1
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_PRESET_K, 1.0f);


/**
 * Minimum thrust in auto thrust control
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @unit norm
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group User Position Control
 */


PARAM_DEFINE_FLOAT(USR_THR_MIN, 0.12f);

/**
 * Hover thrust
 *
 * Vertical thrust required to hover.
 * This value is mapped to center stick for manual throttle control.
 * With this value set to the thrust required to hover, transition
 * from manual to Altitude or Position mode while hovering will occur with the
 * throttle stick near center, which is then interpreted as (near)
 * zero demand for vertical speed.
 *
 * This parameter is also important for the landing detection to work correctly.
 *
 * @unit norm
 * @min 0.1
 * @max 0.8
 * @decimal 2
 * @increment 0.01
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_THR_HOVER, 0.65f);

/**
 * Hover thrust source selector
 *
 * Set false to use the fixed parameter USR_THR_HOVER
 * Set true to use the value computed by the hover thrust estimator
 *
 * @boolean
 * @group User Position Control
 */
PARAM_DEFINE_INT32(USR_USE_HTE, 1);

/**
 * Thrust curve in Manual Mode
 *
 * This parameter defines how the throttle stick input is mapped to commanded thrust
 * in Manual/Stabilized flight mode.
 *
 * In case the default is used ('Rescale to hover thrust'), the stick input is linearly
 * rescaled, such that a centered stick corresponds to the hover throttle (see USR_THR_HOVER).
 *
 * Select 'No Rescale' to directly map the stick 1:1 to the output. This can be useful
 * in case the hover thrust is very low and the default would lead to too much distortion
 * (e.g. if hover thrust is set to 20%, 80% of the upper thrust range is squeezed into the
 * upper half of the stick range).
 *
 * Note: In case USR_THR_HOVER is set to 50%, the modes 0 and 1 are the same.
 *
 * @value 0 Rescale to hover thrust
 * @value 1 No Rescale
 * @group User Position Control
 */
PARAM_DEFINE_INT32(USR_THR_CURVE, 0);

/**
 * Maximum thrust in auto thrust control
 *
 * Limit max allowed thrust
 *
 * @unit norm
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.01
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_THR_MAX, 1.0f);

/**
 * Minimum manual thrust
 *
 * Minimum vertical thrust. It's recommended to set it > 0 to avoid free fall with zero thrust.
 * With MC_AIRMODE set to 1, this can safely be set to 0.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_MANTHR_MIN, 0.08f);

/**
 * Maximum vertical ascent velocity
 *
 * Maximum vertical velocity in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL).
 *
 * @unit m/s
 * @min 0.5
 * @max 8.0
 * @decimal 1
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_Z_VEL_MAX_UP, 0.5f);

/**
 * Maximum vertical descent velocity
 *
 * Maximum vertical velocity in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL).
 *
 * @unit m/s
 * @min 0.5
 * @max 4.0
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_Z_VEL_MAX_DN, 0.5f);

/**
 * Maximum horizontal velocity in mission
 *
 * Horizontal velocity used when flying autonomously in e.g. Missions, RTL, Goto.
 *
 * @unit m/s
 * @min 3.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_XY_CRUISE, 3.0f);

/**
 * Maximum horizontal error allowed by the trajectory generator
 *
 * The integration speed of the trajectory setpoint is linearly
 * reduced with the horizontal position tracking error. When the
 * error is above this parameter, the integration of the
 * trajectory is stopped to wait for the drone.
 *
 * This value can be adjusted depending on the tracking
 * capabilities of the vehicle.
 *
 * @min 0.1
 * @max 10.0
 * @decimal 1
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_XY_ERR_MAX, 2.0f);

/**
 * Maximum horizontal velocity setpoint for manual controlled mode
 *
 * If velocity setpoint larger than USR_XY_VEL_MAX is set, then
 * the setpoint will be capped to USR_XY_VEL_MAX
 *
 * @unit m/s
 * @min 3.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_VEL_MANUAL, 3f);

/**
 * Maximum horizontal velocity
 *
 * Maximum horizontal velocity in AUTO mode. If higher speeds
 * are commanded in a mission they will be capped to this velocity.
 *
 * @unit m/s
 * @min 0.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_XY_VEL_MAX, 1.0f);

/**
 * Maximum tilt angle in air
 *
 * Limits maximum tilt in AUTO and POSCTRL modes during flight.
 *
 * @unit deg
 * @min 20.0
 * @max 89.0
 * @decimal 1
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_TILTMAX_AIR, 45.0f);

/**
 * Maximum tilt during landing
 *
 * Limits maximum tilt angle on landing.
 *
 * @unit deg
 * @min 10.0
 * @max 89.0
 * @decimal 1
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_TILTMAX_LND, 12.0f);

/**
 * Landing descend rate
 *
 * @unit m/s
 * @min 0.6
 * @decimal 1
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_LAND_SPEED, 0.7f);

/**
 * Enable user assisted descent speed for autonomous land routine.
 *
 * When enabled, descent speed will be:
 * stick full up - 0
 * stick centered - USR_LAND_SPEED
 * stick full down - 2 * USR_LAND_SPEED
 *
 * @min 0
 * @max 1
 * @value 0 Fixed descent speed of USR_LAND_SPEED
 * @value 1 User assisted descent speed
 */
PARAM_DEFINE_INT32(USR_LAND_RC_HELP, 0);

/**
 * Takeoff climb rate
 *
 * @unit m/s
 * @min 1
 * @max 5
 * @decimal 2
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_TKO_SPEED, 1.5f);

/**
 * Maximal tilt angle in manual or altitude mode
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_MAN_TILT_MAX, 35.0f);

/**
 * Max manual yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 400
 * @decimal 1
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_MAN_Y_MAX, 150.0f);

/**
 * Manual yaw rate input filter time constant
 *
 * Setting this parameter to 0 disables the filter
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_MAN_Y_TAU, 0.08f);

/**
 * Deadzone of sticks where position hold is enabled
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_HOLD_DZ, 0.1f);

/**
 * Maximum horizontal velocity for which position hold is enabled (use 0 to disable check)
 *
 * @unit m/s
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_HOLD_MAX_XY, 0.8f);

/**
 * Maximum vertical velocity for which position hold is enabled (use 0 to disable check)
 *
 * @unit m/s
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_HOLD_MAX_Z, 0.6f);

/**
 * Low pass filter cut freq. for numerical velocity derivative
 *
 * @unit Hz
 * @min 0.0
 * @max 10
 * @decimal 2
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_VELD_LP, 5.0f);

/**
 * Maximum horizontal acceleration for auto mode and for manual mode
 *
 * USR_POS_MODE
 * 1 just deceleration
 * 3 acceleration and deceleration
 * 4 just acceleration
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_ACC_HOR_MAX, 5.0f);

/**
 * Acceleration for auto and for manual
 *
 * Note: In manual, this parameter is only used in USR_POS_MODE 4.
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group User Position Control
 */

PARAM_DEFINE_FLOAT(USR_ACC_HOR, 3.0f);

/**
 * Maximum vertical acceleration in velocity controlled modes upward
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_ACC_UP_MAX, 4.0f);

/**
 * Maximum vertical acceleration in velocity controlled modes down
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_ACC_DOWN_MAX, 3.0f);

/**
 * Maximum jerk limit
 *
 * Limit the maximum jerk of the vehicle (how fast the acceleration can change).
 * A lower value leads to smoother vehicle motions, but it also limits its
 * agility (how fast it can change directions or break).
 *
 * Setting this to the maximum value essentially disables the limit.
 *
 * Note: This is only used when USR_POS_MODE is set to a smoothing mode 3 or 4.
 *
 * @unit m/s^3
 * @min 0.5
 * @max 500.0
 * @increment 1
 * @decimal 2
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_JERK_MAX, 8.0f);

/**
 * Jerk limit in auto mode
 *
 * Limit the maximum jerk of the vehicle (how fast the acceleration can change).
 * A lower value leads to smoother vehicle motions, but it also limits its
 * agility.
 *
 * @unit m/s^3
 * @min 1.0
 * @max 80.0
 * @increment 1
 * @decimal 1
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_JERK_AUTO, 4.0f);

/**
 * Altitude control mode.
 *
 * Set to 0 to control height relative to the earth frame origin. This origin may move up and down in
 * flight due to sensor drift.
 * Set to 1 to control height relative to estimated distance to ground. The vehicle will move up and down
 * with terrain height variation. Requires a distance to ground sensor. The height controller will
 * revert to using height above origin if the distance to ground estimate becomes invalid as indicated
 * by the local_position.distance_bottom_valid message being false.
 * Set to 2 to control height relative to ground (requires a distance sensor) when stationary and relative
 * to earth frame origin when moving horizontally.
 * The speed threshold is controlled by the USR_HOLD_MAX_XY parameter.
 *
 * @min 0
 * @max 2
 * @value 0 Altitude following
 * @value 1 Terrain following
 * @value 2 Terrain hold
 * @group User Position Control
 */
PARAM_DEFINE_INT32(USR_ALT_MODE, 0);

/**
 * Manual position control stick exponential curve sensitivity
 *
 * The higher the value the less sensitivity the stick has around zero
 * while still reaching the maximum value with full stick deflection.
 *
 * 0 Purely linear input curve (default)
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_XY_MAN_EXPO, 0.6f);

/**
 * Manual control stick vertical exponential curve
 *
 * The higher the value the less sensitivity the stick has around zero
 * while still reaching the maximum value with full stick deflection.
 *
 * 0 Purely linear input curve (default)
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_Z_MAN_EXPO, 0.6f);

/**
 * Manual control stick yaw rotation exponential curve
 *
 * The higher the value the less sensitivity the stick has around zero
 * while still reaching the maximum value with full stick deflection.
 *
 * 0 Purely linear input curve (default)
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_YAW_EXPO, 0.6f);

/**
 * Max yaw rate in auto mode
 *
 * Limit the rate of change of the yaw setpoint in autonomous mode
 * to avoid large control output and mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(USR_YAWRAUTO_MAX, 45.0f);

/**
 * Altitude for 1. step of slow landing (descend)
 *
 * Below this altitude descending velocity gets limited to a value
 * between "USR_Z_VEL_MAX_DN" and "USR_LAND_SPEED"
 * Value needs to be higher than "USR_LAND_ALT2"
 *
 * @unit m
 * @min 0
 * @max 122
 * @decimal 1
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_LAND_ALT1, 10.0f);

/**
 * Altitude for 2. step of slow landing (landing)
 *
 * Below this altitude descending velocity gets
 * limited to "USR_LAND_SPEED".
 * Value needs to be lower than "USR_LAND_ALT1"
 *
 * @unit m
 * @min 0
 * @max 122
 * @decimal 1
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_LAND_ALT2, 5.0f);

/**
 * Position control smooth takeoff ramp time constant
 *
 * Increasing this value will make automatic and manual takeoff slower.
 * If it's too slow the drone might scratch the ground and tip over.
 * A time constant of 0 disables the ramp
 *
 * @min 0
 * @max 5
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_TKO_RAMP_T, 3.0f);

/**
 * Manual-Position control sub-mode
 *
 * The supported sub-modes are:
 * 0 Simple position control where sticks map directly to velocity setpoints
 *   without smoothing. Useful for velocity control tuning.
 * 3 Smooth position control with maximum acceleration and jerk limits based on
 *   jerk optimized trajectory generator (different algorithm than 1).
 * 4 Smooth position control where sticks map to acceleration and there's a virtual brake drag
 *
 * @value 0 Simple position control
 * @value 3 Smooth position control (Jerk optimized)
 * @value 4 Acceleration based input
 * @group User Position Control
 */
PARAM_DEFINE_INT32(USR_POS_MODE, 4);

/**
 * Enforced delay between arming and takeoff
 *
 * For altitude controlled modes the time from arming the motors until
 * a takeoff is possible gets forced to be at least USR_SPOOLUP_TIME seconds
 * to ensure the motors and propellers can sppol up and reach idle speed before
 * getting commanded to spin faster. This delay is particularly useful for vehicles
 * with slow motor spin-up e.g. because of large propellers.
 *
 * @min 0
 * @max 10
 * @unit s
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_SPOOLUP_TIME, 1.0f);

/**
 * Yaw mode.
 *
 * Specifies the heading in Auto.
 *
 * @min 0
 * @max 4
 * @value 0 towards waypoint
 * @value 1 towards home
 * @value 2 away from home
 * @value 3 along trajectory
 * @value 4 towards waypoint (yaw first)
 * @group Mission
 */
PARAM_DEFINE_INT32(USR_YAW_MODE, 0);

/**
 * Responsiveness
 *
 * Changes the overall responsiveness of the vehicle.
 * The higher the value, the faster the vehicle will react.
 *
 * If set to a value greater than zero, other parameters are automatically set (such as
 * the acceleration or jerk limits).
 * If set to a negative value, the existing individual parameters are used.
 *
 * @min -1
 * @max 1
 * @decimal 2
 * @increment 0.05
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_VEHICLE_RESP, -0.4f);

/**
 * Overall Horizonal Velocity Limit
 *
 * If set to a value greater than zero, other parameters are automatically set (such as
 * USR_XY_VEL_MAX or USR_VEL_MANUAL).
 * If set to a negative value, the existing individual parameters are used.
 *
 * @min -20
 * @max 20
 * @decimal 1
 * @increment 1
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_XY_VEL_ALL, 1.0f);

/**
 * Overall Vertical Velocity Limit
 *
 * If set to a value greater than zero, other parameters are automatically set (such as
 * USR_Z_VEL_MAX_UP or USR_LAND_SPEED).
 * If set to a negative value, the existing individual parameters are used.
 *
 * @min -3
 * @max 8
 * @decimal 1
 * @increment 0.5
 * @group User Position Control
 */
PARAM_DEFINE_FLOAT(USR_Z_VEL_ALL, 1.0f);
