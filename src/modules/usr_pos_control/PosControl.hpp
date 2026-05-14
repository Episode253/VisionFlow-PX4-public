/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PosControl.hpp
 *Author:Huazi Cao
 */

#pragma once
#include "ControlMath.hpp"
#include <string.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/pos_helper.h> // load log
#include <float.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>
#include <cmath>
#include <px4_platform_common/log.h> // 包含 PX4 的日志宏定义


#define G	        9.8066f
#define SUM_max_x       0.25f                                          //x方向水平最大积分
#define SUM_max_y       0.25f
#define SUM_max_z       0.25f
#define Pi              3.1415926f


struct PositionControlStates {
        matrix::Vector3f position;
        matrix::Vector3f velocity;
        matrix::Vector3f acceleration;
        float yaw;
};

// User define control parameters
struct ControlParas{                                               //控制器
     matrix::Matrix<float,3, 3> bm_lambda_p;
     matrix::Matrix<float,3, 3> bm_Kv;
     matrix::Matrix<float,3, 3> bm_Kp;
};

// User define structure.
struct Autopilot{
     matrix::Vector3f pos_err;                                    //位置误差
     matrix::Vector3f vel_err;                                    //速度误差
     matrix::Vector3f slide_mode;                                  //滑模
     matrix::Vector3f zp;                                        // 误差
     matrix::Vector3f zp_dot;                                        // 误差
     matrix::Vector3f pos_err_integ;                              //误差积分
     matrix::Vector3f f_iusl;                                     //输出推力
     matrix::Vector3f bm_sv;
     matrix::Vector3f bm_vr;
     matrix::Vector3f bm_vr_deriv;                               //vr导
};


class PosControl
{
public:

        PosControl() = default;
        ~PosControl() = default;

        /**
         * Set the controller gains and ESO gains
         * Find detials from my paper
         */
        void setMasses(const float m_multrotor, const float m_manipulator){_Mb = m_multrotor; _SUM_mi = m_manipulator;};

        // void setControlParas(const matrix::Vector3f &bm_lambda_p, const matrix::Vector3f &bm_K_v);

        void setControlParas(const matrix::Vector3f &bm_lambda_p, const matrix::Vector3f &bm_K_p);

        // void setESOParas(const matrix::Vector3f &ESO_v);

        void setCESOParas(const matrix::Vector3f &CESO_l1, const matrix::Vector3f &CESO_l2, const float &CESO_EPSI,const float &CESO_c1,const float &CESO_c2);

        float PositionCESO_function_g(float error,float l);

        void resetESO();
        void resetPresetTraj();

        void setPresetTraj(const matrix::Vector3f e0, const matrix::Vector3f ev0);

        void setPresetTrajParas(const float &PresetTraj_l, const float &PresetTraj_w,const float &PresetTraj_epsilon,const float &PresetTraj_k);



        /**
         * Set the maximum velocity to execute with feed forward and position control
         * @param vel_horizontal horizontal velocity limit
         * @param vel_up upwards velocity limit
         * @param vel_down downwards velocity limit
         */
        void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);

        /**
         * Set the minimum and maximum collective normalized thrust [0,1] that can be output by the controller
         * @param min minimum thrust e.g. 0.1 or 0
         * @param max maximum thrust e.g. 0.9 or 1
         */
        void setThrustLimits(const float min, const float max);

        /**
         * Set the maximum tilt angle in radians the output attitude is allowed to have
         * @param tilt angle in radians from level orientation
         */
        void setTiltLimit(const float tilt) { _lim_tilt = tilt; }

        /**
         * Set the normalized hover thrust
         * @param thrust [0.1, 0.9] with which the vehicle hovers not acelerating down or up with level orientation
         */
        void setHoverThrust(const float hover_thrust) { _hover_thrust = math::constrain(hover_thrust, 0.1f, 0.9f); }

        /**
         * Update the hover thrust without immediately affecting the output
         * by adjusting the integrator. This prevents propagating the dynamics
         * of the hover thrust signal directly to the output of the controller.
         */
        void updateHoverThrust(const float hover_thrust_new);

        /**
         * Pass the current vehicle state to the controller
         * @param PositionControlStates structure
         */
        void setState(const PositionControlStates &states);

        /**
         * Pass the desired setpoints
         * Note: NAN value means no feed forward/leave state uncontrolled if there's no higher order setpoint.
         * @param setpoint a vehicle_local_position_setpoint_s structure
         */
        void setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint);

        /**
         * Apply P-position and PID-velocity controller that updates the member
         * thrust, yaw- and yawspeed-setpoints.
         * @see _thr_sp
         * @see _yaw_sp
         * @see _yawspeed_sp
         * @param dt time in seconds since last iteration
         * @return true if update succeeded and output setpoint is executable, false if not
         */
        bool update(const float dt);

        /**
         * Set the integral term in xy to 0.
         * @see _vel_int
         */
        void resetIntegral() { _autopilot.pos_err_integ.setZero(); }

        /**
         * Get the controllers output local position setpoint
         * These setpoints are the ones which were executed on including PID output and feed-forward.
         * The acceleration or thrust setpoints can be used for attitude control.
         * @param local_position_setpoint reference to struct to fill up
         */
        void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;

        /**
         * Get the controllers output attitude setpoint
         * This attitude setpoint was generated from the resulting acceleration setpoint after position and velocity control.
         * It needs to be executed by the attitude controller to achieve velocity and position tracking.
         * @param attitude_setpoint reference to struct to fill up
         */
        void getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const;



private:
        bool _updateSuccessful();

        void _positionControl(const float dt); ///< Position control. Details can be found in my paper.

        // Gains
        ControlParas _contolParas;

        //uORB data for logger
        uORB::Publication<pos_helper_s>	pos_helper_pub{ORB_ID(pos_helper)};     //输出uorb

        // User defined varibles
        Autopilot _autopilot;

        // Masses of multirotor and manipulator
        float _Mb;
        float _SUM_mi;

        matrix::Vector3f g = matrix::Vector3f(0, 0,G);

        // User defined log
        pos_helper_s pos_helper {};

        // Limits
        float _lim_vel_horizontal{}; ///< Horizontal velocity limit with feed forward and position control
        float _lim_vel_up{}; ///< Upwards velocity limit with feed forward and position control
        float _lim_vel_down{}; ///< Downwards velocity limit with feed forward and position control
        float _lim_thr_min{}; ///< Minimum collective thrust allowed as output [-1,0] e.g. -0.9
        float _lim_thr_max{}; ///< Maximum collective thrust allowed as output [-1,0] e.g. -0.1
        float _lim_tilt{}; ///< Maximum tilt from level the output attitude is allowed to have

        float _hover_thrust{}; ///< Thrust [0.1, 0.9] with which the vehicle hovers not accelerating down or up with level orientation

        // States
        matrix::Vector3f _pos; /**< current position */
        matrix::Vector3f _vel; /**< current velocity */
        matrix::Vector3f _vel_dot; /**< velocity derivative (replacement for acceleration estimate) */
        matrix::Vector3f _vel_int; /**< integral term of the velocity controller */
        float _yaw{}; /**< current heading */

        // Setpoints
        matrix::Vector3f _pos_sp; /**< desired position */
        matrix::Vector3f _vel_sp; /**< desired velocity */
        matrix::Vector3f _acc_sp; /**< desired acceleration */
        matrix::Vector3f _acc_cal; /**< desired acceleration */
        matrix::Vector3f _thr_sp; /**< desired thrust */
        float _yaw_sp{}; /**< desired heading */
        float _yawspeed_sp{}; /** desired yaw-speed */
        matrix::Vector3f delta_v;

        //set preset trajectory
        matrix::Vector3f pos_error0;
        matrix::Vector3f vel_error0;


        struct usr_ESO
        {
        float EPSI;
        float c1;
        float c2;

        matrix::Vector3f xi1;
        matrix::Vector3f xi2;
        matrix::Vector3f vel_est;
        matrix::Vector3f delta_est;
        matrix::Vector3f L1;
        matrix::Vector3f L2;

        } _usr_eso;


        //set preset trajectory
        matrix::Vector3f _pos_sp_last;

        struct preset_traj
        {
        float l;
        float c;
        float k;
        float b;
        float w;
        float epsilon;

        matrix::Vector3f time;  //记录每个通道的时间，需要3维向量
        matrix::Vector3f time_last;  //记录每个通道的上一次的时间，需要3维向量

        matrix::Vector3f ed;
        matrix::Vector3f ed_dot;
        matrix::Vector3f ed_ddot;
        matrix::Vector3f e0;
        matrix::Vector3f ev0;
        matrix::Vector3f e0_last;
        matrix::Vector3f ev0_last;
        } _preset_traj;


       bool _is_initialized = false; // 标记是否已初始化


        void PositionCESO(matrix::Vector3f pos_in,matrix::Vector3f f,float dt);


};
