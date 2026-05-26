/*****************************************************************************/

/**
* @file pos_control.cpp.cpp
* Author:Huazi Cao
*/

#include "PosControl.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;
using namespace time_literals;


void PosControl::setControlParas(const matrix::Vector3f &bm_lambda_p, const matrix::Vector3f &bm_K_p)
{
    _contolParas.bm_lambda_p(0,0) = bm_lambda_p(0);
    _contolParas.bm_lambda_p(1,1) = bm_lambda_p(1);
    _contolParas.bm_lambda_p(2,2) = bm_lambda_p(2);
    _contolParas.bm_Kp(0,0) = bm_K_p(0);
    _contolParas.bm_Kp(1,1) = bm_K_p(1);
    _contolParas.bm_Kp(2,2) = bm_K_p(2);
}


void PosControl::setCESOParas(const matrix::Vector3f &CESO_l1, const matrix::Vector3f &CESO_l2,const float &CESO_EPSI,const float &CESO_c1,const float &CESO_c2)
{
    _usr_eso.L1= CESO_l1;
    _usr_eso.L2= CESO_l2;
    _usr_eso.EPSI= CESO_EPSI;
    _usr_eso.c1= CESO_c1;
    _usr_eso.c2= CESO_c2;
}

void PosControl::setPresetTrajParas(const float &PresetTraj_l, const float &PresetTraj_w,const float &PresetTraj_epsilon,const float &PresetTraj_k)
{
     _preset_traj.l = PresetTraj_l;
    _preset_traj.w = PresetTraj_w;
    _preset_traj.epsilon = PresetTraj_epsilon;
    _preset_traj.k = PresetTraj_k;

}


// set preset trajectory
void PosControl::setPresetTraj(const matrix::Vector3f ep, const matrix::Vector3f ep_dot)
{

    float absolute_time = hrt_absolute_time()*1.e-6f;

    ControlMath::setZeroIfNanVector3f(_preset_traj.e0_last);
    ControlMath::setZeroIfNanVector3f(_preset_traj.ev0_last);

    for(int i = 0; i < 3; i++){
        if( !PX4_ISFINITE( _pos_sp_last(i))){
        _pos_sp_last(i) = _pos_sp(i) + _preset_traj.epsilon; // 由于_attitude_setpoint_q是四元数，我们只关注矢量部分
        }
    };

    for (int i = 0; i < 3; i++){
     _preset_traj.e0(i) = _preset_traj.w *ep(i);
     _preset_traj.ev0(i) = _preset_traj.w * ep_dot(i);

         if ( fabs(_pos_sp_last(i) - _pos_sp(i)) >= _preset_traj.epsilon) {
         _preset_traj.time(i) = 0.f;  //指令不断地在变化， preset_trajectory不断地被更新，时间不断地设置为0
         _preset_traj.time_last(i) = absolute_time;
         _preset_traj.e0_last(i) = _preset_traj.w *ep(i);
         _preset_traj.ev0_last(i) = _preset_traj.w *ep_dot(i);
         }
         else
         {
        _preset_traj.time(i) = absolute_time - _preset_traj.time_last(i);  // 从指令没变化时，开始的时间
        _preset_traj.e0(i) = _preset_traj.e0_last(i);   // e0为初始时刻的值
        _preset_traj.ev0(i) = _preset_traj.ev0_last(i);
         }

        _preset_traj.b = _preset_traj.l*_preset_traj.e0(i) + _preset_traj.ev0(i);

        _preset_traj.c = abs(_preset_traj.b)/(2.0f) + _preset_traj.epsilon;

        _preset_traj.ed(i) = _preset_traj.e0(i)*exp(-_preset_traj.l* _preset_traj.time(i))
        +_preset_traj.b/_preset_traj.c*(1-exp(-_preset_traj.c*_preset_traj.time(i)))*exp(-_preset_traj.l*_preset_traj.time(i));

        _preset_traj.ed_dot(i) = -_preset_traj.l*_preset_traj.ed(i)+_preset_traj.b*exp(-(_preset_traj.l+ _preset_traj.c)*_preset_traj.time(i));

        _preset_traj.ed_ddot(i) = -_preset_traj.l*_preset_traj.ed_dot(i)-
        _preset_traj.b*(_preset_traj.l+_preset_traj.c)*exp(-(_preset_traj.l+ _preset_traj.c)*_preset_traj.time(i));

        _pos_sp_last(i) = _pos_sp(i); // 保存这一时刻的值
    }

}


void PosControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
        _lim_vel_horizontal = vel_horizontal;
        _lim_vel_up = vel_up;
        _lim_vel_down = vel_down;
}

void PosControl::setThrustLimits(const float min, const float max)
{
        // make sure there's always enough thrust vector length to infer the attitude
        _lim_thr_min = math::max(min, 10e-4f);
        _lim_thr_max = max;
}

void PosControl::updateHoverThrust(const float hover_thrust_new)
{
        _vel_int(2) += (hover_thrust_new - _hover_thrust) * (CONSTANTS_ONE_G / hover_thrust_new);
        setHoverThrust(hover_thrust_new);
}

void PosControl::setState(const PositionControlStates &states)
{
        _pos = states.position;
        _vel = states.velocity;
        _yaw = states.yaw;
        _vel_dot = states.acceleration;
}

void PosControl::setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
        _pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
        _vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
        _acc_sp = Vector3f(setpoint.acceleration);
        _yaw_sp = setpoint.yaw;
        _yawspeed_sp = setpoint.yawspeed;
}

// TODO:I will rewrite this function
bool PosControl::update(const float dt)
{
        // x and y input setpoints always have to come in pairs
        const bool valid = (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)))
                           && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)))
                           && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

        // if(!PX4_ISFINITE(_pos_sp(0))&&!PX4_ISFINITE(_pos_sp(1))){
	// 	if(PX4_ISFINITE(_vel_sp(0)) && PX4_ISFINITE(_vel_sp(1))){
	// 		if (PX4_ISFINITE(_pos(0))&&PX4_ISFINITE(_pos(2))){
	// 			_pos_sp =  _pos + _vel_sp*dt;
	// 		}
	// 	}
	// }
        // if only velocity cmd is inputed, we integret velocity cmd to obtain position cmd.
        // This procedure is necessary, because the whole position controller requires position cmd.
        // unless noly velocity control is required by user

       for(int i = 0; i < 3; i++){
                if ((!PX4_ISFINITE(_pos_sp(i))) && PX4_ISFINITE(_vel_sp(i)) &&PX4_ISFINITE(_pos(i))) {
                        // pos_sp is NAN, vel_sp is not NAN
                        _pos_sp(i) = _pos(i) + _vel_sp(i)*dt;
                }
       }

        _positionControl(dt);

        _yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
        _yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control

//        PX4_WARN("valid = %d\n", valid);
//        PX4_WARN("_updateSuccessful = %d\n", _updateSuccessful());
        return valid && _updateSuccessful();
}

void PosControl::_positionControl(const float dt)
{
    _autopilot.pos_err = _pos - _pos_sp;


    // make sure there are no NAN elements for further reference while constraining
    ControlMath::setZeroIfNanVector3f(_autopilot.pos_err);
    // check _vel_sp
    ControlMath::setZeroIfNanVector3f(_vel_sp);
   // Constrain velocity in x,y,z-direction.
   _vel_sp(0) = math::constrain(_vel_sp(0), -_lim_vel_horizontal, _lim_vel_horizontal);
   _vel_sp(1) = math::constrain(_vel_sp(1), -_lim_vel_horizontal, _lim_vel_horizontal);
   _vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);


   _autopilot.vel_err   = _vel - _vel_sp; // velocity error
    ControlMath::setZeroIfNanVector3f(_autopilot.vel_err);

// 控制律
// TODO:Interface for position ESO should be writen
    delta_v = _usr_eso.delta_est;//Interface for position ESO
    ControlMath::setZeroIfNanVector3f(delta_v);

    setPresetTraj(_autopilot.pos_err, _autopilot.vel_err);

    _autopilot.zp = _autopilot.pos_err - _preset_traj.k*_preset_traj.ed;
    _autopilot.zp_dot = _autopilot.vel_err - _preset_traj.k*_preset_traj.ed_dot;
    _autopilot.slide_mode = _autopilot.zp_dot + _contolParas.bm_lambda_p*_autopilot.zp;

    //control law
    _autopilot.f_iusl = (_Mb+_SUM_mi)*(_contolParas.bm_Kp*_autopilot.slide_mode + g + delta_v +
    _contolParas.bm_lambda_p*_autopilot.zp_dot - _preset_traj.k* _preset_traj.ed_ddot ) ;

//      PX4_INFO("_acc_sp x= %.4f,_acc_sp y= %.4f,_acc_sp z= %.4f",
//     static_cast<double>( _acc_sp(0)),
//     static_cast<double>( _acc_sp(1)),
//     static_cast<double>( _acc_sp(2)));



     Vector3f u_v = - (1.0f/(_Mb+_SUM_mi) ) * _autopilot.f_iusl + g;

    PositionCESO(_pos,u_v, dt);

   //  convert force to acceleration
    _acc_cal = - _autopilot.f_iusl/(_Mb+_SUM_mi)+g;


    // Assume standard acceleration due to gravity in vertical direction for attitude generation
    Vector3f body_z = Vector3f(-_acc_cal(0), -_acc_cal(1), G).normalized();

    ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);

    /* Scale thrust assuming hover thrust produces standard gravity
     * from PX4:        float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
     * old version:     float collective_thrust = _acc_sp(2) * (_hover_thrust / G) - _hover_thrust;
     * newest version:  float collective_thrust = (_acc_sp(2) - G) * (_Mb + _SUM_mi);
     * result: PX4 is right. Only the codes from PX4 can run well.
    */


    float collective_thrust = _acc_cal(2) * ( _hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;

    // Project thrust to planned body attitude
    collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
    collective_thrust = math::min(collective_thrust, -_lim_thr_min);
    _thr_sp = body_z * collective_thrust;


    // Saturate maximal vertical thrust
    _thr_sp(2) = math::max(_thr_sp(2) , -_lim_thr_max);

    // Get allowed horizontal thrust after prioritizing vertical control
    const float thrust_max_squared = _lim_thr_max * _lim_thr_max;
    const float thrust_z_squared = _thr_sp(2) * _thr_sp(2) ;
    const float thrust_max_xy_squared = thrust_max_squared - thrust_z_squared;
    float thrust_max_xy = 0;

    if (thrust_max_xy_squared > 0) {
        thrust_max_xy = sqrtf(thrust_max_xy_squared);
    }

    // Saturate thrust in horizontal direction
    matrix::Vector2f thrust_sp_xy(_thr_sp(0),_thr_sp(1));

    float thrust_sp_xy_norm = thrust_sp_xy.norm();

    //Vector3f _thr_sp;
    if (thrust_sp_xy_norm > thrust_max_xy) {
        _thr_sp(0) = thrust_sp_xy(0) / thrust_sp_xy_norm * thrust_max_xy;
        _thr_sp(1) = thrust_sp_xy(1) / thrust_sp_xy_norm * thrust_max_xy;
    }


    //log
    pos_helper.timestamp = hrt_absolute_time();

    pos_helper.ed[0] = delta_v(0);
    pos_helper.ed[1] = delta_v(1);
    pos_helper.ed[2] = delta_v(2);

    pos_helper_pub.publish(pos_helper);
}

bool PosControl::_updateSuccessful()
{
        bool valid = true;

        // For each controlled state the estimate has to be valid
        for (int i = 0; i <= 2; i++) {
                if (PX4_ISFINITE(_pos_sp(i))) {
                        valid = valid && PX4_ISFINITE(_pos(i));
                }

                if (PX4_ISFINITE(_vel_sp(i))) {
                        valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
                }
        }

        // There has to be a valid output accleration and thrust setpoint otherwise there was no
        // setpoint-state pair for each axis that can get controlled
        valid = valid && PX4_ISFINITE(_acc_cal(0)) && PX4_ISFINITE(_acc_cal(1)) && PX4_ISFINITE(_acc_cal(2));
        valid = valid && PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1)) && PX4_ISFINITE(_thr_sp(2));
        return valid;
}



 void PosControl::PositionCESO(matrix::Vector3f pos_in, matrix::Vector3f u,float dt)
 {
        // initialize _usr_eso.pos_est
    if (!_is_initialized) {
        _usr_eso.xi1 = pos_in; // 设置初始值为 pos_in
        _is_initialized = true; // 标记初始化已完成
    }

    // check
    ControlMath::setZeroIfNanVector3f(_usr_eso.xi1);
    ControlMath::setZeroIfNanVector3f(_usr_eso.xi2); // the internal observer states in CESO
    ControlMath::setZeroIfNanVector3f(_usr_eso.vel_est);
    ControlMath::setZeroIfNanVector3f(_usr_eso.delta_est);
    ControlMath::setZeroIfNanVector3f(pos_in);
    ControlMath::setZeroIfNanVector3f(u);
    dt = PX4_ISFINITE(dt)? dt:0.0f;
    for (int i = 0; i < 3; i++){

        float error1=pos_in(i)-_usr_eso.xi1(i);
        float function_g1=PositionCESO_function_g(error1,_usr_eso.L1(i));

        float xi1_dot=function_g1/_usr_eso.EPSI;
        _usr_eso.vel_est(i)= function_g1/_usr_eso.EPSI;
        _usr_eso.xi1(i)=_usr_eso.xi1(i)+xi1_dot*dt;

        float error2=_usr_eso.vel_est(i)-_usr_eso.xi2(i);
        float function_g2=PositionCESO_function_g(error2,_usr_eso.L2(i));

        float xi2_dot=function_g2/_usr_eso.EPSI+u(i);

        _usr_eso.delta_est(i)= function_g2/_usr_eso.EPSI;
        _usr_eso.xi2(i)=_usr_eso.xi2(i)+xi2_dot*dt;

    }
 }


float  PosControl::PositionCESO_function_g(float error,float l)
{
        float function_g=l*error*(exp(error)+exp(-error))/(_usr_eso.c1*(exp(error)+exp(-error))+_usr_eso.c2);
        return function_g;
}

void PosControl::resetESO()
{
    _usr_eso.delta_est={0.0f,0.0f,0.0f};
    _usr_eso.xi2 = {0.0f,0.0f,0.0f};

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



void PosControl::resetPresetTraj()
 {
    _pos_sp_last(0) = NAN; //第一个时刻它是没值的,设置为NAN,从1开始，因为我们考虑的是矢量部分.
    _pos_sp_last(1) = NAN;
    _pos_sp_last(2) = NAN;

    _preset_traj.time_last(0) = hrt_absolute_time()/ 1.e6f; // 换算成秒
    _preset_traj.time_last(1) = hrt_absolute_time()/ 1.e6f; // 换算成秒
    _preset_traj.time_last(2) = hrt_absolute_time()/ 1.e6f; // 换算成秒


    for(int i = 0; i < 3; i++){
    _preset_traj.e0_last(i) = NAN;
    _preset_traj.ev0_last(i) = NAN;
    }

 }


void PosControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
        ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
        attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
