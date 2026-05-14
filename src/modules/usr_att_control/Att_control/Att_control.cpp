#include "Att_control.hpp"

using namespace matrix;

void addIfNotNan(float &setpoint, const float addition)
{
	if (PX4_ISFINITE(setpoint) && PX4_ISFINITE(addition)) {
		// No NAN, add to the setpoint
		setpoint += addition;

	} else if (!PX4_ISFINITE(setpoint)) {
		// Setpoint NAN, take addition
		setpoint = addition;
	}

	// Addition is NAN or both are NAN, nothing to do
}

void addIfNotNanVector3f(Vector3f &setpoint, const Vector3f &addition)
{
	for (int i = 0; i < 3; i++) {
		addIfNotNan(setpoint(i), addition(i));
	}
}

void setZeroIfNanVector3f(Vector3f &vector)
{
	// Adding zero vector overwrites elements that are NaN with zero
	addIfNotNanVector3f(vector, Vector3f());
}

void Att_Control::setControllerGain(const matrix::Vector3f &lambda_omega,const matrix::Vector3f &usr_k_q)
{
     _controller_param.lambda_q(0,0)=lambda_omega(0);
     _controller_param.lambda_q(1,1)=lambda_omega(1);
     _controller_param.lambda_q(2,2)=lambda_omega(2);

     _controller_param.K_q(0,0)=usr_k_q(0);
     _controller_param.K_q(1,1)=usr_k_q(1);
     _controller_param.K_q(2,2)=usr_k_q(2);
}

void Att_Control::setCESOParas(const matrix::Vector3f &CESO_l,const float &CESO_EPSI,const float &CESO_c1,const float &CESO_c2)
{
    _usr_eso.L= CESO_l;
    _usr_eso.EPSI= CESO_EPSI;
    _usr_eso.c1= CESO_c1;
    _usr_eso.c2= CESO_c2;
}

void Att_Control::setPresetTrajParas(const float &PresetTraj_l, const float &PresetTraj_w,const float &PresetTraj_epsilon,const float &PresetTraj_k)
{
    _preset_traj.l = PresetTraj_l;
    _preset_traj.w = PresetTraj_w;
    _preset_traj.epsilon = PresetTraj_epsilon;
    _preset_traj.k = PresetTraj_k;

}

// set preset trajectory
void Att_Control::setPresetTraj(const matrix::Vector3f qv_error, const matrix::Vector3f qv_error_dot)
{

    float absolute_time = hrt_absolute_time()/1.e6f;

    setZeroIfNanVector3f(_preset_traj.e0_last);
    setZeroIfNanVector3f(_preset_traj.ev0_last);

    for(int i = 1; i < 4; i++){
        if( !PX4_ISFINITE(_attitude_setpoint_q_last(i))){
        _attitude_setpoint_q_last(i) = _attitude_setpoint_q(i) + _preset_traj.epsilon; // 由于_attitude_setpoint_q是四元数，我们只关注矢量部分
        }
    };

    for (int i = 0; i < 3; i++){
     _preset_traj.e0(i) = _preset_traj.w *qv_error(i);
     _preset_traj.ev0(i) = _preset_traj.w * qv_error_dot(i);

    // _attitude_setpoint_q 比较的是矢量部分,所以要加1
         if ( fabs(_attitude_setpoint_q_last(i+1) - _attitude_setpoint_q(i+1)) >= _preset_traj.epsilon) {
         _preset_traj.time(i)= 0.f;  //指令不断地在变化， preset_trajectory不断地被更新，时间不断地设置为0
         _preset_traj.time_last(i) = absolute_time;

         _preset_traj.e0_last(i) = _preset_traj.w *qv_error(i);
         _preset_traj.ev0_last(i) = _preset_traj.w * qv_error_dot(i);
         }
         else
         {
        _preset_traj.time(i) = absolute_time - _preset_traj.time_last(i);  // 从指令没变化时，开始的时间

        _preset_traj.e0(i) = _preset_traj.e0_last(i);   // e0为初始时刻的值
        _preset_traj.ev0(i) = _preset_traj.ev0_last(i);

         }

        _preset_traj.b = _preset_traj.l*_preset_traj.e0(i) + _preset_traj.ev0(i);
        _preset_traj.c = abs(_preset_traj.b)/(0.1f) + _preset_traj.epsilon;

        _preset_traj.ed(i) = _preset_traj.e0(i)*exp(-_preset_traj.l* _preset_traj.time(i))
        +_preset_traj.b/_preset_traj.c*(1-exp(-_preset_traj.c*_preset_traj.time(i)))*exp(-_preset_traj.l*_preset_traj.time(i));

        _preset_traj.ed_dot(i) = -_preset_traj.l*_preset_traj.ed(i)+_preset_traj.b*exp(-(_preset_traj.l+ _preset_traj.c)*_preset_traj.time(i));

        _preset_traj.ed_ddot(i) = -_preset_traj.l*_preset_traj.ed_dot(i)-
        _preset_traj.b*(_preset_traj.l+_preset_traj.c)*exp(-(_preset_traj.l+ _preset_traj.c)*_preset_traj.time(i));

        _attitude_setpoint_q_last(i+1) = _attitude_setpoint_q(i+1); // 保存这一时刻的值
    }

}



void Att_Control:: update(const matrix::Quatf &q, const matrix::Vector3f &rate,
                          const float &dt, const bool &landed, Vector3f& torque,matrix::Vector3f &rates_sp,const float &pos_z)
{
    runAttitudeControl(q,rate,dt,torque,rates_sp,pos_z);
}


void  Att_Control::runAttitudeControl(const matrix::Quatf &q, const matrix::Vector3f &rate,
                                      const float &dt, Vector3f& torque,matrix::Vector3f &rates_sp, const float &pos_z)
{

    matrix::Quatf q_error = _attitude_setpoint_q.inversed() * q;

    q_error.normalize();

    Vector3f qv_error;
    qv_error = q_error.imag();
    float q0_error = q_error(0);

    float q_for_skew_symm[9]={0,-qv_error(2),qv_error(1),qv_error(2),0,-qv_error(0),-qv_error(1),qv_error(0),0};
    Matrix<float,3, 3> q_skew_symm(q_for_skew_symm);

    Vector3f omega_r;
    omega_r={0.0f,0.0f,0.0f};
    rates_sp = omega_r;

    // // Feed forward the yaw setpoint rate.
    // // yawspeed_setpoint is the feed forward commanded rotation around the world z-axis,
    // // but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
    // // Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
    // // and multiply it by the yaw setpoint rate (yawspeed_setpoint).
    // // This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
    // // such that it can be added to the rates setpoint.

    if (is_finite(_yawspeed_setpoint)) {
            omega_r += q.inversed().dcm_z() * _yawspeed_setpoint;
    }


    Vector3f omega_error = rate - omega_r;
    float q0_error_dot = -0.5f*(qv_error*omega_error);

    eye_3.identity();

    Matrix<float,3, 3> Q_dot;
    Matrix3f Q = eye_3*q0_error+q_skew_symm;
    Matrix3f Q_inv;
    Q_inv = inv(Q);

    Vector3f qv_error_dot = Q*omega_error*0.5f;
    float qv_error_dot_for_skew_symm[9]={0,-qv_error_dot(2),qv_error_dot(1),qv_error_dot(2),0,-qv_error_dot(0),-qv_error_dot(1),qv_error_dot(0),0};
    Matrix<float,3, 3> qv_error_dot_skew_symm(qv_error_dot_for_skew_symm);
    Q_dot = eye_3*q0_error_dot+qv_error_dot_skew_symm;


    setPresetTraj(qv_error, qv_error_dot);

    Vector3f zq = qv_error - _preset_traj.k * _preset_traj.ed;
    Vector3f zq_dot = qv_error_dot - _preset_traj.k *_preset_traj.ed_dot;
    Vector3f slide_mode_q = zq_dot + _controller_param.lambda_q*zq;

    //    introduce CESO when z-position of quadcopter achieve up to 2m
    if(pos_z < -1.0f){
    setZeroIfNanVector3f(_usr_eso.delta_esti);
    }
    else{
        _usr_eso.delta_esti = {0.0f,0.0f,0.0f};
    }

    // setZeroIfNanVector3f(_usr_eso.delta_esti);



    torque = 2.0f*_I_b*Q_inv*(-_controller_param.K_q*slide_mode_q- 0.5f*Q_dot*omega_error-_controller_param.lambda_q*zq_dot + _preset_traj.k *_preset_traj.ed_ddot)+
    rate%(_I_b*rate) - _I_b *_usr_eso.delta_esti;


    _tau(0) = torque(0);
    _tau(1) = torque(1);
    _tau(2) = torque(2);

    Vector3f u_w=_I_b_inve*(_tau-rate%(_I_b*rate));

    UsrAttitudeESO(rate,u_w, dt);

    _usr_eso.delta_esti(0) = math::constrain(_usr_eso.delta_esti(0), -20.0f,20.0f);
    _usr_eso.delta_esti(1) = math::constrain(_usr_eso.delta_esti(1), -20.0f,20.0f);
    _usr_eso.delta_esti(2) = math::constrain(_usr_eso.delta_esti(2), -20.0f,20.0f);

    // _att_helper.timestamp = hrt_absolute_time();

    // att_helper_pub.publish(_att_helper);
}

void Att_Control::UsrAttitudeESO(matrix::Vector3f bm_omega,matrix::Vector3f u,float dt)
{

    // check
    setZeroIfNanVector3f(_usr_eso.delta_esti);
    setZeroIfNanVector3f(_usr_eso.xi);
    setZeroIfNanVector3f(bm_omega);
    setZeroIfNanVector3f(u);
    dt = PX4_ISFINITE(dt)? dt:0.0f;

  for (int i = 0; i < 3; i++){
        float error = bm_omega(i)-_usr_eso.xi(i);
        float function_g = CESO_function_g(error,_usr_eso.L(i));
        float xi_dot=function_g/_usr_eso.EPSI+u(i);
        _usr_eso.delta_esti(i)= function_g/_usr_eso.EPSI;
        _usr_eso.xi(i)=_usr_eso.xi(i)+xi_dot*dt;
    }

}

void Att_Control::resetESO()
{
    _usr_eso.delta_esti={0.0f,0.0f,0.0f};
    _usr_eso.xi = {0.0f,0.0f,0.0f};
    _usr_eso.xi_dot = {0.0f,0.0f,0.0f};
     _usr_eso.delta_esti_dot = {0.0f,0.0f,0.0f};
    _tau={0.0f,0.0f,0.0f};

}

 void Att_Control::resetPresetTraj()
 {
    _attitude_setpoint_q_last(1) = NAN; //第一个时刻它是没值的,设置为NAN,从1开始，因为我们考虑的是矢量部分.
    _attitude_setpoint_q_last(2) = NAN;
    _attitude_setpoint_q_last(3) = NAN;

    _preset_traj.time_last(0) = hrt_absolute_time()/ 1.e6f; // 换算成秒
    _preset_traj.time_last(1) = hrt_absolute_time()/ 1.e6f; // 换算成秒
    _preset_traj.time_last(2) = hrt_absolute_time()/ 1.e6f; // 换算成秒

    for(int i = 0; i < 3; i++){
    _preset_traj.e0_last(i) = NAN;
    _preset_traj.ev0_last(i) = NAN;
    }

 }

float  Att_Control::CESO_function_g(float error,float l)
{
        float function_g=l*error*(exp(error)+exp(-error))/(_usr_eso.c1*(exp(error)+exp(-error))+_usr_eso.c2);
       return function_g;
}

void Att_Control::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
    rate_ctrl_status.rollspeed_integ = _usr_eso.delta_esti(0);
    rate_ctrl_status.pitchspeed_integ = _usr_eso.delta_esti(1);
    rate_ctrl_status.yawspeed_integ = _usr_eso.delta_esti(2);
}

