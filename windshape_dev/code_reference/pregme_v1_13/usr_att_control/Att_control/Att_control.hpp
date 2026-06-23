#pragma once

#include <math.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>
#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>
#include <uORB/topics/rate_ctrl_status.h>


//log
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
// #include <uORB/topics/att_helper.h> // load log

class Att_Control
{
public:
        Att_Control() = default;
        ~Att_Control() = default;

        void Controllerinit();

        /**
         * Set attitude controller and attitude ESO gains
         */
        // void setControllerGain(const matrix::Vector3f &lambda_omega,const matrix::Vector3f &usr_k_omega,const matrix::Vector3f &usr_eso_gain,const float &usr_Kq);


        void setControllerGain(const matrix::Vector3f &lambda_omega,const matrix::Vector3f &usr_k_q);

        /**
         * Set attitude controller and attitude ESO gains
         */
        void setInertiaMatrix(const matrix::SquareMatrix<float, 3> &Ib){_I_b = Ib;_I_b_inve = matrix::inv(Ib);}

        /**
         * Set hard limit for output rate setpoints
         * @param rate_limit [rad/s] 3D vector containing limits for roll, pitch, yaw
         */
        void setRateLimit(const matrix::Vector3f &rate_limit) { _rate_limit = rate_limit; }
        /**
         * Set the integral item of the ESO as zero
         */
        void resetESO();
        void resetPresetTraj();


        void setCESOParas(const matrix::Vector3f &CESO_l,const float &CESO_EPSI,const float &CESO_c1,const float &CESO_c2);

        float CESO_function_g(float error,float l);

        void setPresetTraj(const matrix::Vector3f e0, const matrix::Vector3f ev0);

        void setPresetTrajParas(const float &PresetTraj_l, const float &PresetTraj_w,const float &PresetTraj_epsilon,const float &PresetTraj_k);



        /**
         * Set a new attitude setpoint replacing the one tracked before
         * @param qd desired vehicle attitude setpoint
         * @param yawspeed_setpoint [rad/s] yaw feed forward angular rate in world frame
         */
        void setAttitudeSetpoint(const matrix::Quatf &qd, const float yawspeed_setpoint) { _attitude_setpoint_q = qd; _attitude_setpoint_q.normalize(); _yawspeed_setpoint = yawspeed_setpoint; }
        /**
         * Adjust last known attitude setpoint by a delta rotation
         * Optional use to avoid glitches when attitude estimate reference e.g. heading changes.
         * @param q_delta delta rotation to apply
         */
        void adaptAttitudeSetpoint(const matrix::Quatf &q_delta) { _attitude_setpoint_q = q_delta * _attitude_setpoint_q; }

        /**
         * Run one control loop cycle calculation
         * @param q estimation of the current vehicle attitude unit quaternion
         * @return [rad/s] body frame 3D angular rate setpoint vector to be executed by the rate controller
         */
        void update(const matrix::Quatf &q, const matrix::Vector3f &rate,
                          const float &dt, const bool &landed, matrix::Vector3f &torque,matrix::Vector3f &rates_sp,const float &pos_z);
        /**
         * Set saturation status
         * @param status message from mixer reporting about saturation
         */
        void getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status);
private:

        /**
         * Set hard limit for output rate setpoints
         * @param rate_limit [rad/s] 3D vector containing limits for roll, pitch, yaw
         */
        void runAttitudeControl(const matrix::Quatf &q, const matrix::Vector3f &rate,
                                const float &dt, matrix::Vector3f& torque,matrix::Vector3f &rates_sp,  const float &pos_z);
        /**
         * Run attitude ESO
         * @param angular rate[rad/s] output tau[N*M]
         */
        void UsrAttitudeESO(matrix::Vector3f bm_omega,matrix::Vector3f u,float dt);

        matrix::Vector3f _rate_limit;
        float _yaw_w{0.f}; ///< yaw weight [0,1] to deprioritize caompared to roll and pitch

        matrix::Quatf _attitude_setpoint_q; ///< latest known attitude setpoint e.g. from position control
        float _yawspeed_setpoint{0.f}; ///< latest known yawspeed feed-forward setpoint
        //uORB data for logger
        // uORB::Publication<att_helper_s>	att_helper_pub{ORB_ID(att_helper)};     //输出uorb

        typedef matrix::Vector<float, 4> Vector4f;


        //set preset trajectory
        Vector4f _attitude_setpoint_q_last;

        matrix::Matrix<float,3, 3> eye_3;


        struct usr_ESO
        {
        matrix::Vector3f xi;                    //ESO
        matrix::Vector3f xi_dot;               //ESO

        matrix::Vector3f delta_esti;
        matrix::Vector3f delta_esti_dot;
        matrix::Vector3f L;

        float EPSI;
        float c1;
        float c2;
        } _usr_eso;


        struct usr_att_controller                        //controller
        {
        float k_q;
        matrix::Matrix<float,3, 3> lambda_q;
        matrix::Matrix<float,3, 3> K_w;
        matrix::Matrix<float,3, 3> K_q;
        } _controller_param;

        matrix::Vector3f _tau;
        matrix::Matrix<float,3, 3> _I_b;
        matrix::SquareMatrix<float, 3> _I_b_inve;

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



        // User defined log
        // att_helper_s _att_helper {};

};
