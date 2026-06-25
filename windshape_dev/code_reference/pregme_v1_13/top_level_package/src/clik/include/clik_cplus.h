#ifndef _CLIK_CPLUS_
#define _CLIK_CPLUS_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <math.h>
#include <string.h>
#include <Eigen/SVD>
#include "osqp/osqp.h"

#ifndef ROS_ERROR_STREAM
#define ROS_ERROR_STREAM(msg) RCLCPP_ERROR_STREAM(rclcpp::get_logger("clik"), msg)
#endif



// External inputs (root inport signals with default storage)
typedef struct {
  float state[9];                     // '<Root>/state'
  float Pe[3];                        // '<Root>/Pe'
  float Ped[3];                        // '<Root>/Pe'
  float Pe_cmd[3];                    // '<Root>/Pe_cmd'
  float Pe_vcmd[3];                    // '<Root>/Pe_cmd'
  float dtime;                         // "/time step"
  float distance_manip;                // 'distance from now to manipulation point'
  Eigen::Matrix3d rotation_d2i;
} ExtU_CLIK_T;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  double quad_out[3];                  // '<Root>/quad_out'
  double delta_out[3];                 // '<Root>/delta_out'
} ExtY_CLIK_T;

class clik_c_
{
  public:
    clik_c_(){};

    // External inputs
    ExtU_CLIK_T CLIK_U;
    // External outputs
    ExtY_CLIK_T CLIK_Y;
    void clik_solver();
    float xi_min[6],xi_max[6];  // 数组
    double xi_dot_min[6],xi_dot_max[6];// 数组
    float xi_ddot_min[6],xi_ddot_max[6];   // 数组 
  private:
    float d_s_; //权重参数
    float r_s = 0.4;//外
    float r_w = 0.2;//内
    float d_max_ = 3;// 机械臂最大权重值
    float lamda = 0.1; // 优化目标权重
    
};

#endif
