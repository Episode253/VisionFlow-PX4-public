#ifndef CLIK_MAIN_H
#define CLIK_MAIN_H


#include <rclcpp/rclcpp.hpp>
#include "rclcpp/qos.hpp"
#include <vector>
#include <limits>
#include <memory>
#include <chrono>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
// #include <trac_ik/trac_ik.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <stddef.h>
#include <stdio.h>                // This ert_main.c example uses printf/fflush
#include <mutex>
#include <Eigen/Eigen>
#include <math.h>
#include "mavros_msgs/msg/actuator_control.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "mavros_msgs/srv/message_interval.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "clik/msg/position_pub.hpp"
#include "clik/srv/manipulator_mode.hpp"
#include "clik/srv/traj_out_msg.hpp"
#include "clik/srv/traj_solver_msg.hpp"
#include <string.h>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "log4z.h"
#include "clik/msg/action.hpp"
#include "arm_uav_kinematics.h"
#include <fstream>
#include "sensor_msgs/msg/joint_state.hpp"
#include "mavros_msgs/msg/mavlink.hpp"
#include "geometry_msgs/msg/wrench.hpp"  // 消息类型：包含 force 和 torque
#include <custom_mavlink/mavlink/common/mavlink.h>

#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("clik"), __VA_ARGS__)
#define ROS_WARN_THROTTLE(period, ...) RCLCPP_WARN_THROTTLE(rclcpp::get_logger("clik"), *this->get_clock(), static_cast<int>((period) * 1000.0), __VA_ARGS__)
#define ROS_INFO_THROTTLE(period, ...) RCLCPP_INFO_THROTTLE(rclcpp::get_logger("clik"), *this->get_clock(), static_cast<int>((period) * 1000.0), __VA_ARGS__)
#define ROS_ERROR_STREAM(msg) RCLCPP_ERROR_STREAM(rclcpp::get_logger("clik"), msg)


//#include <std_msgs/Int32.h>

//#include "math_utils.h"

enum m_behavior {IDLE, WAYPOINT_FLIGHT,LAND, HOVER,TAKEOFF,FORWARD,BACKWARD,RIGHT,LEFT,UP,DOWN};


typedef struct {
  double x    = 0;                   
  double y    = 0;                      
  double z    = 0;
  double vx  = 0;
  double vy  = 0;
  double vz  = 0;                                
  double phi  =0;                    
  double theta = 0;                
  double psi  = 0;                    
  double wx   = 0;                 
  double wy   = 0;                    
  double wz   = 0;                     
} flightStateStruct;


typedef struct {
  double x = 0;                   
  double y = 0;                      
  double z = 0;                                           
  double psi = 0;                    
  double xe  = 0;                 
  double ye  = 0;                    
  double ze  = 0; 
  double xe_ned;
  double ye_ned;
  double ze_ned;

  double rolle_ned;
  double pitche_ned;
  double yawe_ned;

  double q_init[6] = {0};
} coordinateIni;

typedef struct {
  double q1  = 0;                   
  double q2  = 0;                      
  double q3  = 0;
  double q4  = 0;
  double q5  = 0;
  double q6  = 0;
  double clik_joint_pos[6]={0};
  double clik_joint_vel[6]={0};                                                   
} manipulatorStruct;
// 机械臂的模式：0 放下， 1 收起， 2 协调。
enum COM_MODE { mod_shrink = 0, mod_prepare = 1, mod_control = 2 ,mod_sleep =3,mod_wait =4,mod_read=5};

typedef struct{
    double time_all = 3.0;
    void get_cmd_from_linear(double time,  Eigen::VectorXd &out, Eigen::VectorXd &in, Eigen::VectorXd &target)
    {
      time = (time > time_all)? time_all:time;
      out = in + time/time_all *(target - in);
    }
   }joint_cmd_struct;



class clikRos : public rclcpp::Node
{
public:
    clikRos();
    ~clikRos(){};

    void mainLoop();

    flightStateStruct flightStateData;
    coordinateIni m_coordinate_contr_ini;

    manipulatorStruct manipulatorData;
    joint_cmd_struct joint_cmd;

    mavros_msgs::msg::RCIn m_rcin_;
    mavros_msgs::msg::RCIn m_rcin_prev_;
 
    mavros_msgs::msg::State current_state;
    COM_MODE manipulator_mode;

private:

    //clik_c_ CLIK_Obj;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub ;       // 【订阅】无人机当前状态 - 来自飞控
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_sub;     // 【订阅】无人机当前位置 坐标系:ENU系
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub;     // 【订阅】无人机当前vel 坐标系:ENU系
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr attitude_sub;     // 【订阅】无人机当前欧拉角 坐标系:ENU系
    rclcpp::Subscription<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_sp_sub;  // 【订阅】无人机真实姿态目标
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rcin_sub;         // 【订阅】遥控器的操纵 
    rclcpp::Subscription<clik::msg::Action>::SharedPtr action_sub;       // 【订阅】off_mission 无人机的导航状态

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr Ti5_arm_EE_sub;   // 订阅机械臂的末端位置- 来自vicon 坐标系 地面绝对坐标系（vicon）
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr Ti5_arm_EE_twist_sub; //末端的速度
    rclcpp::Subscription<mavros_msgs::msg::Mavlink>::SharedPtr mavlink_from_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_pub;       // 发布gripper的串口指令
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr force_pub_;      // 【发布】发布扰动力指令
    
    
    rclcpp::Client<clik::srv::TrajSolverMsg>::SharedPtr traj_solver_client; // 【客户端】轨迹生成
    rclcpp::Client<clik::srv::TrajOutMsg>::SharedPtr traj_out_client;    // 【客户端】轨迹输出
    
    rclcpp::Publisher<clik::msg::PositionPub>::SharedPtr Delta_pub;//【发布】 Delta的位置指令
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr local_pos_pub;// 【发布】 飞机的轨迹指令
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr local_vel_pub;// 【发布】 飞机的速度指令
    rclcpp::Publisher<mavros_msgs::msg::Mavlink>::SharedPtr mavlink_raw_pub_; // 【发布】原始 MAVLink，自定义发送 p_C_B 到 PX4 mavros_gs


    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_ctrl_pub;
    rclcpp::Client<clik::srv::ManipulatorMode>::SharedPtr manipulator_client; // 客户端修改模式
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client; // 客户端 修改飞机的模式
    rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedPtr set_message_interval_client; // 客户端 请求PX4回传ATTITUDE_TARGET

    // ArmIK_Solver ik_solver_; // 声明成员对象

    bool reset_CLIK_flag_;
    bool on_off_manipulator_flag_; // 机械臂收放状态：false放，true收
    bool ground_origin_position_initialized_flag_ ;
    bool coordinate_flag_;// 协调控制状态：false不协调，true协调
    bool reset_coordinate_flag_;// 协调控制状态保存 
    bool stall_manipulater_flag_;// 保持机械臂的模式指令
    bool first_off_manipulator;// 首次进入放机械臂状态
    bool first_on_manipulator;// 首次收sho进入机械臂状态
    bool coordinate_running_flag_;// 协调控制已经进入

    rclcpp::Time last_off_manipulator; // 首次进入放机械臂状态时间
    rclcpp::Time last_on_manipulator; // 首次进入放机械臂状态时间
    rclcpp::Time last_coordinate; // 首次进入协同模式时间
    rclcpp::Time last_time; // 上一步计算时间
    rclcpp::Time now_time; // 这一步计算时间
    rclcpp::Time test_begin;// 测试开始时间
    std::mutex rc_mutex_;
    std_msgs::msg::String gripper_msg; 


    clik::srv::TrajOutMsg::Request    traj_out_request;
    clik::srv::TrajSolverMsg::Request traj_solver_request;
    

    int origin_counter;
    double pe[3];
    Eigen::Vector3d position_EE;//机械臂在地面坐标系（绝对坐标系）上的位置，从vicon上订阅
    Eigen::Vector3d velocity_EE;//机械臂在地面坐标系（绝对坐标系）上的速度，从vicon上订阅

    Eigen::Vector3d dOffset;//机械臂的1状态移动平台相对于飞机的误差
    double yaw_offset;//机械臂相对机体x正方向的误差，该误差与vicon的标定直接相关
    Eigen::Matrix3d DCM_mani2body;


    geometry_msgs::msg::PoseStamped m_ref_origin_; //起飞点信息（地面坐标系原点）
    geometry_msgs::msg::PoseStamped current_local_pos; //当前位置信息
    clik::msg::PositionPub Delts_cont; // 发布delta机械臂的位置指令 
    clik::srv::ManipulatorMode::Request Delta_mode_request; //发布机械臂模式
    mavros_msgs::srv::SetMode::Request offb_set_mode_request; // 飞机的飞行模式，这里要把飞机的模式修改位off-board模式
    geometry_msgs::msg::PoseStamped pose; // 储存飞机的位置和偏航指令
    mavros_msgs::msg::PositionTarget traj_cmd; //发布轨迹
    geometry_msgs::msg::Vector3 force_msg;

    Eigen::Vector3d attitude_sp = Eigen::Vector3d::Zero(); // 储存PX4内部姿态目标
    clik::msg::Action cur_action;

    Eigen::Vector3d point1; // 第一个期望点
    Eigen::Vector3d point2; // 第二个期望点
    Eigen::Vector3d point3; // 第三个期望点
    Eigen::Vector3d point4; // 第四个期望点
    Eigen::Vector3d desired_pE; // 期望的末端位置
    Eigen::Vector3d desired_vE;
    Eigen::Vector3d pE_in_M; // 期望的末端位置在机械臂坐标系下
    Eigen::Matrix3d RE_in_M; // 期望的末端姿态在机械臂坐标系下
    Eigen::VectorXd desired_theta = Eigen::VectorXd::Zero(6);

    Eigen::Vector3d vehicle_velocity_cmd; //无人机速度
    Eigen::Vector3d vehicle_position_cmd; //无人机位置


    sensor_msgs::msg::JointState joint_ctrl_msg; // 发布机械臂关节控制角度

    //回调函数
    void state_obtain(const mavros_msgs::msg::State::ConstSharedPtr &msg);
    void pos_obtain(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void vel_obtain(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg);
    void rcin_obtain(const mavros_msgs::msg::RCIn::ConstSharedPtr& msg);
    void att_obtain(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
    void Ti5_arm_EE_obtain(const geometry_msgs::msg::TransformStamped::ConstSharedPtr& message_holder);
    void Ti5_arm_EE_obtain_twist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg);

    void vehicle_action_callback(const clik::msg::Action::ConstSharedPtr& msg);
    void att_sp_obtain(const mavros_msgs::msg::AttitudeTarget::ConstSharedPtr &msg);
    void debug_array_obtain(const mavros_msgs::msg::Mavlink::ConstSharedPtr &msg);
   

    bool isManupulator(const mavros_msgs::msg::RCIn& rcin);
    bool isCoordinate(const mavros_msgs::msg::RCIn& rcin);
    void coordinateManipulator2Body(const  Eigen::Vector3d& p_mani, Eigen::Vector3d& p_body);
    void coordinateBody2Manipulator(const Eigen::Vector3d& p_body , Eigen::Vector3d& p_mani );

    void JointStateCallBack(const sensor_msgs::msg::JointState::ConstSharedPtr& msg_p);

    void updateSystemComState();
    void publishSystemComToPx4();



    // 检查是否解锁
    void checkArmingState();
    // 检查是否协调，并初始化
    void checkCoordinateState();

    void resetCLIK();
    void resetCoordinateIni();
    void setOnGroundOrigin();
    void resetOnGroundOrigin();
    void rt_OneStep(void);

    // 修补剂控制
    void handlePainting();
    // 放下机械臂
    void putDowndMnipulator();
    // 收上机械臂
    void putUpMnipulator();
    // 协调控制
    void handleCoordinate();
    // // 关闭协调控制
    // void handleOffCoordinate();
    // 无人机安全飞行限制
    void safeFlight(geometry_msgs::msg::PoseStamped& pose_);
    // 机械臂安全工作空间
    void safeManipulator(clik::msg::PositionPub& delta_);

    // Eigen::VectorXd compute_angle(const Eigen::VectorXd & error, const Eigen::VectorXd & theta,const double dt);
    bool isNearSingular(const Eigen::MatrixXd& J, double threshold = 1e-3);
    void publishManipulatorJoints(Eigen::VectorXd desired_theta);

    // 输入：当前时间t, 总距离dist, 最大速度v_max, 加速度acc
    // 输出：通过引用返回当前位置pos_ref和当前速度vel_ref

    void GetSmoothProfileLocal(double t_curr, double total_dist, double v_max, double acc,
                           double& pos, double& vel, double& a);

    //void safe_acados_input(geometry_msgs::msg::PoseStamped& pose_)
    Eigen::Matrix3d euler_to_rotation(const Eigen::Vector3d& euler);

    Eigen:: VectorXd compute_desired_theta(const Eigen::Matrix3d& rotation_body,
                       const Eigen::VectorXd& current_theta, 
                       const Eigen::Vector3d& desired_pE,
                       const Eigen::Matrix3d& target_Re,
                       double dt);

   
    void compute_clik_control(const Eigen::Matrix3d& rotation_body, // 机身姿态(测量值)
                                   const Eigen::VectorXd& current_theta, // 机械臂角度(测量值): 仅用于算雅可比和FK
                                   const Eigen::Vector3d& pos_body,      // 机身位置(测量值): 仅用于算误差
                                   const Eigen::Vector3d& desired_pE,         // 目标末端位置
                                   const Eigen::Matrix3d& target_Re,          // 目标末端姿态
                                   double dt,
                                   // --- [输入/输出] 状态变量 (History) ---
                                   Eigen::VectorXd& q_next,            // In: 上次指令 | Out: 本次指令
                                   Eigen::Vector3d& vel_base_opt              // Out: 本次速度指令
                                   );
                                          
    Eigen::VectorXd compute_arm_prediction(const Eigen::VectorXd& error_vec_world,
                                                const Eigen::MatrixXd& J_arm_world,
                                                double dt);


    // 在 clikRos.h 中
    void fcn_indirect_force_control(const Eigen::Vector3d& v_error, 
                          const Eigen::Vector3d& v_error_int, 
                          double des_f_E, 
                          double dt,
                          double& out_adm_pos,  // [新增]
                          double& out_adm_vel); // [新增]
          
    double calculate_desired_force(const Eigen::Vector3d& vel_cmd_EE, 
                                                     const Eigen::Vector3d& acc_cmd_EE);
                      

    bool last_coordinate_flag_ = false;
    bool coordinate_off_flag_ = false;
    bool is_first_call = true;
    bool attitude_target_stream_configured_ = false;
    bool attitude_sp_received_ = false;
    rclcpp::Time last_attitude_stream_request_;
    Eigen::Vector3d px4_pos_comp_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d px4_pos_eso_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d px4_pos_total_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d px4_att_comp_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d px4_att_eso_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d px4_att_total_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d px4_pos_u_nominal_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d px4_att_u_nominal_ = Eigen::Vector3d::Zero();
    bool px4_coordinate_debug_valid_ = false;

    void ensureAttitudeTargetStream();
    bool callManipulatorMode(int64_t mode);
 
    double flight_lower_bound_[3]={-2,-2, 0.1};
    double flight_upper_bound_[3]={ 2, 2, 2.1};
    double delta_lower_bound_[3] = {-0.08,-0.08,-0.310};
    double delta_upper_bound_[3] = { 0.08, 0.08,-0.140};

    Eigen::Vector3d  pos_body;
    Eigen::Vector3d  pos_EE;
    Eigen::Matrix3d  rotation_body;
    Eigen::Matrix3d  Re;
    Eigen::Matrix3d  target_Re;
    Eigen::VectorXd  last_theta;
    Eigen::VectorXd  last_theta_vel;


    Eigen::VectorXd flying_configration_;
    Eigen::VectorXd land_configration_;
    Eigen::VectorXd shaking_configration_;

    Eigen::Matrix3d Assemble_rotation;
    Eigen::Vector3d Assemble_pos;

    double time_from_begin;
    double distance_to_object;   // 无人机悬停位置到object的距离，要小于0.74m 

    double adm_pos_err_;  // 导纳累积的位置偏差 (Y轴)
    double adm_vel_err_;  // 导纳累积的速度偏差 (Y轴)
    Eigen::Vector3d vel_error_int_EE;
    Eigen::Vector3d last_vehicle_position_cmd;
    bool is_phase8_initialized;             // 用于重置积分器的标志位
    bool coordinate_arm_task_initialized_ = false;
    Eigen::VectorXd coordinate_nominal_theta_ = Eigen::VectorXd::Zero(6);
    Eigen::Vector3d coordinate_arm_origin_body_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d coordinate_arm_target_Re_body_ = Eigen::Matrix3d::Identity();
    int coordinate_stage_ = 0;
    rclcpp::Time coordinate_stage_start_;
    rclcpp::Time coordinate_goal_reached_since_;
    Eigen::Vector3d coordinate_waypoint_1_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d coordinate_waypoint_2_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d coordinate_home_point_ = Eigen::Vector3d::Zero();
    arm_uav_kinematics::ComStateInBody com_state_in_body_;
    Eigen::Vector3d system_com_B_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d arm_com_B_ = Eigen::Vector3d::Zero();

    

};

#endif
