#ifndef CLIK_MAIN_H
#define CLIK_MAIN_H

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/qos.hpp"
#include <mavros_msgs/msg/actuator_control.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <stddef.h>
#include <stdio.h>                
#include <mutex>
#include <Eigen/Eigen>
#include <math.h>
#include <string.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <fstream>
#include <gazebo_msgs/srv/get_joint_properties.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

// 本包生成的 msg / srv
#include "calibration/msg/position_pub.hpp"
#include "calibration/srv/manipulator_mode.hpp"
#include "calibration/msg/action.hpp"

#define _USE_MATH_DEFINES

enum m_behavior {IDLE, WAYPOINT_FLIGHT,LAND, HOVER,TAKEOFF,FORWARD,BACKWARD,RIGHT,LEFT,UP,DOWN};

typedef struct {
  double x    = 0;                   
  double y    = 0;                      
  double z    = 0;                             
  double phi  = 0;                    
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
} coordinateIni;

enum COM_MODE { mod_shrink = 0, mod_prepare = 1, mod_control = 2 ,mod_sleep =3,mod_wait =4,mod_read=5};

class clikRos : public rclcpp::Node
{
public:
    clikRos();
    ~clikRos(){};

    void mainLoop();

    flightStateStruct flightStateData;
    coordinateIni m_coordinate_contr_ini;

    mavros_msgs::msg::RCIn m_rcin_;
    mavros_msgs::msg::RCIn m_rcin_prev_;
 
    mavros_msgs::msg::State current_state;
    COM_MODE manipulator_mode;

private:
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr attitude_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr attitude_sp_sub;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rcin_sub;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr Ti5_arm_EE_sub;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr Ti5_arm_base_sub;
    rclcpp::Subscription<calibration::msg::Action>::SharedPtr action_sub;

    rclcpp::Client<calibration::srv::ManipulatorMode>::SharedPtr traj_solver_client;
    rclcpp::Client<calibration::srv::ManipulatorMode>::SharedPtr traj_out_client;
    
    rclcpp::Publisher<calibration::msg::PositionPub>::SharedPtr Delta_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr write_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_ctrl_pub;

    rclcpp::Client<calibration::srv::ManipulatorMode>::SharedPtr manipulator_client;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
    rclcpp::Client<gazebo_msgs::srv::GetJointProperties>::SharedPtr get_jnt_state_client;

    bool reset_CLIK_flag_;
    bool on_off_manipulator_flag_; 
    bool ground_origin_position_initialized_flag_ ;
    bool coordinate_flag_;
    bool reset_coordinate_flag_;
    bool stall_manipulater_flag_;
    bool first_off_manipulator;
    bool first_on_manipulator;
    bool coordinate_running_flag_;

    rclcpp::Time last_off_manipulator;
    rclcpp::Time last_on_manipulator;
    rclcpp::Time last_coordinate;
    rclcpp::Time last_time;
    rclcpp::Time now_time;
    rclcpp::Time test_begin;
    std::mutex rc_mutex_;
    std_msgs::msg::String result_serial; 

    double Yout[6];
    
    int origin_counter;
    double pe[3];
    Eigen::Vector3d dOffset;
    double yaw_offset;

    geometry_msgs::msg::PoseStamped m_ref_origin_; 
    geometry_msgs::msg::PoseStamped current_local_pos; 
    calibration::msg::PositionPub Delts_cont; 
    std::shared_ptr<calibration::srv::ManipulatorMode::Request> Delta_mode_req; 
    mavros_msgs::srv::SetMode::Request offb_set_mode; 
    geometry_msgs::msg::PoseStamped pose; 
    Eigen::Vector3d attitude_sp; 
    calibration::msg::Action cur_action;

    sensor_msgs::msg::JointState joint_ctrl_msg; 

    void state_obtain(const mavros_msgs::msg::State::SharedPtr msg);
    void pos_obtain(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void rcin_obtain(const mavros_msgs::msg::RCIn::SharedPtr msg);
    void att_obtain(const sensor_msgs::msg::Imu::SharedPtr msg);
    void Ti5_arm_EE_obtain(const geometry_msgs::msg::TransformStamped::SharedPtr message_holder);
    void vehicle_action_callback(const calibration::msg::Action::SharedPtr msg);
    void att_sp_obtain(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
   
    bool isManupulator(const mavros_msgs::msg::RCIn& rcin);
    bool isCoordinate(const mavros_msgs::msg::RCIn& rcin);
    void coordinateManipulator2Body(const Eigen::Vector3d& p_mani, Eigen::Vector3d& p_body);
    void coordinateBody2Manipulator(const Eigen::Vector3d& p_body , Eigen::Vector3d& p_mani );

    void publishCommands(int i);
    void recordData(int i);

    void checkArmingState();
    void checkCoordinateState();

    void resetCLIK();
    void resetCoordinateIni();
    void setOnGroundOrigin();
    void resetOnGroundOrigin();
    void rt_OneStep(void);

    void handleCoordinate();

    double intePe[3];
    double intePeCmd[3];
    void euler_to_rotation(const Eigen::Vector3d& euler, Eigen::Matrix3d& rotation);
    Eigen::Vector3d point_wB;
    Eigen::Vector3d point_b_desire;
    Eigen::Vector3d point_e_desire;
    Eigen::Vector3d point_e_b_desire;
    bool last_coordinate_flag_ = false;
    bool coordinate_off_flag_ = false;
    double start_coordinate_point_[3];
    int state_rc_1  = 0; 
    int state_rc_2  = 0;

    int current_group = 0;  
    bool command_published = false;  
    bool data_recorded = false;     

    std::vector<std::vector<double>> q_serial_des; 

    std::vector<Eigen::Vector3d> pos_body;
    std::vector<Eigen::Vector3d> pos_EE;
    std::vector<Eigen::Matrix3d> rotation_b2i;
    
    bool flag_calibrate = false;
    double every_group_time_interval = 10;
    double group_n = 12;
};

#endif