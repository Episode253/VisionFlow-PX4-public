#ifndef SIM_TO_REAL_H
#define SIM_TO_REAL_H

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// 遥控器指令
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <sensor_msgs/msg/joy.hpp>

// 末端位置
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"             // 用于接收 Gazebo 发过来的 Pose_V 数组
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

// 机械臂指令
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>

// 自定义消息和服务 (注意命名空间和后缀)
#include "sim2real/msg/position_pub.hpp"
#include "sim2real/srv/cmd_mode.hpp"
#include "sim2real/msg/gripper_cmd.hpp"

#include <chrono>

enum COM_MODE { mod_shrink = 0, mod_prepare = 1, mod_control = 2, mod_wait = 3 };

class sim2realclass : public rclcpp::Node
{
public:
    sim2realclass();
    ~sim2realclass() {};
    void setInitJoint();

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr EndEffector_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr RC_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr EndEffector_pub_;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr RC_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr endEffe_traj_pub_;
    rclcpp::Publisher<sim2real::msg::GripperCmd>::SharedPtr gripper_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint_gripper_gazebo_pub1;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint_gripper_gazebo_pub2;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr EndEffector_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr manipulator_joint_pub_;

    rclcpp::Service<sim2real::srv::CmdMode>::SharedPtr cmd_mod_srv_;

    rclcpp::Subscription<sim2real::msg::PositionPub>::SharedPtr cmd_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_gripper_mod_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_ctrl_;

    geometry_msgs::msg::TransformStamped endEffectorPosition_;
    geometry_msgs::msg::Point traj_endEffe_;
    geometry_msgs::msg::TwistStamped endEffectorVelocity_;
    sim2real::msg::GripperCmd gripper_cmd_;
    geometry_msgs::msg::TwistStamped manipulator_joint;
    
    std_msgs::msg::Float32 gripper1_, gripper2_;
    mavros_msgs::msg::OverrideRCIn RC_Override_;

    // 定义一个定时器，用于周期性执行某些任务（例如检查状态、发布心跳等）
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // 用于计算速度的缓存
    bool has_last_pose_ = false;
    geometry_msgs::msg::Point last_position_;
    rclcpp::Time last_time_;
    

    int Cmd_mode_ = 0;
    rclcpp::Time Mod_time_;

    // 回调函数声明 (注意 Service 的传参发生了变化)
    void cmd_mode_Callback(const std::shared_ptr<sim2real::srv::CmdMode::Request> req,
                           std::shared_ptr<sim2real::srv::CmdMode::Response> res);
    
    void cmd_gripper_Callback(const std_msgs::msg::String::SharedPtr msg);
    void endEffector_obtain(const nav_msgs::msg::Odometry::SharedPtr msg);
    void cmd_pos_Callback(const sim2real::msg::PositionPub::SharedPtr msg);
    void rc_obtain(const sensor_msgs::msg::Joy::SharedPtr msg);
    void timer_callback();
    void JointControlCallBack(const sensor_msgs::msg::JointState::SharedPtr msg_p);
};

#endif