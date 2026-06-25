#ifndef JOINT_TALKER_H
#define JOINT_TALKER_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "gazebo_msgs/srv/get_joint_properties.hpp"

#include <cmath>
#include <sstream>
#include <vector>

using namespace std;

#define pi 3.14159265358979323846
#define updateRate 400

struct manipulator_data {
    double joint_des[6] = {0};
    double pos_cur[6] = {0};
    double vel_cur[6] = {0};

    double clik_joint_pos[6] = {0};
    double clik_joint_vel[6] = {0};
};

class joint_talker : public rclcpp::Node {
public:
    joint_talker();
    ~joint_talker() {}

private:
    // 将原先散落在 main() 的逻辑彻底封装在类中
    manipulator_data manipulator_gazebo;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint_gazebo_pub1;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint_gazebo_pub2;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint_gazebo_pub3;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint_gazebo_pub4;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint_gazebo_pub5;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint_gazebo_pub6;
    
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr manipulator_joint_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_ctrl_pub;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr endEffe_sub;
    
    rclcpp::Client<gazebo_msgs::srv::GetJointProperties>::SharedPtr get_jnt_state_client;
    
    // 定时器用于替代 ROS 1 中的 while(ros::ok())
    rclcpp::TimerBase::SharedPtr timer_;

    void endEffe_sub_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void control_loop();
};

#endif