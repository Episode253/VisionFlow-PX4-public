#ifndef INKINEMATICS_H
#define INKINEMATICS_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "gazebo_msgs/srv/get_joint_properties.hpp"

#include <tf2/LinearMath/Vector3.h>
#include <cmath>
#include <sstream>
#include <vector>

using namespace std;

#define pi 3.14159265358979323846
#define z_ed -0.14

// 在 ROS 2 中，节点需要继承 rclcpp::Node
class Inverse_Kinematics_Talker : public rclcpp::Node {
private:
    double R = 0.100;
    double r = 0.030;
    double L = 0.120;
    double l = 0.240;

    // inverse kinematics results
    double theta1_, theta2_, theta3_;
    double theta1_save_, theta2_save_, theta3_save_;

    // for inverse kinematics computing
    double x_, y_, z_;

    // from trajGenerator
    tf2::Vector3 desiredPos;

    double IKinemTh(double x0, double y0, double z0);
    void calculation(double X, double Y, double Z);

public:
    Inverse_Kinematics_Talker();
    void inverse_kinematics();
    bool f1;

    // sub trajGenerator cb
    void endEffe_sub_cb(const geometry_msgs::msg::Point::SharedPtr msg);

    void setPos();
    tf2::Vector3 getJointTheta();
    tf2::Vector3 getDesiredPos();

    // ROS 2 Publishers & Subscribers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint_gazebo_pub1;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint_gazebo_pub2;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint_gazebo_pub3;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint_gazebo_pub4;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint_gazebo_pub5;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint_gazebo_pub6;
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr endEffe_sub;
};

#endif