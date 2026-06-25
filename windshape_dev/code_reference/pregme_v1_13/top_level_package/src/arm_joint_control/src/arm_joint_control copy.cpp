#include <algorithm>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include <Eigen/Eigen>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class ArmJointControl : public rclcpp::Node {
public:
  ArmJointControl()
  : Node("arm_joint_control"),
    clik_joint_pos_(Eigen::VectorXd::Zero(6)),
    desired_theta_(Eigen::VectorXd::Zero(6))
  {
    const std::string log_file_path = this->declare_parameter<std::string>(
      "log_file_path", "joint_log.txt");

    joint_desired_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "arm/joint_control", 10,
      std::bind(&ArmJointControl::jointDesiredCallback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "arm/joint_feedback", 10,
      std::bind(&ArmJointControl::jointStateCallback, this, std::placeholders::_1));

    log_file_.open(log_file_path);
    if (!log_file_.is_open()) {
      throw std::runtime_error("Could not open " + log_file_path);
    }

    RCLCPP_INFO(this->get_logger(), "Opened %s", log_file_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Start control...");
  }

  void waitForClock() {
    while (rclcpp::ok() && this->now().nanoseconds() == 0) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "Waiting for ROS clock...");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    start_time_ = this->now();
  }

  void logOnce() {
    const double current_time = (this->now() - start_time_).seconds();
    log_file_ << current_time;
    log_file_ << "\n desired_theta: \n" << desired_theta_.transpose() << std::endl;
    log_file_ << "current_theta: \n" << clik_joint_pos_.transpose() << std::endl;
  }

private:
  void jointDesiredCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    copyPositions(msg, desired_theta_);
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    copyPositions(msg, clik_joint_pos_);
  }

  void copyPositions(
    const sensor_msgs::msg::JointState::SharedPtr & msg,
    Eigen::VectorXd & destination)
  {
    const size_t count = std::min<size_t>(6, msg->position.size());
    for (size_t i = 0; i < count; ++i) {
      destination(static_cast<Eigen::Index>(i)) = msg->position[i];
    }

    if (count < 6) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Received JointState with %zu positions; expected at least 6", msg->position.size());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_desired_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  Eigen::VectorXd clik_joint_pos_;
  Eigen::VectorXd desired_theta_;
  rclcpp::Time start_time_;
  std::ofstream log_file_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ArmJointControl>();
  node->waitForClock();

  rclcpp::Rate loop_rate(100);
  while (rclcpp::ok()) {
    node->logOnce();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
