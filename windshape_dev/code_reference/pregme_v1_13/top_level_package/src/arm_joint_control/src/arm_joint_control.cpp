#include <algorithm>
#include <chrono>
#include <cmath>
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
    desired_theta_(Eigen::VectorXd::Zero(6)),
    flying_configration_(Eigen::VectorXd::Zero(6))
  {
    // 定义发布者：向"arm/joint_control"话题发布JointState消息
    joint_ctrl_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("arm/joint_control", 10);

    // 定义订阅者：订阅"arm/joint_feedback"
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "arm/joint_feedback", 10,
      std::bind(&ArmJointControl::jointStateCallback, this, std::placeholders::_1));

    // 打开日志文件
    log_file_.open("joint_log.txt");
    if (!log_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "could not open joint_log.txt");
      throw std::runtime_error("Could not open joint_log.txt");
    }
    
    RCLCPP_INFO(this->get_logger(), "open joint_log.txt");
    RCLCPP_INFO(this->get_logger(), "start control...");
  }

  ~ArmJointControl() {
    // 析构函数中自动关闭文件并输出日志
    if (log_file_.is_open()) {
      log_file_.close();
      RCLCPP_INFO(this->get_logger(), "Log file closed. Program exited.");
    }
  }

  // 等待 ROS 时钟同步
  void waitForClock() {
    while (rclcpp::ok() && this->now().nanoseconds() == 0) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "Waiting for ROS clock...");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    start_time_ = this->now();
  }

  // 执行单步控制循环，返回 false 表示需要退出
  bool step() {
    // 计算当前运行时间（秒）
    double current_time = (this->now() - start_time_).seconds();

    // --------------------------
    // --- 自动退出逻辑 ---
    // --------------------------
    if (current_time >= 16.0) {
      RCLCPP_INFO(this->get_logger(), "Reached 16 seconds. Shutting down node...");
      return false; // 返回 false 以跳出 while 循环
    }

    // --------------------------
    // 阶段1：前5秒发送飞行构型
    // --------------------------
    if (current_time <= 5.0) {
      flying_configration_ << 0.0, 0.0, -M_PI/4, 0.0, M_PI/2, 0.0;
      desired_theta_ = flying_configration_;
    }
    // --------------------------
    // 阶段2：5秒后运动到新的固定角度2
    // --------------------------
    else if (current_time <= 10.0) {
      flying_configration_ << M_PI/5, M_PI/3, 0.0, M_PI/3, 0.0, M_PI/4;
      desired_theta_ = flying_configration_;
    }
    // --------------------------
    // 阶段3：10秒后运动到新的固定角度3
    // --------------------------
    else if (current_time <= 15.0) {
      flying_configration_ << M_PI/4, M_PI/6, M_PI/3, M_PI/6, M_PI/4, M_PI/2;
      desired_theta_ = flying_configration_;
    }

    // 发布控制指令
    sensor_msgs::msg::JointState joint_ctrl_msg;
    joint_ctrl_msg.position = {
      desired_theta_(0), desired_theta_(1), desired_theta_(2),
      desired_theta_(3), desired_theta_(4), desired_theta_(5)
    };
    joint_ctrl_msg.header.frame_id = "joint_ctrl_frame";
    joint_ctrl_msg.header.stamp = this->now();
    joint_ctrl_pub_->publish(joint_ctrl_msg);

    // 记录日志
    if (log_file_.is_open()) {
      log_file_ << current_time;  // 时间戳
      log_file_ << "\n desired_theta: \n" << desired_theta_.transpose() << std::endl;
      log_file_ << "current_theta: \n" << clik_joint_pos_.transpose() << std::endl;
    }

    return true; // 继续执行
  }

private:
  // 机械臂状态回调函数
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 使用安全的赋值方式替代原先的 memcpy
    const size_t count = std::min<size_t>(6, msg->position.size());
    for (size_t i = 0; i < count; ++i) {
      clik_joint_pos_(static_cast<Eigen::Index>(i)) = msg->position[i];
    }

    if (count < 6) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Received JointState with %zu positions; expected at least 6", msg->position.size());
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_ctrl_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  
  Eigen::VectorXd clik_joint_pos_;
  Eigen::VectorXd desired_theta_;
  Eigen::VectorXd flying_configration_;
  
  rclcpp::Time start_time_;
  std::ofstream log_file_;
};

int main(int argc, char ** argv) {
  // 初始化ROS 2节点
  rclcpp::init(argc, argv);

  // 创建节点实例
  auto node = std::make_shared<ArmJointControl>();
  
  // 等待时钟同步（处理仿真或bag回放场景的时间跳变）
  node->waitForClock();

  // 设置循环频率（100Hz）
  rclcpp::Rate loop_rate(100);
  
  while (rclcpp::ok()) {
    // 运行时间状态机和发布逻辑，若返回 false 则跳出循环
    if (!node->step()) {
      break; 
    }
    
    // 处理回调队列 (相当于 ros::spinOnce)
    rclcpp::spin_some(node);
    
    // 按频率休眠
    loop_rate.sleep();
  }

  // 关闭 ROS 2（析构函数会自动清理日志文件）
  rclcpp::shutdown();
  return 0;
}