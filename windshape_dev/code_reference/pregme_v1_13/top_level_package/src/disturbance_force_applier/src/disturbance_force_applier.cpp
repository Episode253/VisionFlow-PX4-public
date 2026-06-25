#include <chrono>
#include <memory>
#include <string>

#include "gazebo_msgs/srv/apply_body_wrench.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rclcpp/rclcpp.hpp"

class DisturbanceForceApplier : public rclcpp::Node {
public:
  DisturbanceForceApplier() : Node("disturbance_force_applier") {
    body_name_ = this->declare_parameter<std::string>("body_name", "q940::base_link");
    reference_frame_ = this->declare_parameter<std::string>("reference_frame", "world");
    duration_seconds_ = this->declare_parameter<double>("duration", 0.04);

    disturbance_force_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/gazebo_force_apply", 10,
      std::bind(&DisturbanceForceApplier::forceCallback, this, std::placeholders::_1));

    apply_body_wrench_client_ =
      this->create_client<gazebo_msgs::srv::ApplyBodyWrench>("/gazebo/apply_body_wrench");
  }

private:
  void forceCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    if (!apply_body_wrench_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Service /gazebo/apply_body_wrench is not available yet");
      return;
    }

    auto request = std::make_shared<gazebo_msgs::srv::ApplyBodyWrench::Request>();
    request->body_name = body_name_;
    request->reference_frame = reference_frame_;
    request->wrench.force = *msg;
    request->wrench.force.z = -request->wrench.force.z;
    request->wrench.torque.x = 0.0;
    request->wrench.torque.y = 0.0;
    request->wrench.torque.z = 0.0;

    const int64_t duration_ns = static_cast<int64_t>(duration_seconds_ * 1000000000.0);
    request->duration.sec = static_cast<int32_t>(duration_ns / 1000000000);
    request->duration.nanosec = static_cast<uint32_t>(duration_ns % 1000000000);

    apply_body_wrench_client_->async_send_request(
      request,
      [this, request](rclcpp::Client<gazebo_msgs::srv::ApplyBodyWrench>::SharedFuture future) {
        const auto response = future.get();
        if (!response->success) {
          RCLCPP_ERROR(
            this->get_logger(), "Failed to apply force to %s: %s",
            request->body_name.c_str(), response->status_message.c_str());
          return;
        }

        RCLCPP_INFO(
          this->get_logger(), "Applied force to %s: [%.2f, %.2f, %.2f]",
          request->body_name.c_str(),
          request->wrench.force.x, request->wrench.force.y, request->wrench.force.z);
      });
  }

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr disturbance_force_sub_;
  rclcpp::Client<gazebo_msgs::srv::ApplyBodyWrench>::SharedPtr apply_body_wrench_client_;

  std::string body_name_;
  std::string reference_frame_;
  double duration_seconds_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DisturbanceForceApplier>());
  rclcpp::shutdown();
  return 0;
}
