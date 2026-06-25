#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class GripperUDPController : public rclcpp::Node {
public:
  GripperUDPController() : Node("gripper"), sock_fd_(-1) {
    target_ip_ = this->declare_parameter<std::string>("target_ip", "192.168.50.199");
    target_port_ = this->declare_parameter<int>("target_port", 6003);

    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create socket: %s", std::strerror(errno));
      rclcpp::shutdown();
      return;
    }

    std::memset(&target_addr_, 0, sizeof(target_addr_));
    target_addr_.sin_family = AF_INET;
    target_addr_.sin_port = htons(static_cast<uint16_t>(target_port_));
    if (inet_pton(AF_INET, target_ip_.c_str(), &target_addr_.sin_addr) <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid target IP address: %s", target_ip_.c_str());
      rclcpp::shutdown();
      return;
    }

    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/gripper_command", 10,
      std::bind(&GripperUDPController::commandCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(), "UDP gripper controller initialized for %s:%d",
      target_ip_.c_str(), target_port_);
  }

  ~GripperUDPController() override {
    if (sock_fd_ >= 0) {
      close(sock_fd_);
    }
  }

private:
  void commandCallback(const std_msgs::msg::String::SharedPtr msg) {
    const std::string & cmd = msg->data;
    std::string udp_msg;

    if (cmd == "open") {
      udp_msg = "A10";
    } else if (cmd == "close") {
      udp_msg = "A11";
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown command: %s", cmd.c_str());
      return;
    }

    const ssize_t sent = sendto(
      sock_fd_, udp_msg.c_str(), udp_msg.length(), 0,
      reinterpret_cast<struct sockaddr *>(&target_addr_), sizeof(target_addr_));

    if (sent < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send UDP message: %s", std::strerror(errno));
    } else {
      RCLCPP_INFO(this->get_logger(), "Sent UDP command: %s", udp_msg.c_str());
    }
  }

  int sock_fd_;
  std::string target_ip_;
  int target_port_;
  struct sockaddr_in target_addr_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperUDPController>());
  rclcpp::shutdown();
  return 0;
}
