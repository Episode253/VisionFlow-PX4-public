#include "../include/qt_joystick/sim_remote_node.hpp"

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <utility>

namespace qt_joystick {

SimRemoteNode::SimRemoteNode(int argc, char **argv)
    : init_argc_(argc)
    , init_argv_(argv) {
}

SimRemoteNode::~SimRemoteNode() {
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    wait();
}

bool SimRemoteNode::init() {
    if (!rclcpp::ok()) {
        rclcpp::init(init_argc_, init_argv_);
    }

    node_ = std::make_shared<rclcpp::Node>("qt_joystick");
    joy_publisher_ = node_->create_publisher<sensor_msgs::msg::Joy>("virtual_joy", 50);
    joint_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("arm/joint_control", 20);
    state_subscriber_ = node_->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 10, std::bind(&SimRemoteNode::stateCallback, this, std::placeholders::_1));
    arming_client_ = node_->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

    start();
    return true;
}

void SimRemoteNode::run() {
    rclcpp::Rate loop_rate(50);

    while (rclcpp::ok()) {
        sensor_msgs::msg::Joy joy_msg;
        sensor_msgs::msg::JointState joint_msg;
        std::array<float, 7> rc_axes;
        std::array<int, 8> rc_buttons;
        std::array<double, 6> joint_targets;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            rc_axes = rc_axes_;
            rc_buttons = rc_buttons_;
            joint_targets = joint_targets_;
        }

        joy_msg.header.stamp = node_->now();
        joy_msg.header.frame_id = "sim_remote";
        joy_msg.axes.assign(rc_axes.begin(), rc_axes.end());
        joy_msg.buttons.assign(rc_buttons.begin(), rc_buttons.end());
        joy_publisher_->publish(joy_msg);

        const bool coordinate_enabled = rc_buttons[0] != 0;
        if (!coordinate_enabled) {
            joint_msg.header.stamp = node_->now();
            joint_msg.header.frame_id = "sim_arm_remote";
            joint_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
            joint_msg.position.assign(joint_targets.begin(), joint_targets.end());
            joint_publisher_->publish(joint_msg);
        }

        rclcpp::spin_some(node_);
        loop_rate.sleep();
    }

    Q_EMIT rosShutdown();
}

void SimRemoteNode::setRcAxes(const std::array<float, 7> &axes) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    rc_axes_ = axes;
}

void SimRemoteNode::setRcButtons(const std::array<int, 8> &buttons) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    rc_buttons_ = buttons;
}

void SimRemoteNode::setJointTargets(const std::array<double, 6> &joint_targets) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    joint_targets_ = joint_targets;
}

bool SimRemoteNode::armVehicle(bool arm) {
    if (!arming_client_ || !arming_client_->wait_for_service(std::chrono::milliseconds(100))) {
        return false;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    auto future = arming_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        return false;
    }
    return future.get()->success;
}

bool SimRemoteNode::setFlightMode(const std::string &custom_mode) {
    if (!set_mode_client_ || !set_mode_client_->wait_for_service(std::chrono::milliseconds(100))) {
        return false;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = custom_mode;
    auto future = set_mode_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        return false;
    }
    return future.get()->mode_sent;
}

bool SimRemoteNode::hasState() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return has_state_;
}

bool SimRemoteNode::isVehicleArmed() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_state_.armed;
}

std::string SimRemoteNode::currentMode() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_state_.mode;
}

bool SimRemoteNode::isVehicleConnected() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_state_.connected;
}

void SimRemoteNode::stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_state_ = *msg;
    has_state_ = true;
}

}  // namespace qt_joystick
