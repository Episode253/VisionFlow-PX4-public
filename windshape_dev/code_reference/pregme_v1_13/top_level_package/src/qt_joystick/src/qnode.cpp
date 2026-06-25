/**
 * @file /src/qnode.cpp
 *
 * @brief ROS communication central!
 *
 * @date February 2011
 **/

#include "../include/qt_joystick/qnode.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>

namespace qt_joystick {

static mavros_msgs::msg::State current_state;

void QNode::state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state = *msg;
}

QNode::QNode(int argc, char** argv)
    : init_argc(argc)
    , init_argv(argv) {
}

QNode::~QNode() {
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    wait();
}

bool QNode::init() {
    if (!rclcpp::ok()) {
        rclcpp::init(init_argc, init_argv);
    }

    node_ = std::make_shared<rclcpp::Node>("qt_joystick");
    joy_publisher = node_->create_publisher<sensor_msgs::msg::Joy>("joy", 1000);
    arming_client = node_->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    state_sub = node_->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 10, QNode::state_cb);

    start();
    return true;
}

void QNode::run() {
    rclcpp::Rate loop_rate(50);

    while (rclcpp::ok()) {
        sensor_msgs::msg::Joy joy_msg;
        joy_msg.header.stamp = node_->now();
        joy_msg.header.frame_id = "dev/input/js0";
        joy_msg.axes.emplace_back(0);
        joy_msg.axes.emplace_back(0);
        joy_msg.axes.emplace_back(0);
        joy_msg.axes.emplace_back(0);
        joy_msg.axes.emplace_back(Send_data_[0]);
        joy_msg.axes.emplace_back(Send_data_[1]);
        joy_msg.axes.emplace_back(Send_data_[2]);
        joy_msg.buttons.emplace_back(Send_data_[3]);
        joy_msg.buttons.emplace_back(Send_data_[4]);
        joy_msg.buttons.emplace_back(Send_data_[5]);
        joy_msg.buttons.emplace_back(0);
        joy_msg.buttons.emplace_back(0);
        joy_msg.buttons.emplace_back(0);
        joy_msg.buttons.emplace_back(0);
        joy_msg.buttons.emplace_back(0);
        joy_publisher->publish(joy_msg);

        if (!current_state.armed && std::abs(Send_data_[5]) > 0.9 && arming_client->service_is_ready()) {
            auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            request->value = true;
            arming_client->async_send_request(request);
        }

        rclcpp::spin_some(node_);
        loop_rate.sleep();
    }

    std::cout << "ROS shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown();
}

void QNode::Get_data(int *data) {
    for (int i = 0; i < 6; i++) {
        Send_data_[i] = data[i];
    }
}

}  // namespace qt_joystick
