#include "InKinematics.h"
#include "joint_talker.h"

using namespace std::chrono_literals;

joint_talker::joint_talker() : Node("joint_command_talker") {
    // 初始化 Publishers
    joint_gazebo_pub1 = this->create_publisher<std_msgs::msg::Float64>("/gamma_arm/joint1/command", 1000);
    joint_gazebo_pub2 = this->create_publisher<std_msgs::msg::Float64>("/gamma_arm/joint2/command", 1000);
    joint_gazebo_pub3 = this->create_publisher<std_msgs::msg::Float64>("/gamma_arm/joint3/command", 1000);
    joint_gazebo_pub4 = this->create_publisher<std_msgs::msg::Float64>("/gamma_arm/joint4/command", 1000);
    joint_gazebo_pub5 = this->create_publisher<std_msgs::msg::Float64>("/gamma_arm/joint5/command", 1000);
    joint_gazebo_pub6 = this->create_publisher<std_msgs::msg::Float64>("/gamma_arm/joint6/command", 1000);

    manipulator_joint_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/traj/rel_posi", 1);
    pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("arm/joint_feedback", 10);
    joint_ctrl_pub = this->create_publisher<sensor_msgs::msg::JointState>("arm/joint_control", 10);

    // 初始化 Subscribers
    endEffe_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/traj/rel_posi", 1, 
        std::bind(&joint_talker::endEffe_sub_cb, this, std::placeholders::_1)
    );

    // 初始化 Service Client
    get_jnt_state_client = this->create_client<gazebo_msgs::srv::GetJointProperties>("/gazebo/get_joint_properties");

    // 初始化 400Hz (2500微秒) 的控制循环定时器
    timer_ = this->create_wall_timer(
        std::chrono::microseconds(1000000 / updateRate),
        std::bind(&joint_talker::control_loop, this)
    );
}

void joint_talker::endEffe_sub_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    manipulator_gazebo.joint_des[0] = msg->twist.linear.x;
    manipulator_gazebo.joint_des[1] = msg->twist.linear.y;
    manipulator_gazebo.joint_des[2] = msg->twist.linear.z;
    manipulator_gazebo.joint_des[3] = msg->twist.angular.x;
    manipulator_gazebo.joint_des[4] = msg->twist.angular.y;
    manipulator_gazebo.joint_des[5] = msg->twist.angular.z;
}

// 此函数替代了原 ROS 1 main() 中的 while(ros::ok()) 循环
void joint_talker::control_loop() {
    std_msgs::msg::Float64 msg;

    // 发布指令
    msg.data = manipulator_gazebo.joint_des[0]; joint_gazebo_pub1->publish(msg);
    msg.data = manipulator_gazebo.joint_des[1]; joint_gazebo_pub2->publish(msg);
    msg.data = manipulator_gazebo.joint_des[2]; joint_gazebo_pub3->publish(msg);
    msg.data = manipulator_gazebo.joint_des[3]; joint_gazebo_pub4->publish(msg);
    msg.data = manipulator_gazebo.joint_des[4]; joint_gazebo_pub5->publish(msg);
    msg.data = manipulator_gazebo.joint_des[5]; joint_gazebo_pub6->publish(msg);

    // 从 Gazebo 获取反馈 (ROS 2 中推荐异步处理 Service 以避免节点假死)
    std::vector<std::string> joint_names = {
        "ti5_arm::A", "ti5_arm::B", "ti5_arm::C", "ti5_arm::D", "ti5_arm::E", "ti5_arm::F"
    };

    for (int i = 0; i < 6; i++) {
        if (get_jnt_state_client->service_is_ready()) {
            auto request = std::make_shared<gazebo_msgs::srv::GetJointProperties::Request>();
            request->joint_name = joint_names[i];
            
            // 异步请求
            get_jnt_state_client->async_send_request(request, 
                [this, i](rclcpp::Client<gazebo_msgs::srv::GetJointProperties>::SharedFuture future) {
                    auto response = future.get();
                    if (response && !response->position.empty() && !response->rate.empty()) {
                        this->manipulator_gazebo.pos_cur[i] = response->position[0];
                        this->manipulator_gazebo.vel_cur[i] = response->rate[0];
                    }
                }
            );
        }
    }

    // send feedback to clik
    auto joint_state_msg_ = sensor_msgs::msg::JointState();
    joint_state_msg_.header.frame_id = "arm_frame";
    joint_state_msg_.header.stamp = this->now();
    
    for (uint16_t i = 0; i < 6; i++) {
        joint_state_msg_.name.push_back("joint" + std::to_string(i));
        joint_state_msg_.position.push_back(manipulator_gazebo.pos_cur[i]);
        joint_state_msg_.velocity.push_back(manipulator_gazebo.vel_cur[i]);
    }
    
    pub_joint_state_->publish(joint_state_msg_);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    // 实例化并自动挂起运行
    auto node = std::make_shared<joint_talker>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}