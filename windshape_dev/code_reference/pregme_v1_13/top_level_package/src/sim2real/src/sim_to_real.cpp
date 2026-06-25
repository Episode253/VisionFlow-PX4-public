#include "sim_to_real.h"
#include <unistd.h>

double pi = 3.1415926;

sim2realclass::sim2realclass() : Node("sim2real"), Mod_time_(0, 0, this->get_clock()->get_clock_type())
{
    // 初始化数组大小，ROS 2 中使用 std::array 特性的消息最好先调整大小
    RC_Override_.channels.fill(0); // mavros 的 channels 固定是18个，这里用 fill 初始化

    sub_joint_ctrl_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "arm/joint_control", 10, std::bind(&sim2realclass::JointControlCallBack, this, std::placeholders::_1));
    
    EndEffector_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ground_truth/position_end_effc", 10, std::bind(&sim2realclass::endEffector_obtain, this, std::placeholders::_1));

    // EndEffector_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "/gamma_end/ground_truth/odom", 10, std::bind(&sim2realclass::endEffector_obtain, this, std::placeholders::_1));

    RC_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/virtual_joy", 10, std::bind(&sim2realclass::rc_obtain, this, std::placeholders::_1));

    cmd_gripper_mod_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/chatter_tool", 10, std::bind(&sim2realclass::cmd_gripper_Callback, this, std::placeholders::_1));  

    // 这里原代码注释了 cmd_pos_sub_，如果你需要取消注释，参考下面的格式：
    // cmd_pos_sub_ = this->create_subscription<sim2real::msg::PositionPub>(
    //     "/control_signal/pos_pub", 10, std::bind(&sim2realclass::cmd_pos_Callback, this, std::placeholders::_1));

    // 创建服务
    cmd_mod_srv_ = this->create_service<sim2real::srv::CmdMode>(
        "control_signal/command_mode", std::bind(&sim2realclass::cmd_mode_Callback, this, std::placeholders::_1, std::placeholders::_2));

    // 创建发布者
    EndEffector_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/vrpn_client_node/cart/twist", 10);
    EndEffector_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("vicon/Gamma_arm_EE/Gamma_arm_EE", 10); 
    RC_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("mavros/rc/override", 10); 
    gripper_cmd_pub_ = this->create_publisher<sim2real::msg::GripperCmd>("/gripper_cmd", 1); 
    joint_gripper_gazebo_pub1 = this->create_publisher<std_msgs::msg::Float32>("/joint/gripper1_1/position_cmd", 10);
    joint_gripper_gazebo_pub2 = this->create_publisher<std_msgs::msg::Float32>("/joint/gripper1_2/position_cmd", 10);
    manipulator_joint_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/traj/rel_posi", 1); 

        // 1. 初始化 TF2 Buffer 和 Listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 2. 创建发布者：发布完整的 Odometry (包含位姿和我们自己算出的速度)
        // 或者你可以改回发布 TransformStamped，只发位姿
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("vicon/Gamma_arm_EE/odom", 10);

        // 3. 创建定时器 (例如 100Hz 运行)，周期性查询 TF 树
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&sim2realclass::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "TF Listener Node Started. Listening to world -> F_Link");


    setInitJoint();
}

void sim2realclass::setInitJoint()
{
    manipulator_joint.twist.linear.x = 0;
    manipulator_joint.twist.linear.y = 0;
    manipulator_joint.twist.linear.z = 0;
    manipulator_joint.twist.angular.x = 0;
    manipulator_joint.twist.angular.y = 0;
    manipulator_joint.twist.angular.z = 0;
    manipulator_joint_pub_->publish(manipulator_joint);
}

void sim2realclass::rc_obtain(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    for(int i = 0; i < 7; i++) {
        uint16_t uu = 1500 - msg->axes[i] * 500;
        RC_Override_.channels.at(i) = uu;
    }
    for(int i = 0; i < 8; i++) {
        RC_Override_.channels[i+8] = 1400 + msg->buttons[i] * 600;
    }
    RC_pub_->publish(RC_Override_);
}

void sim2realclass::endEffector_obtain(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    endEffectorPosition_.transform.translation.x = msg->pose.pose.position.x; 
    endEffectorPosition_.transform.translation.y = msg->pose.pose.position.y; 
    endEffectorPosition_.transform.translation.z = msg->pose.pose.position.z - 0.8 - 0.32200; 
    endEffectorPosition_.transform.rotation = msg->pose.pose.orientation; 

    EndEffector_pub_->publish(endEffectorPosition_);

    endEffectorVelocity_.header = msg->header; 
    endEffectorVelocity_.twist = msg->twist.twist;

    EndEffector_vel_pub_->publish(endEffectorVelocity_);
}


    void sim2realclass::timer_callback()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        
        try {
            // 核心功能：向 TF 树查询最新时刻从 "world" 到 "F_Link" 的绝对坐标变换
            transformStamped = tf_buffer_->lookupTransform(
                "world",           // 父坐标系 (Global)
                "F_Link",          // 子坐标系 (你的末端执行器)
                tf2::TimePointZero // 获取最新可用的变换
            );
        } 
        catch (const tf2::TransformException & ex) {
            // 在刚启动时，TF 树可能还没建好，忽略初期报错
            // RCLCPP_DEBUG(this->get_logger(), "Could not transform world to F_Link: %s", ex.what());
            RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
            return;
        }

        // ================= 数据打包与速度计算 =================
        
        auto current_time = this->now();
        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.stamp = transformStamped.header.stamp;
        odom_msg.header.frame_id = "world";
        odom_msg.child_frame_id = "gamma_arm_EE"; // 伪装成 Vicon 标识符

        // 1. 填入绝对位姿 (直接从 TF 获取)
        odom_msg.pose.pose.position.x = transformStamped.transform.translation.x;
        odom_msg.pose.pose.position.y = transformStamped.transform.translation.y;
        odom_msg.pose.pose.position.z = transformStamped.transform.translation.z;
        odom_msg.pose.pose.orientation = transformStamped.transform.rotation;
        endEffectorPosition_.transform.translation.x = odom_msg.pose.pose.position.x;
        endEffectorPosition_.transform.translation.y = odom_msg.pose.pose.position.y;
        endEffectorPosition_.transform.translation.z = odom_msg.pose.pose.position.z - 0.8 - 0.32200;
        endEffectorPosition_.transform.rotation = odom_msg.pose.pose.orientation;

        EndEffector_pub_->publish(endEffectorPosition_);
        
        // 2. 差分计算线速度 (v = dx / dt)
        if (has_last_pose_) {
            double dt = (current_time - last_time_).seconds();
            if (dt > 0.0) {
                odom_msg.twist.twist.linear.x = (odom_msg.pose.pose.position.x - last_position_.x) / dt;
                odom_msg.twist.twist.linear.y = (odom_msg.pose.pose.position.y - last_position_.y) / dt;
                odom_msg.twist.twist.linear.z = (odom_msg.pose.pose.position.z - last_position_.z) / dt;
            }
        } else {
            odom_msg.twist.twist.linear.x = 0.0;
            odom_msg.twist.twist.linear.y = 0.0;
            odom_msg.twist.twist.linear.z = 0.0;
            has_last_pose_ = true;
        }

        // 角速度的差分较复杂(需要四元数求导)，如果是真机动捕系统通常也只重点关注线速度
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;

        // 缓存当前状态供下次使用
        last_position_ = odom_msg.pose.pose.position;
        last_time_ = current_time;


        endEffectorVelocity_.header = odom_msg.header; 
        endEffectorVelocity_.twist = odom_msg.twist.twist;

        EndEffector_vel_pub_->publish(endEffectorVelocity_);


        // 3. 发布出去！
        odom_pub_->publish(odom_msg);
    }

// 【Service Callback】 注意 ROS2 这里的参数变成了智能指针
void sim2realclass::cmd_mode_Callback(
    const std::shared_ptr<sim2real::srv::CmdMode::Request> req,
    std::shared_ptr<sim2real::srv::CmdMode::Response> res)
{
    if(req->mode == 0 || req->mode == 1 || req->mode == 2) {
        Mod_time_ = this->now();
        res->result = true;
        Cmd_mode_ = req->mode;
    } else {
        Mod_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        Cmd_mode_ = 0;
        res->result = false;
    }

    switch(Cmd_mode_)
    {
        case mod_shrink:
            manipulator_joint.twist.linear.x = 0;
            manipulator_joint.twist.linear.y = 0;
            manipulator_joint.twist.linear.z = 0;
            manipulator_joint.twist.angular.x = 0;
            manipulator_joint.twist.angular.y = 0;
            manipulator_joint.twist.angular.z = 0;
            manipulator_joint_pub_->publish(manipulator_joint);
            Cmd_mode_ = mod_wait;
            break;

        case mod_prepare:
            for(int i = 0; i < 21; i++) {
                manipulator_joint.twist.linear.x = 0;
                manipulator_joint.twist.linear.y = -pi/2;
                manipulator_joint.twist.linear.z = -pi/2;
                manipulator_joint.twist.angular.x = 0;
                manipulator_joint.twist.angular.y = 0;
                manipulator_joint.twist.angular.z = 0;
                
                manipulator_joint_pub_->publish(manipulator_joint);
                Cmd_mode_ = mod_wait;
                RCLCPP_INFO(this->get_logger(), "joint send;");
                
                // 警告: usleep 会阻塞 ROS 2 的单线程执行器，导致期间无法处理其他回调
                // 如果这是个严谨的控制系统，建议未来用 Timer 重构。这里为了保持原逻辑暂时保留。
                usleep(20000); 
            }
            break;

        case mod_control:
            Cmd_mode_ = mod_wait;
            break;
            
        case mod_wait:
            break;
    }
}

void sim2realclass::cmd_gripper_Callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "接收到的数据：%s", msg->data.c_str());
    
    if(msg->data.length() > 3) {
        if(msg->data[3] == '1'){       // gripper is close
            gripper_cmd_.gripper_left = -0.02;
            gripper_cmd_.gripper_right = 0.02;
            gripper_cmd_pub_->publish(gripper_cmd_);
        }
        else if(msg->data[3] == '0'){  // gripper is open
            gripper_cmd_.gripper_left = 0.02;
            gripper_cmd_.gripper_right = -0.02;
            gripper_cmd_pub_->publish(gripper_cmd_);
        }
        else {
            RCLCPP_INFO(this->get_logger(), "gripper_server no right case");
        }
    }

    gripper1_.data = gripper_cmd_.gripper_left;
    joint_gripper_gazebo_pub1->publish(gripper1_);
    gripper2_.data = gripper_cmd_.gripper_right;
    joint_gripper_gazebo_pub2->publish(gripper2_);
}

void sim2realclass::cmd_pos_Callback(const sim2real::msg::PositionPub::SharedPtr msg1)
{
    // 此函数在原代码中被注释掉，为了完整性提供 ROS 2 版本
    float workspace[6] = {-0.08, 0.08, -0.08, 0.08, -0.40, -0.120};
    traj_endEffe_.x = msg1->x < workspace[0] ? workspace[0] : (msg1->x > workspace[1] ? workspace[1] : msg1->x);
    traj_endEffe_.y = msg1->y < workspace[2] ? workspace[2] : (msg1->y > workspace[3] ? workspace[3] : msg1->y);
    traj_endEffe_.z = msg1->z < workspace[4] ? workspace[4] : (msg1->z > workspace[5] ? workspace[5] : msg1->z);
    // endEffe_traj_pub_->publish(traj_endEffe_); // 需要初始化此 publisher 才能使用
}

void sim2realclass::JointControlCallBack(const sensor_msgs::msg::JointState::SharedPtr msg_p)
{
    if (msg_p->position.size() >= 6) {
        manipulator_joint.twist.linear.x = (float)msg_p->position[0];
        manipulator_joint.twist.linear.y = (float)msg_p->position[1];
        manipulator_joint.twist.linear.z = (float)msg_p->position[2];
        manipulator_joint.twist.angular.x = (float)msg_p->position[3];
        manipulator_joint.twist.angular.y = (float)msg_p->position[4];
        manipulator_joint.twist.angular.z = (float)msg_p->position[5];
        manipulator_joint_pub_->publish(manipulator_joint);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sim2realclass>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}