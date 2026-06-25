#include "clik_main.h"
#include "kinematic.h"
// 修改：
#include <fstream>
#include <iomanip>
#include <string.h>

// 【在此处修改你想保存的绝对路径】
// 注意：请确保运行程序的账户对该目录有写入权限，并将 "your_username" 换成你自己的用户名
const std::string LOG_FILE_PATH = "/home/king/PreGME_ROS2/Plot/plot_clik_custom/clik_custom_log.txt";

extern int number;
int number = 0;

clikRos::clikRos()
: rclcpp::Node("clik")
{
        using std::placeholders::_1;

        // 【订阅】无人机当前状态 - 来自飞控
        //  本话题来自飞控(通过Mavros功能包 /plugins/sys_status.cpp)
        state_sub = this->create_subscription<mavros_msgs::msg::State>("mavros/state", 10, std::bind(&clikRos::state_obtain, this, _1));

        // 【订阅】无人机当前位置 坐标系:ENU系 （此处注意，所有状态量在飞控中均为NED系，但在ros中mavros将其转换为ENU系处理。所以，在ROS中，所有和mavros交互的量都为ENU系）
        //  本话题来自飞控(通过Mavros功能包 /plugins/local_position.cpp读取), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
        position_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", rclcpp::QoS(10).best_effort(), std::bind(&clikRos::pos_obtain, this, _1));

        velocity_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>("mavros/local_position/velocity_local", rclcpp::QoS(10).best_effort(), std::bind(&clikRos::vel_obtain, this, _1));
        // 【订阅】无人机当前欧拉角 坐标系:ENU系
        //  本话题来自飞控(通过Mavros功能包 /plugins/imu.cpp读取), 对应Mavlink消息为ATTITUDE (#30), 对应的飞控中的uORB消息为vehicle_attitude.msg
        attitude_sub = this->create_subscription<sensor_msgs::msg::Imu>("mavros/imu/data", rclcpp::QoS(10).best_effort(), std::bind(&clikRos::att_obtain, this, _1));
        // 【订阅】无人机真实姿态目标
        //  本话题来自 mavros setpoint_raw 插件对 PX4 ATTITUDE_TARGET 的回传，
        //  对应的是 PX4 内部最终使用的姿态目标，而不是外部 PoseStamped 姿态命令。
        attitude_sp_sub = this->create_subscription<mavros_msgs::msg::AttitudeTarget>("mavros/setpoint_raw/target_attitude", rclcpp::QoS(10).best_effort(), std::bind(&clikRos::att_sp_obtain, this, _1));
        // mavlink_from_sub = this->create_subscription<mavros_msgs::msg::Mavlink>("mavlink/from", 10, std::bind(&clikRos::debug_array_obtain, this, _1));      //ROS1订阅话题
        mavlink_from_sub = this->create_subscription<mavros_msgs::msg::Mavlink>("/uas1/mavlink_source", rclcpp::QoS(10).best_effort().durability_volatile(), std::bind(&clikRos::debug_array_obtain, this, _1)); //ROS2订阅话题
        

        // 【订阅】遥控器的操纵 
        //  本话题来自飞控(通过Mavros功能包 /plugins/manulcontrol.cpp读取), 对应Mavlink消息为ATTITUDE (#30), 对应的飞控中的uORB消息为vehicle_attitude.msg
        rcin_sub = this->create_subscription<mavros_msgs::msg::RCIn>("mavros/rc/in", 10, std::bind(&clikRos::rcin_obtain, this, _1)); 
        
        // 【订阅】机械臂的末端位置 - 来自vicon 坐标系 地面绝对坐标系（vicon）
        // 本话题来自vicon_bridge
        Ti5_arm_EE_sub = this->create_subscription<geometry_msgs::msg::TransformStamped>("vicon/Gamma_arm_EE/Gamma_arm_EE", 10, std::bind(&clikRos::Ti5_arm_EE_obtain, this, _1));

        // Ti5_arm_EE_twist_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>("/vrpn_client_node/Ti5_arm_EE/twist", 10, std::bind(&clikRos::Ti5_arm_EE_obtain_twist, this, _1));
        //修改：vicon的末端速度话题在 /vrpn_client_node/cart/twist
        Ti5_arm_EE_twist_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>("/vrpn_client_node/cart/twist", 10, std::bind(&clikRos::Ti5_arm_EE_obtain_twist, this, _1));


        // joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>("arm/joint_feedback", 10, std::bind(&clikRos::JointStateCallBack, this, _1));
        joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>("/gamma_arm/joint_states", 10, std::bind(&clikRos::JointStateCallBack, this, _1));


        // 【订阅】无人机的导航状态
        //  本话题来自于 off_mission
        action_sub = this->create_subscription<clik::msg::Action>("navigator/vehicle_action", 10, std::bind(&clikRos::vehicle_action_callback, this, _1)); 


        // 【发布】机械臂末端位置指令
        joint_ctrl_pub = this->create_publisher<sensor_msgs::msg::JointState>("arm/joint_control", 10);

        Delta_pub = this->create_publisher<clik::msg::PositionPub>("control_signal/pos_pub", 10);
        
        // 【发布】gripper串口指令
        gripper_pub = this->create_publisher<std_msgs::msg::String>("/gripper_command", 10);

        // 【发布】 飞行平台的轨迹指令和偏航指令 to 飞控
        local_pos_pub = this->create_publisher<mavros_msgs::msg::PositionTarget>("online_target", 10);
        // mavlink_raw_pub_ = this->create_publisher<mavros_msgs::msg::Mavlink>("mavlink/to", 10);      //ROS1发布话题
        mavlink_raw_pub_ = this->create_publisher<mavros_msgs::msg::Mavlink>("/uas1/mavlink_sink", rclcpp::QoS(10).best_effort().durability_volatile()); //ROS2发布话题

        
        // local_pos_pub = nh.advertise<geometry_msgs::msg::PoseStamped>("online_target",10); 

        // 【客户端】修改机械臂模式
        manipulator_client = this->create_client<clik::srv::ManipulatorMode>("control_signal/command_mode");

        //【客户端】 修改飞机飞行模式 发送给飞控
        set_mode_client = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        set_message_interval_client = this->create_client<mavros_msgs::srv::MessageInterval>("mavros/set_message_interval");

        // 【客户端】轨迹生成
        traj_solver_client  = this->create_client<clik::srv::TrajSolverMsg>("trajectory_solver");
        // 【客户端】当前位置指令获取
        traj_out_client     = this->create_client<clik::srv::TrajOutMsg>("trajectory_result");    

        // 初始化机械臂工作状态
        manipulator_mode = mod_sleep;

        flying_configration_.resize(6);
        flying_configration_ << 0.0, M_PI/3, 2*M_PI/3,0.0, -M_PI/3, - M_PI/2;  //飞行构型
        
        land_configration_.resize(6);
        land_configration_.setZero();  //着陆构型

        Assemble_pos << 0.40866, -0.0100795, 0.106783; // 标定的位置
        Assemble_rotation << -0.00650564,  -0.00164351 , 0.996607,
                             -0.00559075,  0.99941 , -0.0146378,
                             -0.990284,  0.00156677 , -0.0118127; //正交化之后的机械臂安装坐标系旋转矩阵

        adm_pos_err_ = 0.0; // 导纳控制
        adm_vel_err_ = 0.0;

        vel_error_int_EE.setZero();
        last_vehicle_position_cmd.setZero();
        is_phase8_initialized = false;

        position_EE.setZero();
        velocity_EE.setZero();

        desired_pE.setZero();; // 期望的末端位置
        desired_vE.setZero();;
        target_Re.Identity();

       

          char *buffer;
        //也可以将buffer作为输出参数
        if((buffer = getcwd(NULL, 0)) == NULL)
        {
            printf("getcwd error\n");
        }
        else
        {
            printf("%s\n", buffer);
            free(buffer);
        }
        //start log4z  
        zsummer::log4z::ILog4zManager::getRef().start();

        // 【新增】每次启动时清空或新建自定义的 txt 日志文件
        std::ofstream init_log(LOG_FILE_PATH, std::ios::out | std::ios::trunc);
        if (init_log.is_open()) {
            init_log << "CLIK Data Log Started\n";
            init_log.close();
        } else {
            LOGFMTD("Failed to open log file at: %s", LOG_FILE_PATH.c_str());
        }
}
    
     
        // 四元素转欧拉角
    Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q)
    {
        float quat[4];
        quat[0] = q.w();
        quat[1] = q.x();
        quat[2] = q.y();
        quat[3] = q.z();

        Eigen::Vector3d ans;
        ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
        ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
        ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
        return ans;
    }

    // 欧拉角转旋转矩阵
    Eigen::Matrix3d clikRos::euler_to_rotation(const Eigen::Vector3d& euler)
    {
        double phi=euler(0);
        double theta=euler(1);
        double psi=euler(2);

        Eigen:: Matrix3d RX;
        Eigen:: Matrix3d RY;
        Eigen:: Matrix3d RZ;
    
        RX<<1,0,0,
            0,cos(phi),-sin(phi),
            0,sin(phi),cos(phi);
        RY<<cos(theta),0,sin(theta),
            0,1,0,
            -sin(theta),0,cos(theta);
        RZ<<cos(psi),-sin(psi),0,
            sin(psi),cos(psi),0,
            0,0,1;
        Eigen:: Matrix3d rotation=RZ*RY*RX;
        return rotation;
    }
        
     
     // 【回调函数】 飞机状态 
    void  clikRos::state_obtain(const mavros_msgs::msg::State::ConstSharedPtr &msg)
    {
        current_state = *msg;
    }

    // 【回调函数】
    void clikRos::vehicle_action_callback(const clik::msg::Action::ConstSharedPtr& msg)
    {
        cur_action = *msg;
    }

    // 【回调函数】机械臂的末端位置 - 来自 Vicon 坐标系（地面绝对坐标系）,转换到NED
    void clikRos::Ti5_arm_EE_obtain(const geometry_msgs::msg::TransformStamped::ConstSharedPtr& message_holder)
    {
        // 位置提取
        position_EE(0) = message_holder->transform.translation.x;
        position_EE(1) = -message_holder->transform.translation.y;
        position_EE(2) = -message_holder->transform.translation.z;

        // 四元数提取
        Eigen::Quaterniond q(
            message_holder->transform.rotation.w,
            message_holder->transform.rotation.x,
            message_holder->transform.rotation.y,
            message_holder->transform.rotation.z
        );

        // 转换为旋转矩阵
        Re = q.toRotationMatrix();
        Eigen:: Matrix3d Re_to_NED; // 世界坐标系到NED的转换矩阵
        Re_to_NED << 1, 0, 0,
                     0, -1, 0,
                     0, 0, -1;

        Re = Re_to_NED * Re;
    }


    // 【回调函数】机械臂的末端速度 - 来自 Vicon 坐标系（地面绝对坐标系）,转换到NED

    void clikRos::Ti5_arm_EE_obtain_twist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg) {
        velocity_EE(0) = msg->twist.linear.x;
        velocity_EE(1) = -msg->twist.linear.y;
        velocity_EE(2) = -msg->twist.linear.z;
    }

        // 【回调函数】 姿态角&角速度 ENU->NED
    void  clikRos::att_obtain(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    {
        Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        //Transform the Quaternion to euler Angles
        Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);
        flightStateData.phi    = euler_fcu[0];
        flightStateData.theta  = - euler_fcu[1];
        flightStateData.psi    = - euler_fcu[2] ;
        
        double x = msg->orientation.x;
        double y = msg->orientation.y;
        double z = msg->orientation.z;

        flightStateData.wx = msg->angular_velocity.x;
        flightStateData.wy = - msg->angular_velocity.y;
        flightStateData.wz = - msg->angular_velocity.z;
    }


    // 【回调函数】 PX4 内部姿态目标 ENU->NED
    void clikRos::att_sp_obtain(const mavros_msgs::msg::AttitudeTarget::ConstSharedPtr &msg)
    {
        Eigen::Quaterniond q_target(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        Eigen::Vector3d euler_target = quaternion_to_euler(q_target);
        attitude_sp(0) = euler_target[0];
        attitude_sp(1) = -euler_target[1];
        attitude_sp(2) = -euler_target[2];
        attitude_sp_received_ = true;
    }

    void clikRos::debug_array_obtain(const mavros_msgs::msg::Mavlink::ConstSharedPtr &msg)
    {
        if (msg->msgid != MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY || msg->payload64.empty()) {
            return;
        }

        mavlink_message_t mav_msg{};
        mav_msg.magic = msg->magic;
        mav_msg.len = msg->len;
        mav_msg.incompat_flags = msg->incompat_flags;
        mav_msg.compat_flags = msg->compat_flags;
        mav_msg.seq = msg->seq;
        mav_msg.sysid = msg->sysid;
        mav_msg.compid = msg->compid;
        mav_msg.msgid = msg->msgid;
        mav_msg.checksum = msg->checksum;

        const size_t payload64_len = (msg->len + 7) / 8;

        for (size_t i = 0; i < payload64_len && i < msg->payload64.size(); ++i) {
            mav_msg.payload64[i] = msg->payload64[i];
        }

        mavlink_debug_float_array_t dbg{};
        mavlink_msg_debug_float_array_decode(&mav_msg, &dbg);

        const bool is_coord_px4 = strncmp(dbg.name, "coord_px4", sizeof(dbg.name)) == 0;
        const bool is_coord_att_legacy = strncmp(dbg.name, "coord_att", sizeof(dbg.name)) == 0;

        if (is_coord_px4) {
            px4_pos_comp_ << dbg.data[0], dbg.data[1], dbg.data[2];
            px4_pos_eso_ << dbg.data[3], dbg.data[4], dbg.data[5];
            px4_pos_total_ << dbg.data[6], dbg.data[7], dbg.data[8];
            px4_att_comp_ << dbg.data[9], dbg.data[10], dbg.data[11];
            px4_att_eso_ << dbg.data[12], dbg.data[13], dbg.data[14];
            px4_att_total_ << dbg.data[15], dbg.data[16], dbg.data[17];
            px4_pos_u_nominal_ << dbg.data[18], dbg.data[19], dbg.data[20];
            px4_att_u_nominal_ << dbg.data[21], dbg.data[22], dbg.data[23];
            px4_coordinate_debug_valid_ = true;
        } else if (is_coord_att_legacy) {
            // Backward compatibility for the old 9-float layout published by PX4.
            px4_pos_comp_ << dbg.data[0], dbg.data[1], dbg.data[2];
            px4_pos_eso_ << dbg.data[3], dbg.data[4], dbg.data[5];
            px4_pos_total_ = px4_pos_comp_ + px4_pos_eso_;
            px4_att_comp_ << dbg.data[6], dbg.data[7], dbg.data[8];
            px4_att_eso_.setZero();
            px4_att_total_ = px4_att_comp_;
            px4_pos_u_nominal_.setZero();
            px4_att_u_nominal_.setZero();
            px4_coordinate_debug_valid_ = true;
        }
    }

    void clikRos::ensureAttitudeTargetStream()
    {
        if (!current_state.connected || (attitude_target_stream_configured_ && attitude_sp_received_)) {
            return;
        }

        if (last_attitude_stream_request_.nanoseconds() != 0 &&
            (this->now() - last_attitude_stream_request_) < rclcpp::Duration::from_seconds(2.0)) {
            return;
        }

        last_attitude_stream_request_ = this->now();

        if (!set_message_interval_client->service_is_ready()) {
            ROS_WARN_THROTTLE(5.0, "CLIK: waiting for /mavros/set_message_interval service...");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();
        request->message_id = 83;   // MAVLINK_MSG_ID_ATTITUDE_TARGET
        request->message_rate = 50; // Hz

        set_message_interval_client->async_send_request(
            request,
            [this](rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedFuture future) {
                if (future.get()->success) {
                    attitude_target_stream_configured_ = true;
                    ROS_INFO_THROTTLE(5.0, "CLIK: requested PX4 ATTITUDE_TARGET stream at 50 Hz");
                } else {
                    ROS_WARN_THROTTLE(5.0, "CLIK: failed to request PX4 ATTITUDE_TARGET stream");
                }
            });
    }

    bool clikRos::callManipulatorMode(int64_t mode)
    {
        if (!manipulator_client->service_is_ready()) {
            ROS_WARN_THROTTLE(5.0, "CLIK: waiting for control_signal/command_mode service...");
            return false;
        }

        auto request = std::make_shared<clik::srv::ManipulatorMode::Request>();
        request->mode = mode;
        auto future = manipulator_client->async_send_request(request);
        const auto result = rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), future, std::chrono::milliseconds(200));
        return result == rclcpp::FutureReturnCode::SUCCESS && future.get()->result;
    }


    // 【回调函数】 rcin
    void  clikRos::rcin_obtain(const mavros_msgs::msg::RCIn::ConstSharedPtr& msg)
    {
        std::lock_guard<std::mutex> lock(rc_mutex_);
        m_rcin_ = *msg;
        //ROS_INFO("CLIK:1111");
        //ROS_INFO("RC_IN chanlle = %u\n",m_rcin_.channels.at(9));
    }

      void  clikRos::pos_obtain(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
    {
        current_local_pos = *msg;
        //待修改：
        flightStateData.x =  msg->pose.position.x;
        flightStateData.y = -  msg->pose.position.y;
        flightStateData.z = - msg->pose.position.z;
    }


    // 【回调函数】 位置 ENU->NED
    void  clikRos::vel_obtain(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg)
    {
        flightStateData.vx =  msg->twist.linear.x;
        flightStateData.vy = -  msg->twist.linear.y;
        flightStateData.vz = - msg->twist.linear.z;
    }


    // 机械臂statae回调函数，读取了机械臂6个关节角和关节速度
    void clikRos::JointStateCallBack(const sensor_msgs::msg::JointState::ConstSharedPtr& msg_p){
    double arm_pos_[6] = {0};
    double arm_vel_[6] = {0};
    static bool joint_filter_initialized = false;
    static double filtered_arm_pos[6] = {0};
    static rclcpp::Time last_joint_sample_time;
    
    //arm current 电流
    memcpy((uint8_t*)&arm_pos_[0], msg_p.get()->position.data(), 8 * 6);
    memcpy((uint8_t*)&arm_vel_[0], msg_p.get()->velocity.data(), 8 * 6);

    // 在关节角进入补偿/质心计算前做一阶低通滤波，抑制高频抖动
    rclcpp::Time sample_time = msg_p->header.stamp;
    if (sample_time.nanoseconds() == 0) {
        sample_time = this->now();
    }

    if (!joint_filter_initialized) {
        for (int i = 0; i < 6; ++i) {
            filtered_arm_pos[i] = arm_pos_[i];
        }
        joint_filter_initialized = true;
    } else {
        const double dt = std::max((sample_time - last_joint_sample_time).seconds(), 1e-3);
        const double tau = 0.05; // about 3 Hz cutoff at 50 Hz update
        const double alpha_raw = dt / (tau + dt);
        const double alpha = std::max(0.0, std::min(alpha_raw, 1.0));

        for (int i = 0; i < 6; ++i) {
            filtered_arm_pos[i] += alpha * (arm_pos_[i] - filtered_arm_pos[i]);
        }
    }
    last_joint_sample_time = sample_time;

    manipulatorData.clik_joint_pos[0] = filtered_arm_pos[0];
    manipulatorData.clik_joint_pos[1] = filtered_arm_pos[1];
    manipulatorData.clik_joint_pos[2] = filtered_arm_pos[2];
    manipulatorData.clik_joint_pos[3] = filtered_arm_pos[3];
    manipulatorData.clik_joint_pos[4] = filtered_arm_pos[4];
    manipulatorData.clik_joint_pos[5] = filtered_arm_pos[5];

    manipulatorData.clik_joint_vel[0] = (double)arm_vel_[0];
    manipulatorData.clik_joint_vel[1] = (double)arm_vel_[1];
    manipulatorData.clik_joint_vel[2] = (double)arm_vel_[2];
    manipulatorData.clik_joint_vel[3] = (double)arm_vel_[3];
    manipulatorData.clik_joint_vel[4] = (double)arm_vel_[4];
    manipulatorData.clik_joint_vel[5] = (double)arm_vel_[5];

}

    bool clikRos::isManupulator(const mavros_msgs::msg::RCIn& rcin)
    {
        return ((rcin.channels.size()>=7) && (rcin.channels.at(9)>1500));//机械臂收放 通道10
    }

    bool clikRos::isCoordinate(const mavros_msgs::msg::RCIn& rcin)
    {
        return (rcin.channels.size()>=7 && rcin.channels.at(8)>1500);//进入协同模式 通道待定
    }


void clikRos::checkArmingState()
{
    if(!current_state.armed && !reset_CLIK_flag_)
    {
        reset_CLIK_flag_ = true;
        ROS_INFO("-----------------------------------------------");
        ROS_INFO("Disarmed! Will restart the mission next time armed!\n");
    }
    if(current_state.armed && reset_CLIK_flag_)
    {
        reset_CLIK_flag_ = false;
        ROS_INFO("First time to Arm");
        resetCLIK();
    }
}


void clikRos::setOnGroundOrigin()
{
     m_ref_origin_.pose.position.x = current_local_pos.pose.position.x;
     m_ref_origin_.pose.position.y = current_local_pos.pose.position.y;
     m_ref_origin_.pose.position.z = current_local_pos.pose.position.z;
     ground_origin_position_initialized_flag_ = true;
     ROS_INFO("refernce origin set at position x  %f y  %f z %f " , m_ref_origin_.pose.position.x , 
                                                                       m_ref_origin_.pose.position.y , 
                                                                      m_ref_origin_.pose.position.z);
}
void clikRos::resetCLIK()
{
    on_off_manipulator_flag_ = false;// false：摆臂垂直 true：摆臂水平
    coordinate_running_flag_ = false;
    coordinate_arm_task_initialized_ = false;
    coordinate_stage_ = 0;
    coordinate_stage_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    coordinate_goal_reached_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    resetOnGroundOrigin();
}
void clikRos::resetOnGroundOrigin()
{
    ground_origin_position_initialized_flag_ = false;
    m_ref_origin_.pose.position.x = 0;
    m_ref_origin_.pose.position.y = 0;
    m_ref_origin_.pose.position.z = 0;
}

void clikRos::resetCoordinateIni()
{
    //待修改：
    m_coordinate_contr_ini.x = flightStateData.x;
    m_coordinate_contr_ini.y = flightStateData.y;
    m_coordinate_contr_ini.z = flightStateData.z;
    
}


void clikRos::putDowndMnipulator()
{
    if (!on_off_manipulator_flag_ && !coordinate_flag_ && (manipulator_mode != mod_shrink))
    {
        /* 放下机械臂，不协调*/
        first_off_manipulator = true;
        last_off_manipulator =  this->now();
        manipulator_mode = mod_shrink;
    }
    if (first_off_manipulator && (this->now() - last_off_manipulator > rclcpp::Duration::from_seconds(1.0)))
    {
       
        first_off_manipulator = false;
        Delta_mode_request.mode = 0;

        Eigen::VectorXd q_cmd(6);
        q_cmd << 0.0, 0.0, 0.0,0.0, 0.0, 0.0;
        publishManipulatorJoints(q_cmd);

    }
}

void clikRos::putUpMnipulator()
{
    if (on_off_manipulator_flag_&&!coordinate_flag_ && !(manipulator_mode==mod_prepare) )
    {
        //收起机械臂， 不协调
        first_on_manipulator = true;
        last_on_manipulator =  this->now();
        manipulator_mode = mod_prepare; 
    }

    if (first_on_manipulator && (this->now() - last_on_manipulator > rclcpp::Duration::from_seconds(1.0)))
    {
        first_on_manipulator = false;
        Delta_mode_request.mode = 1;
        Eigen::VectorXd q_cmd = flying_configration_;
        publishManipulatorJoints(q_cmd);

    }
}

 void clikRos::checkCoordinateState()
{
    if(!coordinate_flag_ && !reset_coordinate_flag_)
    {
        reset_coordinate_flag_ = true;
        coordinate_running_flag_ = false;
        coordinate_arm_task_initialized_ = false;
        coordinate_stage_ = 0;
        coordinate_stage_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        coordinate_goal_reached_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        ROS_INFO("-----------------------------------------------");
        ROS_INFO("Discoordinated! Will restart the mission next time coordinated!\n");
    }
    if( coordinate_flag_ && reset_coordinate_flag_)
    {
        reset_coordinate_flag_ = false;
        coordinate_running_flag_ = false;
        coordinate_arm_task_initialized_ = false;
        coordinate_stage_ = 0;
        coordinate_stage_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        coordinate_goal_reached_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        ROS_INFO("First time to Coordinate");
        resetCoordinateIni();//在这里要改变起CLIK的初值
        last_coordinate = this->now();
    }
}


void clikRos::publishManipulatorJoints(Eigen::VectorXd desired_theta)
{
    sensor_msgs::msg::JointState joint_ctrl_msg;
    joint_ctrl_msg.position = {
        desired_theta(0), desired_theta(1), desired_theta(2),
        desired_theta(3), desired_theta(4), desired_theta(5)
    };
    joint_ctrl_msg.header.frame_id = "joint_ctrl_frame";
    joint_ctrl_msg.header.stamp = this->now();
    joint_ctrl_pub->publish(joint_ctrl_msg);

}

//计算系统质心位置
void clikRos::updateSystemComState()
{
    Eigen::VectorXd current_theta(6);

    for (int i = 0; i < 6; ++i) {
        current_theta(i) = manipulatorData.clik_joint_pos[i];
    }

    com_state_in_body_ = arm_uav_kinematics::computeComStateInBody(current_theta);
    system_com_B_ = com_state_in_body_.p_C_B;
    arm_com_B_ = com_state_in_body_.p_C_arm_B;

    // LOGFMTD("system_com_B_x %f", static_cast<double>(system_com_B_(0)));
    // LOGFMTD("system_com_B_y %f", static_cast<double>(system_com_B_(1)));
    // LOGFMTD("system_com_B_z %f", static_cast<double>(system_com_B_(2)));

    // ROS_INFO_STREAM_THROTTLE(
    //     1.0,
    //     "arm q[B]: " << current_theta.transpose()
    //     << " | arm_com_B: " << arm_com_B_.transpose()
    //     << " | system_com_B: " << system_com_B_.transpose()
    //     << " | body_euler[B]: " << flightStateData.phi << " "
    //     << flightStateData.theta << " " << flightStateData.psi
    //     << " | body_w[B]: " << flightStateData.wx << " "
    //     << flightStateData.wy << " " << flightStateData.wz);
}

void clikRos::publishSystemComToPx4()
{
    if (mavlink_raw_pub_->get_subscription_count() == 0) {
        return;
    }

    mavlink_message_t mav_msg{};
    float gs_payload[3] = {
        static_cast<float>(system_com_B_(0)),
        static_cast<float>(system_com_B_(1)),
        static_cast<float>(system_com_B_(2))
    };

    mavlink_msg_mav_gs_pack(
        255,
        MAV_COMP_ID_ONBOARD_COMPUTER,
        &mav_msg,
        gs_payload);

    mavros_msgs::msg::Mavlink ros_msg;
    ros_msg.header.stamp = this->now();
    ros_msg.framing_status = mavros_msgs::msg::Mavlink::FRAMING_OK;
    ros_msg.magic = mav_msg.magic;
    ros_msg.len = mav_msg.len;
    ros_msg.incompat_flags = mav_msg.incompat_flags;
    ros_msg.compat_flags = mav_msg.compat_flags;
    ros_msg.seq = mav_msg.seq;
    ros_msg.sysid = mav_msg.sysid;
    ros_msg.compid = mav_msg.compid;
    ros_msg.msgid = mav_msg.msgid;
    ros_msg.checksum = mav_msg.checksum;

    const size_t payload64_len = (mav_msg.len + 7) / 8;
    ros_msg.payload64.assign(mav_msg.payload64, mav_msg.payload64 + payload64_len);
    ros_msg.signature.clear();
    mavlink_raw_pub_->publish(ros_msg);
}


bool clikRos :: isNearSingular(const Eigen::MatrixXd& J, double threshold ) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
    double min_singular = svd.singularValues().minCoeff();
    return min_singular < threshold;
}

// 机械臂控制函数
Eigen::VectorXd clikRos::compute_desired_theta(const Eigen::Matrix3d &rotation_body,
                       const Eigen::VectorXd& current_theta, 
                       const Eigen::Vector3d& desired_pE,
                       const Eigen::Matrix3d& target_Re,
                       double dt) {
    // -----------------------------------------------------------
    // 1. 位置误差计算与安全熔断
    // -----------------------------------------------------------
    Eigen::Vector3d Delta_pE = desired_pE - position_EE;
    double delta_p_norm = Delta_pE.norm();  

    // 安全熔断：位置误差过大直接锁死
    if (delta_p_norm > 0.3) {
        LOGFMTD("Position error too large (%.3f), locking arm for safety.", delta_p_norm);
        return current_theta; // 直接返回当前角度，保持不动
    }

    // -----------------------------------------------------------
    // 2. 误差向量构建
    // -----------------------------------------------------------

    // 姿态误差计算
    //修改：
    Eigen::Matrix3d target_Re_world = rotation_body * Assemble_rotation * target_Re; // 将目标姿态转换到NED世界系下
    // Eigen::Matrix3d target_Re_world = Assemble_rotation * target_Re; // 将目标姿态转换到NED世界系下
    Eigen::Matrix3d R_error = Re.transpose() * target_Re_world; 
    Eigen::AngleAxisd aa_error(R_error);    

    double max_angle_rad  = 5 * M_PI / 180.0;   //每次执行角度的最大步长，不是电机的步长，而是末端向目标位置移动时，单次允许执行的最大角度
    double step_angle = std::min( aa_error.angle(), max_angle_rad);

    Eigen::AngleAxisd aa_interp(step_angle, aa_error.axis());
    Eigen::Matrix3d R_step = Re * aa_interp.toRotationMatrix();
    Eigen::Matrix3d R_err = 0.5 * (R_step * Re.transpose() - Re * R_step.transpose());

    Eigen::Vector3d e_rot;
    e_rot << R_err(2,1), R_err(0,2), R_err(1,0); 
   
    Eigen::VectorXd error(6);
    error.head<3>() = Delta_pE;
    error.tail<3>() = e_rot;
    // -----------------------------------------------------------
    // 3. 求解关节增量
    // -----------------------------------------------------------
    Eigen::Matrix<double, 6, 6> J = get_jacobian(current_theta);

    // 计算从机械臂基座到世界坐标系的旋转矩阵
    Eigen::Matrix3d R_base_to_world = rotation_body * Assemble_rotation;

    // 分别旋转雅可比矩阵的线速度部分(前3行)和角速度部分(后3行)
    J.topRows(3)    = R_base_to_world * J.topRows(3);
    J.bottomRows(3) = R_base_to_world * J.bottomRows(3);

    double lambda = 0.1;
    Eigen::Matrix<double, 6, 6> J_pinv = J.transpose() * (J * J.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6,6)).inverse();

    double k = 2;  
    // 计算出理论上需要的关节角度增量 delta_theta
    Eigen::VectorXd delta_theta = k * J_pinv * error * dt;

    // 计算出理论上的下一时刻角度 (Candidate)
    Eigen::VectorXd next_theta = desired_theta + delta_theta;

    // -----------------------------------------------------------
    // 4. 关节限位约束 (Satuation / Clamping)
    // -----------------------------------------------------------
    
    // 定义关节限位
    double limit_J5 = 150.0 * M_PI / 180.0;
    Eigen::VectorXd q_min(6), q_max(6);
    //修改：
    q_min << -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI;
    q_max <<  M_PI,  M_PI,  M_PI,  M_PI,  M_PI,  M_PI;

    // -----------------------------------------------------------
    // 4. 关节限位约束 & 5. 更新输出 (合并简化)
    // -----------------------------------------------------------
    
    // 遍历检查每个关节
    for (int i = 0; i < 6; ++i) {
        // 如果任意一个关节超过上限 或 低于下限
        if (next_theta(i) > q_max(i) || next_theta(i) < q_min(i)) {
            // 触发限位：直接“短路”，返回旧的 desired_theta (维持原状)
            // LOGFMTD("Joint limit reached at joint %d, holding position.", i);
            return desired_theta; 
        }
    }

    // 如果代码能运行到这里，说明没有触发任何限位
    desired_theta = next_theta; // 更新状态
    return desired_theta;       // 返回新状态
}


// ---------------------------------------------------------------------
// 辅助函数：计算机械臂关节增量预测 (只负责计算 dq 和 预测的 q)
// ---------------------------------------------------------------------
Eigen::VectorXd clikRos::compute_arm_prediction(const Eigen::VectorXd& error_vec_world,
                                                const Eigen::MatrixXd& J_arm_world,
                                                double dt) {
    // DLS 参数
    double lambda = 0.05;
    Eigen::MatrixXd I6 = Eigen::MatrixXd::Identity(6, 6);
    
    // J_inv = J.T * (J * J.T + lambda^2 * I)^-1
    Eigen::MatrixXd J_dls_inv = J_arm_world.transpose() * (J_arm_world * J_arm_world.transpose() + lambda * lambda * I6).inverse();

    // 计算关节速度 dq = J_inv * v_error
    Eigen::VectorXd dq = J_dls_inv * error_vec_world;

    // 关节速度限幅 (Safety)
    //修改：
    // 建立一个6维向量，分别存放 J1 到 J6 的最大允许角速度 (单位: rad/s)
    Eigen::VectorXd max_vel_limits(6);
    
    // 【在这里填入你真实机械臂电机的限速参数】
    // 提示：1 rad/s 约等于 57.3 度/s
    max_vel_limits << 
        10.47,    // Joint 1: 底座偏航 (通常最重，需要限速较小)
        10.47,    // Joint 2: 大臂俯仰
        10.47,    // Joint 3: 肘部
        50.26,    // Joint 4: 腕部偏航
        50.26,    // Joint 5: 腕部俯仰
        50.26;    // Joint 6: 末端法兰 (负载最轻，可以转最快)
    // double max_arm_vel_rad = 10.47;
    for (int i = 0; i < 6; ++i) {
        dq(i) = std::max(std::min(dq(i), max_vel_limits(i)), -max_vel_limits(i));
    }

    // [关键修正]：使用 "上一时刻指令值" + 增量 = "下一时刻指令值"
    // 防止使用测量值导致的漂移
    return desired_theta + dq * dt;
}

// ---------------------------------------------------------------------
// 主控制函数
// ---------------------------------------------------------------------
void clikRos::compute_clik_control(const Eigen::Matrix3d& rotation_body, // 机身姿态(测量值)
                                   const Eigen::VectorXd& current_theta, // 机械臂角度(测量值): 仅用于算雅可比和FK
                                   const Eigen::Vector3d& pos_body,      // 机身位置(测量值): 仅用于算误差
                                   const Eigen::Vector3d& desired_pE_in,         // 目标末端位置
                                   const Eigen::Matrix3d& target_Re,          // 目标末端姿态
                                   double dt,
                                   // --- [输入/输出] 状态变量 (History) ---
                                   Eigen::VectorXd& q_next,            // In: 上次指令 | Out: 本次指令
                                   Eigen::Vector3d& vel_base_opt              // Out: 本次速度指令
                                   ) 
{
    
    vel_base_opt.setZero();
    // 定义关节限位
    double limit_J5 = 150.0 * M_PI / 180.0;
    Eigen::VectorXd q_min(6), q_max(6);
    //修改：
    q_min << -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI;
    q_max <<  M_PI,  M_PI,  M_PI,  M_PI,  M_PI,  M_PI;

    // ---------------------------------------------------------------------
    // 1. 状态更新与误差计算 (World Frame)
    // ---------------------------------------------------------------------
    // 注意：这里需要用"测量值"来计算当前真实的物理误差
    Eigen::Vector3d Delta_pE = desired_pE_in - position_EE; 
 
    double delta_p_norm = Delta_pE.norm();

    // 死区与熔断
    const double pos_threshold = 0.005;
    // if (delta_p_norm < pos_threshold) {
    //     return; // 误差极小，不更新指令，维持上一时刻的 pos_base_opt 和 desired_theta
    // }
    if (delta_p_norm > 0.3) {
        // 误差过大，锁死，不更新指令
        // LOGFMTD("Position error too large (%.3f), locking arm for safety.", delta_p_norm);
        return; 
    }

    // ---------------------------------------------------------------------
    // 2. 姿态误差 (World Frame)
    // ---------------------------------------------------------------------
    //修改：
    Eigen::Matrix3d target_Re_world = rotation_body * Assemble_rotation * target_Re; // 将目标姿态转换到NED世界系下
    Eigen::Matrix3d R_error = Re.transpose() * target_Re_world; // Re 是基于测量值的当前末端姿态
    Eigen::AngleAxisd aa_error(R_error);    
    double step_angle = std::min(aa_error.angle(), 5.0 * M_PI / 180.0);
    Eigen::AngleAxisd aa_interp(step_angle, aa_error.axis());
    Eigen::Matrix3d R_step = Re * aa_interp.toRotationMatrix();
    Eigen::Matrix3d R_err_mat = 0.5 * (R_step * Re.transpose() - Re * R_step.transpose());

    Eigen::Vector3d e_rot;
    e_rot << R_err_mat(2,1), R_err_mat(0,2), R_err_mat(1,0); 

    // ---------------------------------------------------------------------
    // 3. 基座速度规划 (Macro Layer)
    // ---------------------------------------------------------------------
    Eigen::Vector3d r_EB_current = position_EE - pos_body; // 使用测量值计算当前相对距离
    Eigen::Vector3d r_EB_des = Eigen::Vector3d::Zero(); 
    
    double k_base = 0.5;
    Eigen::Vector3d v_base_opt = k_base * (r_EB_current - r_EB_des);

    // [基座约束]
    v_base_opt(0) = 0.0; // X轴锁定

    // double limit_pos_adjust = 0.06;
    double limit_pos_adjust = 0.10; // 位置误差超过这个值时，禁止基座继续朝那个方向移动
    // double limit_vel_adjust = 0.02;
    double limit_vel_adjust = 0.03; // 基座速度的最大调整量
    Eigen::Vector3d base_ideal_pos = position_EE - r_EB_des;
    Eigen::Vector3d base_drift = pos_body - base_ideal_pos; // 漂移量用测量值判断

    for (int i = 1; i < 3; ++i) { 
        v_base_opt(i) = std::max(std::min(v_base_opt(i), limit_vel_adjust), -limit_vel_adjust);
        if (base_drift(i) > limit_pos_adjust && v_base_opt(i) > 0) {
        v_base_opt(i) = 0.0; }
        else if (base_drift(i) < -limit_pos_adjust && v_base_opt(i) < 0) {
        v_base_opt(i) = 0.0; }

    }

    // ---------------------------------------------------------------------
    // 4. 准备雅可比 (World Frame)
    // ---------------------------------------------------------------------
    // 雅可比必须用"测量角度"计算，反映当前真实构型
    Eigen::Matrix<double, 6, 6> J_body = get_jacobian(current_theta);
    Eigen::Matrix<double, 6, 6> J_world = J_body;

    Eigen::Matrix3d R_base_to_world = rotation_body* Assemble_rotation;
    J_world.topRows(3)    = R_base_to_world * J_body.topRows(3);
    J_world.bottomRows(3) = R_base_to_world * J_body.bottomRows(3);

    // ---------------------------------------------------------------------
    // 5. 第一次预测 (含基座补偿)
    // ---------------------------------------------------------------------
    double k_q = 2.0;
    Eigen::VectorXd error_vec_total(6);
    // 误差 = (目标 - 测量) - 基座速度 + 姿态误差
    error_vec_total.head<3>() = -v_base_opt + k_q * Delta_pE; 
    error_vec_total.tail<3>() = k_q * e_rot;

    // [关键]: 传入 desired_theta (上一时刻指令) 用于积分
    Eigen::VectorXd q_next_predict = compute_arm_prediction( error_vec_total, J_world, dt);

    // ---------------------------------------------------------------------
    // 6. 饱和检测与策略选择
    // ---------------------------------------------------------------------
    bool is_saturated = false;
    for (int i = 0; i < 6; ++i) {
        if (q_next_predict(i) > q_max(i) || q_next_predict(i) < q_min(i)) {
            is_saturated = true;
            break; 
        }
    }

    if (is_saturated) {
        // [策略2]: 饱和触发 -> 基座锁死 -> 机械臂重算
        vel_base_opt.setZero(); // 速度指令置0
        
        // 误差向量不再包含 v_base_opt
        error_vec_total.head<3>() = k_q * Delta_pE; 
        
        // 再次预测
        q_next_predict = compute_arm_prediction(error_vec_total, J_world, dt);
    } else {
        // [策略1]: 未饱和 -> 基座运动生效
        vel_base_opt = v_base_opt;

    }

    // 2. 更新机械臂指令角度 (带硬限位截断)
    for (int i = 0; i < 6; ++i) {
        q_next(i) = std::max(q_min(i), std::min(q_max(i), q_next_predict(i)));
    }
}



// 修改函数签名，增加两个引用参数用于输出
void clikRos::fcn_indirect_force_control(const Eigen::Vector3d& v_error, 
                               const Eigen::Vector3d& v_error_int, 
                               double des_f_E,     
                               double dt,
                               double& out_adm_pos, // [输出] 导纳位置修正量
                               double& out_adm_vel) // [输出] 导纳速度修正量
{
    // 1. 初始化
    double m_B = 5.4;
    double m_R = 2.6;
    double m_total = m_B + m_R;

    // =====================================================================
    // 2. 复合力控制器 (Feedforward + Feedback)
    // =====================================================================
    double Kp_force = 2.0; 
    double Ki_force = 0.5; 
    double int_limit = 20.0;

    // Y轴误差 (Eigen索引 1 为 Y轴)
    double v_err_y = v_error(1); 
    double v_err_int_y = std::max(-int_limit, std::min(int_limit, v_error_int(1)));

    // 反馈项 (PI)
    double u_feedback = Kp_force * v_err_y + Ki_force * v_err_int_y;
    
    // 前馈项 (Feedforward)
    double u_feedforward = des_f_E;

    // 总虚拟力
    double u_f = u_feedforward + u_feedback;

    // =====================================================================
    // 3. 导纳动力学解算
    // =====================================================================
    double Lambda_p_val = 4.0;
    double Kp_pos_val   = 4.0;
    double D_eq = m_total * (Kp_pos_val + Lambda_p_val); 
    double K_eq = m_total * (Kp_pos_val * Lambda_p_val);

    // 阻尼力与弹簧力 (使用类的成员变量)
    double F_damping = D_eq * adm_vel_err_; 
    double F_spring  = K_eq * adm_pos_err_;

    // 计算导纳加速度
    double acc_net = u_f - F_damping - F_spring;
    double dde_p = acc_net / m_total;

    // =====================================================================
    // 4. 状态积分 (内部更新)
    // =====================================================================
    adm_pos_err_ += adm_vel_err_ * dt;
    adm_vel_err_ += dde_p * dt;

    // =====================================================================
    // 5. 输出当前状态
    // =====================================================================
    out_adm_pos = adm_pos_err_;
    out_adm_vel = adm_vel_err_;
}

double clikRos::calculate_desired_force(const Eigen::Vector3d& vel_cmd_EE, 
                                                     const Eigen::Vector3d& acc_cmd_EE) 
{
    // 模型参数
    double m_C = 14.60;   // 小车质量
    double b_v = 1.16;     // 粘性摩擦系数
    double c_v = 1.58;     // 库伦摩擦幅值
    double gamma = 50.0;  // 双曲正切平滑因子
    
    // 提取 Y 轴分量 (Eigen 中 Y轴索引为 1)
    double v_cart_y = vel_cmd_EE(1);
    double acc_cmd_y = acc_cmd_EE(1);
    
    // 计算前馈力: F = ma + bv + c*tanh(gamma*v)
    double des_f_E = m_C * acc_cmd_y 
                   + b_v * v_cart_y 
                   + c_v * std::tanh(gamma * v_cart_y);
                   
    return des_f_E;
}


void clikRos::GetSmoothProfileLocal(double t_curr, double total_dist, double v_max, double acc,
                           double& pos, double& vel, double& a) {
    // 1. 预计算理论上的加速时间与距离 
    // t = v / a，最大速度为0.25m/s时候，所用时长为15.22
    double t_acc = v_max / acc; 
    // d = 0.5 * v * t
    double d_acc = 0.5 * v_max * t_acc; 
    
    double t_cruise = 0.0;

    // 2. 判断是否需要降级为三角形规划 (距离不够加速到最大速度)
    if (total_dist < 2.0 * d_acc) {
        // [三角形规划逻辑]
        d_acc = total_dist / 2.0;
        
        // 重新计算加速时间 (根据距离和原最大速度比例缩放)
        t_acc = 2.0 * d_acc / v_max; 
        
        // 重新计算实际能达到的最大速度
        v_max = 2.0 * d_acc / t_acc; 
        
        t_cruise = 0.0;
    } else {
        // [梯形规划逻辑]
        double d_cruise = total_dist - 2.0 * d_acc;
        t_cruise = d_cruise / v_max;
    }

    // 3. 定义时间节点
    double t1 = t_acc;
    double t2 = t_acc + t_cruise;
    double t3 = t2 + t_acc;

    // 4. S形曲线多项式计算
    if (t_curr < 0) {
        pos = 0; vel = 0; a = 0;
    } 
    else if (t_curr < t1) {
        // 加速段
        double tau = t_curr / t1;
        double tau2 = tau * tau;
        double tau3 = tau2 * tau;
        double tau4 = tau3 * tau;

        vel = v_max * (3*tau2 - 2*tau3);
        a   = (v_max / t1) * (6*tau - 6*tau2);
        pos = v_max * t1 * (tau3 - 0.5*tau4);
    } 
    else if (t_curr < t2) {
        // 匀速段
        double dt_cruise = t_curr - t1;
        vel = v_max;
        a   = 0;
        pos = d_acc + v_max * dt_cruise;
    } 
    else if (t_curr < t3) {
        // 减速段
        double tau = (t_curr - t2) / t1; // 注意这里分母是用加速时间 t1
        double tau2 = tau * tau;
        double tau3 = tau2 * tau;
        double tau4 = tau3 * tau;

        vel = v_max * (1.0 - (3*tau2 - 2*tau3));
        a   = -(v_max / t1) * (6*tau - 6*tau2);
        
        double dist_decel = v_max * t1 * (tau - tau3 + 0.5*tau4);
        pos = d_acc + (v_max * t_cruise) + dist_decel;
    } 
    else {
        // 结束
        pos = total_dist; vel = 0; a = 0;
    }
}


void clikRos::mainLoop()
{
    ensureAttitudeTargetStream();

    /*--------- 初始检查--------- */
    // If first time arm, reset whole mission
    checkArmingState();
    // Set relative origin, taking local position drift into account
    if(!ground_origin_position_initialized_flag_ )
    {
        setOnGroundOrigin();
        return;
    } 
    coordinate_flag_ = isCoordinate(m_rcin_);
    on_off_manipulator_flag_ = isManupulator(m_rcin_);

    // 新写法：无论是否进入 coordinate，都持续计算并发送系统质心到 PX4，
    // 保证补偿量与 PX4 侧 ESO / 控制器同步更新。
    updateSystemComState();
    publishSystemComToPx4();

    // if (current_state.connected && !coordinate_flag_) {
    //     LOGFMTD("Pb_x %f", flightStateData.x);
    //     LOGFMTD("Pb_y %f", flightStateData.y);
    //     LOGFMTD("Pb_z %f", flightStateData.z);
    //     LOGFMTD("Pe_x %f", position_EE(0));
    //     LOGFMTD("Pe_y %f", position_EE(1));
    //     LOGFMTD("Pe_z %f", position_EE(2));
    // }

    if (!coordinate_flag_ && last_coordinate_flag_)
    {
        coordinate_off_flag_ = true;
    }
    last_coordinate_flag_ = coordinate_flag_;

    if ( !(cur_action.behavior == WAYPOINT_FLIGHT))
    {
        coordinate_off_flag_ = false;
    }

    // 检查是否协调控制
    checkCoordinateState();

    /*--------- 机械臂操作--------- */
    // 机械臂的收放
    // 这里有三种情况，不协调（放）、不协调（收）、协调

    // 放下机械臂 根据指令来执行
    putDowndMnipulator();
    // 收机械臂
    putUpMnipulator();
  
    /*--------- 协调控制--------- */
    // 协调控制开始
    if (coordinate_flag_  && (this->now() - last_coordinate > rclcpp::Duration::from_seconds(1.0)))// 地面调试
    handleCoordinate();

}                                                                                                         

//修改：完成初始构型后的末端位置
Eigen::Vector3d initial_pos_EE = Eigen::Vector3d::Zero();
//修改：阶段号：
int current_phase_ = -1;
//修改：偏航角设置
double yaw_cmd = 0;

//修改：增加日志记录开关，默认开启
bool log_file_write = true;


// 机械臂与无人机协同运动
void clikRos::handleCoordinate()
{
    if (manipulator_mode != mod_control) {
        manipulator_mode = mod_control;
        Delta_mode_request.mode = 2;

        if (callManipulatorMode(Delta_mode_request.mode)) {
            ROS_INFO("CLIK: Manipulator is coordinated;");
        }

        coordinate_running_flag_ = true;
        ROS_INFO("Offboard enabled");

        last_time = this->now();
        now_time = this->now();
        test_begin = this->now();

        resetCoordinateIni();
        printf("come into init loop");
    }

    if (coordinate_running_flag_) {
        now_time = this->now();
        time_from_begin = now_time.seconds() - test_begin.seconds();
        double dt = now_time.seconds() - last_time.seconds();
        last_time = now_time;

        time_from_begin = (time_from_begin < 0.0) ? 0.0 : time_from_begin;

        Eigen::VectorXd current_desired_theta(6);
        Eigen::VectorXd initial_configuration_(6);
        Eigen::VectorXd pull_configuration_(6);
        Eigen::VectorXd quad_clik_nominal(6);
        quad_clik_nominal.setZero();

        Eigen::Vector3d euler_temp(flightStateData.phi, flightStateData.theta, flightStateData.psi);
        Eigen::Matrix3d rotation_body = euler_to_rotation(euler_temp);
        pos_body << flightStateData.x, flightStateData.y, flightStateData.z;

        Eigen::VectorXd current_theta(6);
        for (int i = 0; i < 6; ++i) {
            current_theta(i) = manipulatorData.clik_joint_pos[i];
        }

        const double t1 = 6.0;
        const double t2 = t1 + 1.0;
        const double t3 = t2 + 6.5;
        const double t4 = t3 + 10.0;
        const double t5 = t4 + 6.5;
        const double t6 = t5 + 10.0;
        const double t7 = t6 + 6.5;
        const double t8 = t7 + 10.0;
        const double t9 = t8 + 3.0;

        initial_configuration_ << 0, -M_PI/2, M_PI/2, 0, 0, 0; //初始构型
        pull_configuration_ << 0.0042, 0.2533, 0.6561, 1.6414, 0.3449, 0.0005;
        vehicle_position_cmd << m_coordinate_contr_ini.x, m_coordinate_contr_ini.y, m_coordinate_contr_ini.z;
        vehicle_velocity_cmd.setZero();

        const Eigen::Vector3d takeoff_point(m_coordinate_contr_ini.x,
                                            m_coordinate_contr_ini.y,
                                            m_coordinate_contr_ini.z);
        const Eigen::Vector3d waypoint_1 = takeoff_point + Eigen::Vector3d(-1.2, -1.2, 0.00);
        const Eigen::Vector3d waypoint_2 = waypoint_1 + Eigen::Vector3d(0.00, 2.4, 0.00);
        const Eigen::Vector3d waypoint_home = takeoff_point;

        auto commandVehicleSegment = [&](double segment_start,
                                         double segment_end,
                                         const Eigen::Vector3d& start_point,
                                         const Eigen::Vector3d& end_point) {
            const double segment_duration = std::max(segment_end - segment_start, 1e-3);
            const double tau_raw = (time_from_begin - segment_start) / segment_duration;
            const double tau = std::max(0.0, std::min(tau_raw, 1.0));
            const double s = tau * tau * (3.0 - 2.0 * tau);
            const double ds = (6.0 * tau * (1.0 - tau)) / segment_duration;
            const Eigen::Vector3d delta = end_point - start_point;

            vehicle_position_cmd = start_point + s * delta;
            vehicle_velocity_cmd = ds * delta;
        };

        if (time_from_begin < t1) {

            int new_phase = 0;

            double time_index = time_from_begin - 0.0;
            joint_cmd.get_cmd_from_linear(time_index, current_desired_theta, current_theta, pull_configuration_);
            desired_theta = current_desired_theta;

            last_vehicle_position_cmd = vehicle_position_cmd;
            

        } else if (time_from_begin < t2) {

            int new_phase = 1;

            last_vehicle_position_cmd = vehicle_position_cmd;

            vehicle_velocity_cmd.setZero();

        } else if (time_from_begin < t3) {

            int new_phase = 2;
            if (new_phase != current_phase_) {
                current_phase_ = new_phase;

                initial_pos_EE = position_EE;
            }

            vehicle_position_cmd = takeoff_point;
            vehicle_velocity_cmd.setZero();

            desired_pE << initial_pos_EE(0) + 0.200,  initial_pos_EE(1) + 0.00, initial_pos_EE(2) - 0.10;
            // desired_pE << pos_body(0) + 1.700, pos_body(1) + 0.00, pos_body(2) + 0.55;
            Eigen::Isometry3d Pose = Forward_Kinematic(pull_configuration_);
            target_Re = Pose.rotation();
            desired_theta = compute_desired_theta(rotation_body, current_theta, desired_pE, target_Re, dt);


        } else if (time_from_begin < t4) {

            int new_phase = 3;

            // desired_theta = current_theta;
            commandVehicleSegment(t3, t4, takeoff_point, waypoint_1);

            last_vehicle_position_cmd = vehicle_position_cmd;

        } else if (time_from_begin < t5) {

            int new_phase = 4;
            if (new_phase != current_phase_) {
                current_phase_ = new_phase;

                initial_pos_EE = position_EE;
            }

            vehicle_position_cmd = waypoint_1;
            vehicle_velocity_cmd.setZero();

            desired_pE << initial_pos_EE(0),  initial_pos_EE(1) + 0.150, initial_pos_EE(2) - 0.10;
            // desired_pE << pos_body(0) + 1.70, pos_body(1) + 0.250, pos_body(2) + 0.45;
            Eigen::Isometry3d Pose = Forward_Kinematic(pull_configuration_);
            target_Re = Pose.rotation();
            desired_theta = compute_desired_theta(rotation_body, current_theta, desired_pE, target_Re, dt);


        } else if (time_from_begin < t6) {

            int new_phase = 5;

            // desired_theta = current_theta;
            commandVehicleSegment(t5, t6, waypoint_1, waypoint_2);
            
            last_vehicle_position_cmd = vehicle_position_cmd;

        } else if (time_from_begin < t7) {

            int new_phase = 6;
            if (new_phase != current_phase_) {
                current_phase_ = new_phase;

                initial_pos_EE = position_EE;
            }

            vehicle_position_cmd = waypoint_2;
            vehicle_velocity_cmd.setZero();

            desired_pE << initial_pos_EE(0) - 0.200,  initial_pos_EE(1) - 0.10, initial_pos_EE(2) - 0.10;
            // desired_pE << pos_body(0) + 1.50, pos_body(1) + 0.150, pos_body(2) + 0.35;
            Eigen::Isometry3d Pose = Forward_Kinematic(pull_configuration_);
            target_Re = Pose.rotation();
            desired_theta = compute_desired_theta(rotation_body, current_theta, desired_pE, target_Re, dt);


        } else if (time_from_begin < t8) {

            int new_phase = 7;

            // desired_theta = current_theta;
            commandVehicleSegment(t7, t8, waypoint_2, waypoint_home);

            last_vehicle_position_cmd = vehicle_position_cmd;

        } else if (time_from_begin < t9) {

            int new_phase = 8;

            vehicle_position_cmd = waypoint_home;
            vehicle_velocity_cmd.setZero();

            joint_cmd.time_all = t9 - t8;
            double time_index = time_from_begin - t8;
            joint_cmd.get_cmd_from_linear(time_index, current_desired_theta, current_theta, land_configration_);
            desired_theta = current_desired_theta;
        } else {
            vehicle_position_cmd = waypoint_home;
            vehicle_velocity_cmd.setZero();
            desired_theta = land_configration_;
        }

        traj_cmd.type_mask = 2048;
        traj_cmd.coordinate_frame = 1;
        traj_cmd.header.stamp = this->now();

        traj_cmd.position.x = vehicle_position_cmd[0];
        traj_cmd.position.y = -vehicle_position_cmd[1];
        traj_cmd.position.z = -vehicle_position_cmd[2];

        traj_cmd.velocity.x = vehicle_velocity_cmd[0];
        traj_cmd.velocity.y = -vehicle_velocity_cmd[1];
        traj_cmd.velocity.z = -vehicle_velocity_cmd[2];

        traj_cmd.yaw = 0.0f;
        traj_cmd.acceleration_or_force.x = 0.0f;
        traj_cmd.acceleration_or_force.y = -0.0f;
        traj_cmd.acceleration_or_force.z = -0.0f;

        local_pos_pub->publish(traj_cmd);

        LOGFMTD("Pb_cmd_x %f", vehicle_position_cmd[0]);
        LOGFMTD("Pb_cmd_y %f", vehicle_position_cmd[1]);
        LOGFMTD("Pb_cmd_z %f", vehicle_position_cmd[2]);
        LOGFMTD("Pb_x %f", flightStateData.x);
        LOGFMTD("Pb_y %f", flightStateData.y);
        LOGFMTD("Pb_z %f", flightStateData.z);
        LOGFMTD("Pb_clik_x %f", quad_clik_nominal[0]);
        LOGFMTD("Pb_clik_y %f", quad_clik_nominal[1]);
        LOGFMTD("Pb_clik_z %f", quad_clik_nominal[2]);

        LOGFMTD("Pb_cmd_vx %f", vehicle_velocity_cmd[0]);
        LOGFMTD("Pb_cmd_vy %f", vehicle_velocity_cmd[1]);
        LOGFMTD("Pb_cmd_vz %f", vehicle_velocity_cmd[2]);
        LOGFMTD("Pb_vx %f", flightStateData.vx);
        LOGFMTD("Pb_vy %f", flightStateData.vy);
        LOGFMTD("Pb_vz %f", flightStateData.vz);
        LOGFMTD("Pb_clik_vx %f", quad_clik_nominal[3]);
        LOGFMTD("Pb_clik_vy %f", quad_clik_nominal[4]);
        LOGFMTD("Pb_clik_vz %f", quad_clik_nominal[5]);
        LOGFMTD("phi %f", flightStateData.phi);
        LOGFMTD("theta %f", flightStateData.theta);
        LOGFMTD("psi %f", flightStateData.psi);
        LOGFMTD("des_phi %f", attitude_sp(0));
        LOGFMTD("des_theta %f", attitude_sp(1));
        LOGFMTD("des_psi %f", attitude_sp(2));

        publishManipulatorJoints(desired_theta);

        for (int i = 0; i < 6; ++i) {
            LOGFMTD("q%d_cmd %f", i + 1, desired_theta(i));
            LOGFMTD("q%d %f", i + 1, manipulatorData.clik_joint_pos[i]);
        }

        LOGFMTD("Pe_cmd_x %f", desired_pE(0));
        LOGFMTD("Pe_cmd_y %f", desired_pE(1));
        LOGFMTD("Pe_cmd_z %f", desired_pE(2));
        LOGFMTD("Pe_x %f", position_EE(0));
        LOGFMTD("Pe_y %f", position_EE(1));
        LOGFMTD("Pe_z %f", position_EE(2));

        LOGFMTD("Pe_cmd_vx %f", desired_vE(0));
        LOGFMTD("Pe_cmd_vy %f", desired_vE(1));
        LOGFMTD("Pe_cmd_vz %f", desired_vE(2));
        LOGFMTD("Pe_vx %f", velocity_EE(0));
        LOGFMTD("Pe_vy %f", velocity_EE(1));
        LOGFMTD("Pe_vz %f", velocity_EE(2));

        Eigen::Quaterniond desired_attitude_q(rotation_body * Assemble_rotation * target_Re);
        Eigen::Quaterniond attitude_q(Re);

        LOGFMTD("desired_attitude_q_w %f", desired_attitude_q.w());
        LOGFMTD("desired_attitude_q_x %f", desired_attitude_q.x());
        LOGFMTD("desired_attitude_q_y %f", desired_attitude_q.y());
        LOGFMTD("desired_attitude_q_z %f", desired_attitude_q.z());

        LOGFMTD("attitude_q_w %f", attitude_q.w());
        LOGFMTD("attitude_q_x %f", attitude_q.x());
        LOGFMTD("attitude_q_y %f", attitude_q.y());
        LOGFMTD("attitude_q_z %f", attitude_q.z());

        if (px4_coordinate_debug_valid_) {
            LOGFMTD("px4_pos_comp_x %f", px4_pos_comp_(0));
            LOGFMTD("px4_pos_comp_y %f", px4_pos_comp_(1));
            LOGFMTD("px4_pos_comp_z %f", px4_pos_comp_(2));
            LOGFMTD("px4_pos_eso_x %f", px4_pos_eso_(0));
            LOGFMTD("px4_pos_eso_y %f", px4_pos_eso_(1));
            LOGFMTD("px4_pos_eso_z %f", px4_pos_eso_(2));
            LOGFMTD("px4_u_v_nom_x %f", px4_pos_u_nominal_(0));
            LOGFMTD("px4_u_v_nom_y %f", px4_pos_u_nominal_(1));
            LOGFMTD("px4_u_v_nom_z %f", px4_pos_u_nominal_(2));
            LOGFMTD("px4_att_comp_x %f", px4_att_comp_(0));
            LOGFMTD("px4_att_comp_y %f", px4_att_comp_(1));
            LOGFMTD("px4_att_comp_z %f", px4_att_comp_(2));
            LOGFMTD("px4_att_eso_x %f", px4_att_eso_(0));
            LOGFMTD("px4_att_eso_y %f", px4_att_eso_(1));
            LOGFMTD("px4_att_eso_z %f", px4_att_eso_(2));
            LOGFMTD("px4_u_w_nom_x %f", px4_att_u_nominal_(0));
            LOGFMTD("px4_u_w_nom_y %f", px4_att_u_nominal_(1));
            LOGFMTD("px4_u_w_nom_z %f", px4_att_u_nominal_(2));
        }

        if (log_file_write)    {

            // --- 写入自定义 txt 文件 ---
            std::ofstream log_file(LOG_FILE_PATH, std::ios::app);
            if (log_file.is_open()) {
                log_file << "TIME " << time_from_begin << "\n";

                // log states
                log_file << "Pb_cmd_x " << vehicle_position_cmd[0] << "\n";
                log_file << "Pb_cmd_y " << vehicle_position_cmd[1] << "\n";
                log_file << "Pb_cmd_z " << vehicle_position_cmd[2] << "\n";
                log_file << "Pb_x " << flightStateData.x << "\n";
                log_file << "Pb_y " << flightStateData.y << "\n";
                log_file << "Pb_z " << flightStateData.z << "\n";

                log_file << "Pb_cmd_vx " << vehicle_velocity_cmd[0] << "\n";
                log_file << "Pb_cmd_vy " << vehicle_velocity_cmd[1] << "\n";
                log_file << "Pb_cmd_vz " << vehicle_velocity_cmd[2] << "\n";
                log_file << "Pb_vx " << flightStateData.vx << "\n";
                log_file << "Pb_vy " << flightStateData.vy << "\n";
                log_file << "Pb_vz " << flightStateData.vz << "\n";

                log_file << "phi " << flightStateData.phi << "\n";
                log_file << "theta " << flightStateData.theta << "\n";
                log_file << "psi " << flightStateData.psi << "\n";
                log_file << "des_phi " << attitude_sp(0) << "\n";
                log_file << "des_theta " << attitude_sp(1) << "\n";
                log_file << "des_psi " << attitude_sp(2) << "\n";

                // log joint states
                for (int i = 0; i < 6; ++i) {
                    log_file << "q" << i + 1 << "_cmd " << desired_theta(i) << "\n";
                    log_file << "q" << i + 1 << " " << manipulatorData.clik_joint_pos[i] << "\n";
                }

                // log Effector Position & Velocity
                log_file << "Pe_cmd_x " << desired_pE(0) << "\n";
                log_file << "Pe_cmd_y " << desired_pE(1) << "\n";
                log_file << "Pe_cmd_z " << desired_pE(2) << "\n";
                log_file << "Pe_x " << position_EE(0) << "\n";
                log_file << "Pe_y " << position_EE(1) << "\n";
                log_file << "Pe_z " << position_EE(2) << "\n";

                log_file << "Pe_cmd_vx " << desired_vE(0) << "\n";
                log_file << "Pe_cmd_vy " << desired_vE(1) << "\n";
                log_file << "Pe_cmd_vz " << desired_vE(2) << "\n";
                log_file << "Pe_vx " << velocity_EE(0) << "\n";
                log_file << "Pe_vy " << velocity_EE(1) << "\n";
                log_file << "Pe_vz " << velocity_EE(2) << "\n";

                // log Effector Quaternions
                log_file << "desired_attitude_q_w " << desired_attitude_q.w() << "\n";
                log_file << "desired_attitude_q_x " << desired_attitude_q.x() << "\n";
                log_file << "desired_attitude_q_y " << desired_attitude_q.y() << "\n";
                log_file << "desired_attitude_q_z " << desired_attitude_q.z() << "\n";
                log_file << "attitude_q_w " << attitude_q.w() << "\n";
                log_file << "attitude_q_x " << attitude_q.x() << "\n";
                log_file << "attitude_q_y " << attitude_q.y() << "\n";
                log_file << "attitude_q_z " << attitude_q.z() << "\n";

                log_file << "--------------------\n"; // 分隔符
                log_file.close();
            }
        }
    }
}
