#include "clik_main.h"
#include "forward_kinematic.h"
#include <unistd.h>

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

clikRos::clikRos() : Node("clik"), 
                     last_off_manipulator(0, 0, this->get_clock()->get_clock_type()),
                     last_on_manipulator(0, 0, this->get_clock()->get_clock_type()),
                     last_coordinate(0, 0, this->get_clock()->get_clock_type()),
                     last_time(0, 0, this->get_clock()->get_clock_type()),
                     now_time(0, 0, this->get_clock()->get_clock_type()),
                     test_begin(0, 0, this->get_clock()->get_clock_type())
{
    state_sub = this->create_subscription<mavros_msgs::msg::State>("mavros/state", 10, std::bind(&clikRos::state_obtain, this, std::placeholders::_1));
    position_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", rclcpp::QoS(10).best_effort(), std::bind(&clikRos::pos_obtain, this, std::placeholders::_1));
    attitude_sub = this->create_subscription<sensor_msgs::msg::Imu>("mavros/imu/data", rclcpp::QoS(10).best_effort(), std::bind(&clikRos::att_obtain, this, std::placeholders::_1));
    rcin_sub = this->create_subscription<mavros_msgs::msg::RCIn>("mavros/rc/in", 10, std::bind(&clikRos::rcin_obtain, this, std::placeholders::_1)); 
    Ti5_arm_EE_sub = this->create_subscription<geometry_msgs::msg::TransformStamped>("vicon/Ti5_arm_EE/Ti5_arm_EE", 10, std::bind(&clikRos::Ti5_arm_EE_obtain, this, std::placeholders::_1));
    action_sub = this->create_subscription<calibration::msg::Action>("navigator/vehicle_action", 10, std::bind(&clikRos::vehicle_action_callback, this, std::placeholders::_1)); 
    
    // get_jnt_state_client = this->create_client<gazebo_msgs::srv::GetJointProperties>("/gazebo/get_joint_properties");

    Delta_pub = this->create_publisher<calibration::msg::PositionPub>("control_signal/pos_pub", 10);
    joint_ctrl_pub = this->create_publisher<sensor_msgs::msg::JointState>("arm/joint_control", 10);
    local_pos_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("online_target", 10); 
    write_pub = this->create_publisher<std_msgs::msg::String>("serial_write", 1000);

    manipulator_client = this->create_client<calibration::srv::ManipulatorMode>("control_signal/command_mode");
    set_mode_client = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

    manipulator_mode = mod_sleep;
    now_time = this->now();
    
    double pi=3.14159;
    q_serial_des.resize(group_n);
    q_serial_des.at(0)={0,pi/2,0,0,-pi/2,0};
    q_serial_des.at(1)={pi/6,pi/3,pi/6,-pi/3,pi/3,0};
    q_serial_des.at(2)={pi/4,0,pi/3,-pi/2,pi/3,0};
    q_serial_des.at(3)={2*pi/3,-pi/6,pi/2,pi/5,pi/3,0};
    q_serial_des.at(4)={0,0,pi/6,-pi/5,-pi/4,0};
    q_serial_des.at(5)={-pi/2,pi/2,0,pi/4,-pi/2,0};
    q_serial_des.at(6)={-pi/2,-pi/6,-pi/6,pi/2,0,0};
    q_serial_des.at(7)={-pi/3,-pi/2,pi/6,0,pi/2,pi/3};
    q_serial_des.at(8)={-pi/3,-pi/3,pi/6,0,pi/4,pi/6};
    q_serial_des.at(9)={-pi/6,0,pi/6,0,pi/2,pi/3};
    q_serial_des.at(10)={-pi/6,0,0,pi/4,pi/2,pi/3};
    q_serial_des.at(11)={0,0,pi/6,0,pi/2,pi/3};

    pos_body.resize(group_n);
    pos_EE.resize(group_n);
    rotation_b2i.resize(group_n);
    
    Delta_mode_req = std::make_shared<calibration::srv::ManipulatorMode::Request>();
}

void clikRos::euler_to_rotation(const Eigen::Vector3d& euler, Eigen::Matrix3d& rotation) {
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
    rotation=RZ*RY*RX;
}

void clikRos::state_obtain(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state = *msg;
}

void clikRos::pos_obtain(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_local_pos = *msg;
    flightStateData.x = msg->pose.position.x;
    flightStateData.y = - msg->pose.position.y;
    flightStateData.z = - msg->pose.position.z;
}

void clikRos::rcin_obtain(const mavros_msgs::msg::RCIn::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(rc_mutex_);
    m_rcin_ = *msg;
}

void clikRos::att_obtain(const sensor_msgs::msg::Imu::SharedPtr msg) {
    Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);
    flightStateData.phi    = euler_fcu[0];
    flightStateData.theta  =  - euler_fcu[1];
    flightStateData.psi    =  - euler_fcu[2] ;

    flightStateData.wx = msg->angular_velocity.x;
    flightStateData.wy = - msg->angular_velocity.y;
    flightStateData.wz = - msg->angular_velocity.z;
}

void clikRos::Ti5_arm_EE_obtain(const geometry_msgs::msg::TransformStamped::SharedPtr message_holder) {
   pe[0] = message_holder->transform.translation.x;
   pe[1] = - message_holder->transform.translation.y;
   pe[2] = - message_holder->transform.translation.z;
}

void clikRos::vehicle_action_callback(const calibration::msg::Action::SharedPtr msg) {
    cur_action = *msg;
}

bool clikRos::isManupulator(const mavros_msgs::msg::RCIn& rcin) {
    return ((rcin.channels.size()>=7) && (rcin.channels.at(9)>1500));
}

bool clikRos::isCoordinate(const mavros_msgs::msg::RCIn& rcin) {
    return (rcin.channels.size()>=7 && rcin.channels.at(8)>1500);
}

void clikRos::setOnGroundOrigin() {
     m_ref_origin_.pose.position.x = current_local_pos.pose.position.x;
     m_ref_origin_.pose.position.y = current_local_pos.pose.position.y;
     m_ref_origin_.pose.position.z = current_local_pos.pose.position.z;
     ground_origin_position_initialized_flag_ = true;
     RCLCPP_INFO(this->get_logger(), "refernce origin set at position x  %f y  %f z %f ", m_ref_origin_.pose.position.x , m_ref_origin_.pose.position.y , m_ref_origin_.pose.position.z);
}

void clikRos::resetCLIK() {
    on_off_manipulator_flag_ = false;
    resetOnGroundOrigin();
}

void clikRos::resetOnGroundOrigin() {
    ground_origin_position_initialized_flag_ = false;
    origin_counter  = 0;
    m_ref_origin_.pose.position.x = 0;
    m_ref_origin_.pose.position.y = 0;
    m_ref_origin_.pose.position.z = 0;
}

void clikRos::resetCoordinateIni() {
    m_coordinate_contr_ini.x = flightStateData.x;
    m_coordinate_contr_ini.y = flightStateData.y;
    m_coordinate_contr_ini.z = flightStateData.z;
    m_coordinate_contr_ini.psi = flightStateData.psi;
    double dtemp[3];
    dtemp[0] = pe[0]- flightStateData.x;
    dtemp[1] = pe[1]- flightStateData.y;
    dtemp[2] = pe[2]- flightStateData.z;
    Eigen::Vector3d euler;
    euler(0) = flightStateData.phi;
    euler(1) = flightStateData.theta;
    euler(2) = flightStateData.psi;

    Eigen::Matrix3d rotation  = Eigen::Matrix3d::Identity();
    euler_to_rotation(euler,rotation);
    
    m_coordinate_contr_ini.xe = rotation(0,0)*dtemp[0]+rotation(1,0)*dtemp[1]+rotation(2,0)*dtemp[2];
    m_coordinate_contr_ini.ye = rotation(0,1)*dtemp[0]+rotation(1,1)*dtemp[1]+rotation(2,1)*dtemp[2];
    m_coordinate_contr_ini.ze = rotation(0,2)*dtemp[0]+rotation(1,2)*dtemp[1]+rotation(2,2)*dtemp[2];
    RCLCPP_INFO(this->get_logger(), "CLIK:INI_X=%f\tINI_Y=%f\tINI_Z=%f\t\n",m_coordinate_contr_ini.xe,m_coordinate_contr_ini.ye,m_coordinate_contr_ini.ze);
}

void clikRos::checkArmingState() {
    if(!current_state.armed && !reset_CLIK_flag_) {
        reset_CLIK_flag_ = true;
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Disarmed! Will restart the mission next time armed!\n");
    }
    if(current_state.armed && reset_CLIK_flag_) {
        reset_CLIK_flag_ = false;
        RCLCPP_INFO(this->get_logger(), "First time to Arm");
        resetCLIK();
    }
}

void clikRos::checkCoordinateState() {
    if(!coordinate_flag_ && !reset_coordinate_flag_) {
        reset_coordinate_flag_ = true;
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Discoordinated! Will restart the mission next time coordinated!\n");
    }
    if( coordinate_flag_ && reset_coordinate_flag_) {
        reset_coordinate_flag_ = false;
        RCLCPP_INFO(this->get_logger(), "First time to Coordinate");
        resetCoordinateIni();
        last_coordinate = this->now();
    }
}

void clikRos::publishCommands(int i) {        
    joint_ctrl_msg.position.clear();
    joint_ctrl_msg.position.push_back(q_serial_des.at(i)[0]);
    joint_ctrl_msg.position.push_back(q_serial_des.at(i)[1]);
    joint_ctrl_msg.position.push_back(q_serial_des.at(i)[2]);
    joint_ctrl_msg.position.push_back(q_serial_des.at(i)[3]);
    joint_ctrl_msg.position.push_back(q_serial_des.at(i)[4]);
    joint_ctrl_msg.position.push_back(q_serial_des.at(i)[5]);
    joint_ctrl_msg.header.frame_id = "joint_ctrl_frame";
    joint_ctrl_msg.header.stamp = this->now();
    joint_ctrl_pub->publish(joint_ctrl_msg);
}

void clikRos::recordData(int i) {
    std::ofstream data_file("/home/iusl/preset_traj_control/data_record.txt", std::ios::app);
    if (!data_file.is_open()) {
        std::cerr << "无法打开文件进行数据记录!" << std::endl;
        return;
    }

    pos_body.at(i) << flightStateData.x, flightStateData.y, flightStateData.z;
    pos_EE.at(i) << pe[0], pe[1], pe[2];
    Eigen::Vector3d euler_temp(flightStateData.phi, flightStateData.theta, flightStateData.psi);
    Eigen::Matrix3d rotation;
    euler_to_rotation(euler_temp, rotation);
    rotation_b2i.at(i) = rotation;

    std::vector<double> q;
    Eigen::Matrix4d end_effector_T;
    Eigen::Vector3d end_effector_pos_in_base;

    q = q_serial_des.at(i);
    end_effector_T = Forward_Kinematic(q);
    end_effector_pos_in_base = end_effector_T.block<3,1>(0,3);

    data_file << "Recording Data " << i + 1 << std::endl;
    data_file << "pb: " << pos_body.at(i).transpose() << std::endl;
    data_file << "euler_temp: " << euler_temp.transpose() << std::endl;
    data_file << "rotation_b2i: \n" << rotation << std::endl;
    data_file << "pos_EE: " << pos_EE.at(i).transpose() << std::endl;
    data_file << "end_effector_pos_in_base: " << end_effector_pos_in_base.transpose() << std::endl;
    data_file << "---------------------------" << std::endl;
    data_file.close();
}

void clikRos::handleCoordinate() {
    if (!(manipulator_mode==mod_control)) {
        manipulator_mode = mod_control;
        Delta_mode_req->mode = 2;
        
        // 异步发送服务请求（完美兼容原 ROS 1 的 spinOnce 架构）
        if (manipulator_client->service_is_ready()) {
            manipulator_client->async_send_request(Delta_mode_req, 
                [this](rclcpp::Client<calibration::srv::ManipulatorMode>::SharedFuture future) {
                    if (future.get()->result) {
                        RCLCPP_INFO(this->get_logger(), "CLIK: Manipulator is coordinated;");
                    }
                });
        }
        
        coordinate_running_flag_ = true;
        memset(Yout,0,sizeof(Yout));

        last_time = this->now() ; 
        now_time  = this->now() ; 

        test_begin = this->now() ;
        resetCoordinateIni();
    }

    if (coordinate_running_flag_) {
        now_time  = this->now() ;
        double time_from_begin =  (now_time - test_begin).seconds();

        if (time_from_begin> current_group * every_group_time_interval && time_from_begin < (current_group + 1) * every_group_time_interval && current_group< group_n) {
            std::cout<< "group"<< current_group +1 <<std::endl;

            if (!command_published) {
                publishCommands(current_group);
                command_published = true;  
                data_recorded = false;     
            }

            if (time_from_begin >= (current_group + 1) * every_group_time_interval - 1.0 && !data_recorded) {
                recordData(current_group);
                data_recorded = true;  
                current_group++;
                command_published = false;  
            }
        }

        if (time_from_begin >= group_n* every_group_time_interval && !flag_calibrate) {
            Eigen::Vector3d pe_b;
            Eigen::MatrixXd A(24,12);
            Eigen::Matrix3d E3 = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d A1;
            Eigen::Matrix3d A2;
            Eigen::Matrix3d A3;
            A.setZero();
            A1.setZero();
            A2.setZero();
            A3.setZero();

            Eigen::MatrixXd A_(3,12);
            Eigen::VectorXd end_effector_pos_(24);
            Eigen::VectorXd assemble_pos_attitude(12);

            std::vector<double> q;
            Eigen::Matrix4d end_effector_T;
            Eigen::Vector3d end_effector_pos_in_base;

            for (int i = 0; i < 8; i++) {
                pe_b= rotation_b2i.at(i).transpose() * (pos_EE.at(i) - pos_body.at(i));
                q=q_serial_des.at(i);
                end_effector_T= Forward_Kinematic(q);
                end_effector_pos_in_base=end_effector_T.block<3,1>(0,3);
                A1.row(0)=end_effector_pos_in_base;
                A2.row(1)=end_effector_pos_in_base;
                A3.row(2)=end_effector_pos_in_base;
                A_<< E3,A1,A2,A3;
                A.block<3,12>(3*i,0)=A_;
                end_effector_pos_.segment<3>(3*i)=pe_b;
            } 

            std::cout<<"det of A"<< (A.transpose()*A).determinant() <<std::endl;

            assemble_pos_attitude=(A.transpose()*A).inverse()*A.transpose()*end_effector_pos_;
            Eigen::Vector3d assemble_pos=assemble_pos_attitude.segment<3>(0);
            Eigen::VectorXd assemble_attitude;
            assemble_attitude=assemble_pos_attitude.segment<9>(3);
            Eigen :: Matrix3d Aasseble_rotation = Eigen :: Map <Eigen::Matrix3d>(assemble_attitude.data()).transpose();

            std::cout<< "caculated value"<<std::endl;
            std::cout<< assemble_pos<<std::endl;
            std::cout<< Aasseble_rotation<<std::endl;

           std::ofstream data_file("/home/iusl/preset_traj_control/data_record.txt", std::ios::app);
           if (!data_file.is_open()) {
             std::cerr << "无法打开文件进行数据记录!" << std::endl;
             return;
            }

              data_file << "assemble_pos: " << assemble_pos << std::endl;
              data_file << "Aasseble_rotation:\n " << Aasseble_rotation << std::endl;
              data_file << "---------------------------" << std::endl;
            data_file.close();
            
            flag_calibrate = true;

            if (flag_calibrate) {
                Eigen::Vector3d pe_in_body;
                Eigen::Vector3d caculated_pe;

                q=q_serial_des.at(8); 
                end_effector_T= Forward_Kinematic(q);
                pe_in_body=end_effector_T.block<3,1>(0,3);
                caculated_pe=pos_body.at(8)+rotation_b2i.at(8)*(assemble_pos+Aasseble_rotation*pe_in_body);
                std::cout<< "caculated value\n"<< caculated_pe <<std::endl;
                std::cout<< "real value\n"<< pos_EE.at(8) <<std::endl;
                
                q=q_serial_des.at(9); 
                end_effector_T= Forward_Kinematic(q);
                pe_in_body=end_effector_T.block<3,1>(0,3);
                caculated_pe=pos_body.at(9)+rotation_b2i.at(9)*(assemble_pos+Aasseble_rotation*pe_in_body);
                std::cout<< "caculated value\n"<< caculated_pe<<std::endl;
                std::cout<< "real value\n"<< pos_EE.at(9) <<std::endl;

                q=q_serial_des.at(10); 
                end_effector_T= Forward_Kinematic(q);
                pe_in_body=end_effector_T.block<3,1>(0,3);
                caculated_pe=pos_body.at(10)+rotation_b2i.at(10)*(assemble_pos+Aasseble_rotation*pe_in_body);
                std::cout<< "caculated value\n"<< caculated_pe<<std::endl;
                std::cout<< "real value\n"<< pos_EE.at(10) <<std::endl;

                q=q_serial_des.at(11); 
                end_effector_T= Forward_Kinematic(q);
                pe_in_body=end_effector_T.block<3,1>(0,3);
                caculated_pe=pos_body.at(11)+rotation_b2i.at(11)*(assemble_pos+Aasseble_rotation*pe_in_body);
                std::cout<< "caculated value\n"<< caculated_pe<<std::endl;
                std::cout<< "real value\n"<< pos_EE.at(11) <<std::endl;
           }
        }
    }
}
        
void clikRos::mainLoop() {
    checkArmingState();
    if(!ground_origin_position_initialized_flag_ ) {
        setOnGroundOrigin();
        return;
    } 
    coordinate_flag_ = isCoordinate(m_rcin_);

    on_off_manipulator_flag_ = isManupulator(m_rcin_);
    if (!coordinate_flag_ && last_coordinate_flag_) {
        coordinate_off_flag_ = true;
    }
    last_coordinate_flag_ = coordinate_flag_;

    if ( !(cur_action.behavior == WAYPOINT_FLIGHT)) {
        coordinate_off_flag_ = false;
    }
    
    checkCoordinateState();
    
    if (coordinate_flag_  && ((this->now() - last_coordinate).seconds() > 1.0)) {
        handleCoordinate();
    }
}