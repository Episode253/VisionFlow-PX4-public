#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <yolo_ros_msgs/msg/bounding_boxes.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <deque>
#include <cmath>
#include <algorithm>
#include <string>

// EMA 滤波器
class EmaFilter {
public:
    EmaFilter(double alpha = 0.4) : alpha_(alpha), initialized_(false), value_(0.0) {}

    double update(double new_val) {
        if (!initialized_) {
            value_ = new_val;
            initialized_ = true;
        } else {
            value_ = alpha_ * new_val + (1.0 - alpha_) * value_;
        }
        return value_;
    }

    void reset() { initialized_ = false; }

private:
    double alpha_;
    bool initialized_;
    double value_;
};

// PID 控制器
class PIDController {
public:
    PIDController(double kp, double kd, double out_min, double out_max)
        : kp_(kp), kd_(kd), min_(out_min), max_(out_max), prev_error_(0.0), prev_time_(-1.0) {}

    double compute(double error, double current_time) {
        if (prev_time_ < 0.0) {
            prev_time_ = current_time;
            prev_error_ = error;
            return 0.0;
        }

        double dt = current_time - prev_time_;
        if (dt <= 0.0001) return 0.0;

        double derivative = (error - prev_error_) / dt;
        double output = (kp_ * error) + (kd_ * derivative);

        output = std::max(std::min(output, max_), min_);

        prev_error_ = error;
        prev_time_ = current_time;
        return output;
    }

    void reset() {
        prev_time_ = -1.0;
        prev_error_ = 0.0;
    }

    void set_limits(double min_val, double max_val) {
        min_ = min_val;
        max_ = max_val;
    }

private:
    double kp_, kd_, min_, max_;
    double prev_error_, prev_time_;
};

class YoloOffboardTracker : public rclcpp::Node {
public:
    YoloOffboardTracker() : Node("yolo_human_tracking"),
                            filter_x_(0.12), filter_y_(0.15), filter_d_(0.18),
                            pid_x_(3.2, 0.0, -4.0, 4.0),
                            pid_z_(0.5, 0.0, -1.0, 1.0),
                            pid_yaw_(1.8, 0.0, -3.0, 3.0)
    {
        // QoS 配置
        rclcpp::QoS qos_profile(10);
        qos_profile.best_effort();
        qos_profile.keep_last(10);

        // 订阅与发布
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10, std::bind(&YoloOffboardTracker::state_cb, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/mavros/local_position/odom", qos_profile, std::bind(&YoloOffboardTracker::odom_cb, this, std::placeholders::_1));
        yolo_sub_ = this->create_subscription<yolo_ros_msgs::msg::BoundingBoxes>(
            "/yolo/BoundingBoxes", qos_profile, std::bind(&YoloOffboardTracker::yolo_cb, this, std::placeholders::_1));

        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);
        vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

        // 服务客户端
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        // 定时器
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&YoloOffboardTracker::control_loop, this));
        offboard_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&YoloOffboardTracker::offboard_arm_loop, this));

        last_ctrl_time_ = this->now().seconds();
        RCLCPP_INFO(this->get_logger(), "Yolo Human Tracking Node (C++) Initialized");
    }

    ~YoloOffboardTracker() {
        // 优雅退出：发送刹车指令
        auto stop_vel = geometry_msgs::msg::TwistStamped();
        stop_vel.header.stamp = this->now();
        stop_vel.header.frame_id = "base_link";
        stop_vel.twist.linear.x = 0.0;
        stop_vel.twist.linear.y = 0.0;
        stop_vel.twist.linear.z = 0.0;
        stop_vel.twist.angular.z = 0.0;
        vel_pub_->publish(stop_vel);
        RCLCPP_INFO(this->get_logger(), "Stop command sent.");
    }

private:
    double takeoff_height_ = 1.20;
    double pos_tolerance_ = 0.065;

    EmaFilter filter_x_, filter_y_, filter_d_;
    PIDController pid_x_, pid_z_, pid_yaw_;

    double accel_limit_x_ = 2.0;
    double accel_limit_z_ = 1.0;
    double accel_limit_yaw_ = 1.0;

    double max_vel_x_ = 2.0;
    double max_vel_z_ = 0.8;
    double max_vel_yaw_ = 0.5;

    double last_vel_x_ = 0.0;
    double last_vel_z_ = 0.0;
    double last_vel_yaw_ = 0.0;
    double last_ctrl_time_;

    double img_width_ = 1280.0;
    double img_height_ = 720.0;
    double center_x_ = img_width_ / 2.0;
    double center_y_ = img_height_ / 2.0;
    double target_height_ref_ = 360.0;

    mavros_msgs::msg::State current_state_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_ = nullptr;

    double hover_yaw_ = 0.0;
    std::vector<double> hover_pos_ = {0.0, 0.0, 0.0};
    std::string flight_phase_ = "CHECK_STATUS";

    bool target_captured_ = false;
    int lost_count_ = 0;

    struct TargetInfo { double x = 0; double y = 0; double d = 0; } target_info_;

    bool locked_ = false;
    int lock_frames_ = 0;
    int lock_confirm_frames_ = 40;
    std::deque<std::pair<double, double>> det_window_;

    bool waiting_for_mavros_ = false;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<yolo_ros_msgs::msg::BoundingBoxes>::SharedPtr yolo_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;

    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr offboard_timer_;

    void state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
        current_state_ = *msg;
    }

    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_ = msg;
        if (flight_phase_ == "CHECK_STATUS") {
            tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
            );
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            hover_yaw_ = yaw;
        }
    }

    void offboard_arm_loop() {
        if (flight_phase_ == "CHECK_STATUS" ||
            current_state_.mode == "AUTO.LAND" ||
            current_state_.mode == "AUTO.RTL" ||
            current_state_.mode == "AUTO.TAKEOFF") {
            return;
        }
        if (current_state_.connected) {
            if (current_state_.mode != "OFFBOARD") {
                auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                req->custom_mode = "OFFBOARD";
                set_mode_client_->async_send_request(req);
                RCLCPP_INFO_ONCE(this->get_logger(), "Requesting OFFBOARD mode...");
            }
            if (!current_state_.armed) {
                auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                req->value = true;
                arming_client_->async_send_request(req);
                RCLCPP_INFO_ONCE(this->get_logger(), "Requesting ARM...");
            }
        }
    }

    void yolo_cb(const yolo_ros_msgs::msg::BoundingBoxes::SharedPtr msg) {
        bool detected = false;

        for (const auto& box : msg->bounding_boxes) {
            if (box.class_id == "target") {
                target_captured_ = true;
                lost_count_ = 0;
                double raw_x = (box.xmin + box.xmax) / 2.0;
                double raw_y = (box.ymin + box.ymax) / 2.0;
                double raw_d = static_cast<double>(box.ymax - box.ymin);

                target_info_.x = filter_x_.update(raw_x);
                target_info_.y = filter_y_.update(raw_y);
                target_info_.d = filter_d_.update(raw_d);

                det_window_.push_back({target_info_.x, target_info_.y});
                if (det_window_.size() > 60) det_window_.pop_front();

                lock_frames_++;
                if (!locked_ && lock_frames_ >= lock_confirm_frames_) {
                    locked_ = true;
                }

                detected = true;
                break;
            }
        }

        if (!detected) {
            filter_x_.reset();
            filter_y_.reset();
            filter_d_.reset();
            lock_frames_ = 0;
        }
    }

    void control_loop() {
        if (!current_odom_ || !current_state_.connected) {
            if (!waiting_for_mavros_) {
                waiting_for_mavros_ = true;
                RCLCPP_INFO(this->get_logger(), "MAVROS not ready. Waiting for connection...");
            }
            return;
        } else {
            if (waiting_for_mavros_) {
                waiting_for_mavros_ = false;
                RCLCPP_INFO(this->get_logger(), "MAVROS connected, starting tracking state machine.");
            }
        }

        if (flight_phase_ == "CHECK_STATUS") {
            hover_pos_[0] = current_odom_->pose.pose.position.x;
            hover_pos_[1] = current_odom_->pose.pose.position.y;
            double current_z = current_odom_->pose.pose.position.z;

            if (current_state_.armed) {
                hover_pos_[2] = current_z;
                RCLCPP_INFO(this->get_logger(), "Airborne detected (z=%.2fm). Holding current altitude.", current_z);
            } else {
                hover_pos_[2] = takeoff_height_;
                RCLCPP_INFO(this->get_logger(), "On Ground. Taking off to default %.2fm.", takeoff_height_);
            }
            flight_phase_ = "TAKEOFF";
        }
        else if (flight_phase_ == "TAKEOFF") {
            perform_takeoff();
            double curr_z = current_odom_->pose.pose.position.z;
            if (std::abs(curr_z - hover_pos_[2]) < pos_tolerance_) {
                flight_phase_ = "HOVER";
                RCLCPP_INFO(this->get_logger(), "Phase: HOVER");
            }
        }
        else if (flight_phase_ == "HOVER") {
            perform_hover();
            if (target_captured_) {
                pid_x_.reset(); pid_z_.reset(); pid_yaw_.reset();
                filter_x_.reset(); filter_y_.reset(); filter_d_.reset();
                last_vel_x_ = 0.0; last_vel_z_ = 0.0; last_vel_yaw_ = 0.0;

                flight_phase_ = "TRACK";
                RCLCPP_INFO(this->get_logger(), "Phase: TRACK");
            }
        }
        else if (flight_phase_ == "TRACK") {
            perform_tracking();
            if (!target_captured_) {
                hover_pos_[0] = current_odom_->pose.pose.position.x;
                hover_pos_[1] = current_odom_->pose.pose.position.y;
                hover_pos_[2] = current_odom_->pose.pose.position.z;

                tf2::Quaternion q(
                    current_odom_->pose.pose.orientation.x,
                    current_odom_->pose.pose.orientation.y,
                    current_odom_->pose.pose.orientation.z,
                    current_odom_->pose.pose.orientation.w
                );
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                hover_yaw_ = yaw;

                flight_phase_ = "HOVER";
                RCLCPP_WARN(this->get_logger(), "Target Lost -> HOVER");
            }
        }
    }

    void publish_pose(double x, double y, double z, double yaw) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();

        local_pos_pub_->publish(pose);
    }

    void perform_takeoff() { publish_pose(hover_pos_[0], hover_pos_[1], hover_pos_[2], hover_yaw_); }
    void perform_hover() { publish_pose(hover_pos_[0], hover_pos_[1], hover_pos_[2], hover_yaw_); }

    void perform_tracking() {
        lost_count_++;
        if (lost_count_ > 40) {
            target_captured_ = false;
            return;
        }

        double ref_x, ref_y;
        if (locked_ || det_window_.empty()) {
            ref_x = target_info_.x;
            ref_y = target_info_.y;
        } else {
            double sum_x = 0, sum_y = 0;
            for (const auto& p : det_window_) {
                sum_x += p.first;
                sum_y += p.second;
            }
            ref_x = sum_x / det_window_.size();
            ref_y = sum_y / det_window_.size();
        }

        const double DEADBAND_X = 30.0;
        const double DEADBAND_Y = 80.0;

        double raw_err_x = ref_x - center_x_;
        double raw_err_y = ref_y - center_y_;

        double err_pix_x = 0.0, err_pix_y = 0.0;

        if (std::abs(raw_err_x) >= DEADBAND_X) {
            err_pix_x = (std::abs(raw_err_x) - DEADBAND_X) * (raw_err_x > 0 ? 1.0 : -1.0);
        }

        if (std::abs(raw_err_y) >= DEADBAND_Y) {
            err_pix_y = (std::abs(raw_err_y) - DEADBAND_Y) * (raw_err_y > 0 ? 1.0 : -1.0);
        }

        double error_yaw = 0.0, error_z = 0.0, error_x = 0.0;

        if (!locked_) {
            error_yaw = err_pix_x * -0.005;
        } else {
            error_yaw = err_pix_x * -0.005;
            error_z = err_pix_y * -0.005;
            error_x = (1.0 - target_info_.d / target_height_ref_);
            if (std::abs(error_x) < 0.05) error_x = 0;
        }

        double dynamic_limit_x = std::min(0.5 + 2.5 * std::abs(error_x), 3.0);
        pid_x_.set_limits(-dynamic_limit_x, dynamic_limit_x);

        double now = this->now().seconds();
        double target_vel_x = pid_x_.compute(error_x, now);
        double target_vel_z = pid_z_.compute(error_z, now);
        double target_vel_yaw = pid_yaw_.compute(error_yaw, now);

        double dt = now - last_ctrl_time_;
        if (dt <= 0) dt = 0.05;
        last_ctrl_time_ = now;

        auto apply_ramp = [](double target, double current, double limit, double dt) {
            double max_step = limit * dt;
            double diff = target - current;
            double step = std::max(std::min(diff, max_step), -max_step);
            return current + step;
        };

        double smooth_vel_x = apply_ramp(target_vel_x, last_vel_x_, accel_limit_x_, dt);
        double smooth_vel_z = apply_ramp(target_vel_z, last_vel_z_, accel_limit_z_, dt);
        double smooth_vel_yaw = apply_ramp(target_vel_yaw, last_vel_yaw_, accel_limit_yaw_, dt);

        smooth_vel_x = std::max(-max_vel_x_, std::min(max_vel_x_, smooth_vel_x));
        smooth_vel_z = std::max(-max_vel_z_, std::min(max_vel_z_, smooth_vel_z));
        smooth_vel_yaw = std::max(-max_vel_yaw_, std::min(max_vel_yaw_, smooth_vel_yaw));

        last_vel_x_ = smooth_vel_x;
        last_vel_z_ = smooth_vel_z;
        last_vel_yaw_ = smooth_vel_yaw;

        geometry_msgs::msg::TwistStamped vel_msg;
        vel_msg.header.stamp = this->now();
        vel_msg.header.frame_id = "base_link";
        vel_msg.twist.linear.x = smooth_vel_x;
        vel_msg.twist.linear.y = 0.0;
        vel_msg.twist.linear.z = smooth_vel_z;
        vel_msg.twist.angular.x = 0.0;
        vel_msg.twist.angular.y = 0.0;
        vel_msg.twist.angular.z = smooth_vel_yaw;

        vel_pub_->publish(vel_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YoloOffboardTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
