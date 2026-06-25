#ifndef QT_JOYSTICK_SIM_REMOTE_NODE_HPP_
#define QT_JOYSTICK_SIM_REMOTE_NODE_HPP_

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif

#include <array>
#include <mutex>
#include <string>

#include <QThread>

#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace qt_joystick {

class SimRemoteNode : public QThread {
    Q_OBJECT

public:
    SimRemoteNode(int argc, char **argv);
    ~SimRemoteNode() override;

    bool init();
    void run() override;

    void setRcAxes(const std::array<float, 7> &axes);
    void setRcButtons(const std::array<int, 8> &buttons);
    void setJointTargets(const std::array<double, 6> &joint_targets);

    bool armVehicle(bool arm);
    bool setFlightMode(const std::string &custom_mode);

    bool hasState() const;
    bool isVehicleArmed() const;
    std::string currentMode() const;
    bool isVehicleConnected() const;

Q_SIGNALS:
    void rosShutdown();

private:
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);

    int init_argc_;
    char **init_argv_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscriber_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

    mutable std::mutex data_mutex_;
    std::array<float, 7> rc_axes_ {{0.f, 0.f, 1.f, 0.f, -1.f, 0.f, 0.f}};
    std::array<int, 8> rc_buttons_ {{0, 0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 6> joint_targets_ {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    mavros_msgs::msg::State current_state_;
    bool has_state_{false};
};

}  // namespace qt_joystick

#endif  // QT_JOYSTICK_SIM_REMOTE_NODE_HPP_
