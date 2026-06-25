/**
 * @file /include/qt_joystick/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
#ifndef qt_joystick_QNODE_HPP_
#define qt_joystick_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif

#include <string>

#include <QDebug>
#include <QThread>
#include <QStringListModel>

#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace qt_joystick {

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv);
    virtual ~QNode();
    bool init();
    void run();
    void Get_data(int *data);
    static void state_cb(const mavros_msgs::msg::State::SharedPtr msg);

Q_SIGNALS:
    void rosShutdown();

private:
    int init_argc;
    char** init_argv;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;

    float Send_data_[6] = {0, 0, 0, 0, 0, 0};
};

}  // namespace qt_joystick

#endif /* qt_joystick_QNODE_HPP_ */
