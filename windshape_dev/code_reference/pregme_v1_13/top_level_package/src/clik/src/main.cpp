#include <rclcpp/rclcpp.hpp>

#include "clik_main.h"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto clik_node = std::make_shared<clikRos>();
    //the setpoint publishing rate MUST be faster than 2Hz
    rclcpp::Rate rate(50.0);

    while(rclcpp::ok())
    {        
        clik_node->mainLoop();

        rclcpp::spin_some(clik_node);
        rate.sleep();
    }

    rclcpp::shutdown();

    return 0;

}
