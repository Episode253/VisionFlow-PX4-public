#include <rclcpp/rclcpp.hpp>
#include "clik_main.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto clikRos_obj = std::make_shared<clikRos>();
    
    // the setpoint publishing rate MUST be faster than 2Hz
    rclcpp::WallRate rate(20.0);
    
    // wait for FCU connection (保留了你注释掉的逻辑)
    // while(rclcpp::ok() && !clikRos_obj->current_state.connected)
    // {
    //     printf("1\n");
    //     rclcpp::spin_some(clikRos_obj);
    //     rate.sleep();
    // }

    while(rclcpp::ok())
    {
        // 完美对应原版 clikRos_obj.mainLoop();
        clikRos_obj->mainLoop();

        // 完美对应原版 ros::spinOnce();
        rclcpp::spin_some(clikRos_obj);
        
        // 完美对应原版 rate.sleep();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}