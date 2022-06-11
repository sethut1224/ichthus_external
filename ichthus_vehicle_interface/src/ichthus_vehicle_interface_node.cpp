#include "ichthus_vehicle_interface/ichthus_vehicle_interface.hpp"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ichthus_vehicle_interface::IchthusVehicleInterfaceNode>();
    rclcpp::Rate loop_rate(100);
    auto use_spin_some = node->use_raw_odom;

    while(rclcpp::ok())
    {
        if(use_spin_some)
        {
            rclcpp::spin_some(node);
            node->publish_status_from_raw_odom();
            loop_rate.sleep();
        }
        else
        {
            rclcpp::spin(node);
        }
    }
    rclcpp::shutdown();

}