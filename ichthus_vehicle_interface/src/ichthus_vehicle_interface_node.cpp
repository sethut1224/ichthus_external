#include "ichthus_vehicle_interface/ichthus_vehicle_interface.hpp"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ichthus_vehicle_interface::IchthusVehicleInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

}