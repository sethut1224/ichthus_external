#include "ichthus_vehicle_interface/ichthus_vehicle_interface.hpp"

namespace ichthus_vehicle_interface
{
    IchthusVehicleInterfaceNode::IchthusVehicleInterfaceNode(const rclcpp::NodeOptions &node_options)
        : Node("ichthus_vehicle_interface", node_options)
    {
        pub_velocity_report_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("output/velocity_report", rclcpp::QoS{10});
        pub_steering_report_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("output/steering_report", rclcpp::QoS{10});
        
        wheel_base_ = declare_parameter<double>("wheel_base");
        max_steer_angle_ = declare_parameter<double>("max_steer_angle");

        publish_rate_ = declare_parameter<double>("publish_rate");
        // const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(publish_rate_));
        // timer_ = rclcpp::create_timer(this, this->get_clock(), period_ns, std::bind(&IchthusVehicleInterfaceNode::onTimer, this));
    }

    // void IchthusVehicleInterfaceNode::callback()
    // {
    //     auto velocity_report_msg = autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr(
    //         new autoware_auto_vehicle_msgs::msg::VelocityReport);

    //     auto steering_report_msg = autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr(
    //         new autoware_auto_vehicle_msgs::msg::SteeringReport);

    //     pub_velocity_report_->publish(*velocity_report_msg);
    //     pub_steering_report_->publish(*steering_report_msg);
    // }
} // namespace ichthus_vehicle_interface
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ichthus_vehicle_interface::IchthusVehicleInterfaceNode)