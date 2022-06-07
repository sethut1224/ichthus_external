#include <rclcpp/rclcpp.hpp>
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"

namespace ichthus_vehicle_interface
{

    class IchthusVehicleInterfaceNode : public rclcpp::Node
    {
        private:
            rclcpp::Publisher <autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_velocity_report_;
            rclcpp::Publisher <autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr pub_steering_report_;
            // rclcpp::Subscription
            double publish_rate_;
            double wheel_base_;
            double max_steer_angle_;

        public:
            explicit IchthusVehicleInterfaceNode(const rclcpp::NodeOptions & node_options);
            //void callback();
            
    };
}