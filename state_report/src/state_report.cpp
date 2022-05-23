#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"

namespace state_report
{
class StateReportNode : public rclcpp::Node
{
    private:
        rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VehicleKinematicState>::SharedPtr vehicle_kinematic_state_sub{};

        rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
        turn_indicators_command_sub{};

        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr
        vehicle_velocity_report_pub{};

        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
        turn_indicators_report_pub{};

        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr
        steering_report_pub{};

    public:
        StateReportNode(const rclcpp::NodeOptions & options) : Node("state_report", options)
        {   
            vehicle_velocity_report_pub = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("output/vehicle_velocity_report", rclcpp::QoS{10});

            turn_indicators_report_pub =
            this->create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>("output/turn_indicators_report", rclcpp::QoS{10});

            steering_report_pub =
            this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("output/steering_report", rclcpp::QoS{10});

            vehicle_kinematic_state_sub = this->create_subscription<autoware_auto_vehicle_msgs::msg::VehicleKinematicState>("input/vehicle_kinematic_state", rclcpp::QoS{10},
            [this](autoware_auto_vehicle_msgs::msg::VehicleKinematicState::SharedPtr msg)
            {
                autoware_auto_vehicle_msgs::msg::SteeringReport steering_report;
                steering_report.stamp = msg->header.stamp;
                steering_report.steering_tire_angle = msg->state.front_wheel_angle_rad;
                steering_report_pub->publish(steering_report);
                autoware_auto_vehicle_msgs::msg::VelocityReport vehicle_velocity_report;
                vehicle_velocity_report.header = msg->header;
                vehicle_velocity_report.longitudinal_velocity = msg->state.longitudinal_velocity_mps;
                vehicle_velocity_report.lateral_velocity = msg->state.lateral_velocity_mps;
                vehicle_velocity_report.heading_rate = msg->state.heading_rate_rps;
                vehicle_velocity_report_pub->publish(vehicle_velocity_report);
            });

            turn_indicators_command_sub =
            this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>("input/turn_indicators_command",rclcpp::QoS{10},
            [this](autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr msg)
            {
                autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport turn_indicators_report;
                turn_indicators_report.stamp = msg->stamp;
                turn_indicators_report.report = msg->command;
                turn_indicators_report_pub->publish(turn_indicators_report);
            });
        }
};
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(state_report::StateReportNode)