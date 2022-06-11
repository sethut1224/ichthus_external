#include "ichthus_vehicle_interface/ichthus_vehicle_interface.hpp"

namespace ichthus_vehicle_interface
{
    IchthusVehicleInterfaceNode::IchthusVehicleInterfaceNode()
        : Node("ichthus_vehicle_interface")
    {
        pub_ref_ang = this->create_publisher<std_msgs::msg::Float64>("ref_ang", 10);
        pub_ref_vel = this->create_publisher<std_msgs::msg::Float64>("ref_vel", 10);

        pub_velocity_report_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("/output/velocity_report", rclcpp::QoS{10});
        pub_steering_report_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("/output/steering_report", rclcpp::QoS{10});

        pub_yaw = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("can_odom", 1);
        pub_mps = this->create_publisher<std_msgs::msg::Float64>("can_mps", 1);

        sub_ctrl_cmd = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>("/input/control_cmd", rclcpp::QoS{1}\
                            ,std::bind(&IchthusVehicleInterfaceNode::ctrlCmdCB, this, _1));
        sub_odom = this->create_subscription<ichthus_msgs::msg::Can>("odom_raw",rclcpp::QoS{1}\
                            ,std::bind(&IchthusVehicleInterfaceNode::odomCB, this, _1));
        
        use_raw_odom = this->declare_parameter<bool>("use_raw_odom", false);

        if(use_raw_odom)
        {
            sub_ESP = this->create_subscription<std_msgs::msg::Float64MultiArray>("ESP12", rclcpp::QoS{1}, std::bind(&IchthusVehicleInterfaceNode::callback_ESP, this, _1));
            sub_SPD = this->create_subscription<std_msgs::msg::Float64MultiArray>("WHL_SPD11", rclcpp::QoS{1}, std::bind(&IchthusVehicleInterfaceNode::callback_SPD, this, _1));
            sub_SAS = this->create_subscription<std_msgs::msg::Float64MultiArray>("SAS11", rclcpp::QoS{1}, std::bind(&IchthusVehicleInterfaceNode::callback_SAS, this, _1));
        }
    }

     void IchthusVehicleInterfaceNode::callback_SAS(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        temp_tier_angle = msg->data[0] / STEERING_GEAR_RATIO;
    }

    void IchthusVehicleInterfaceNode::callback_ESP(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        temp_yaw_rate = msg->data[9];
    }

    void IchthusVehicleInterfaceNode::callback_SPD(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        temp_cur_vel = (msg->data[0] + msg->data[1] + msg->data[2] + msg->data[3]) / 4.0;
    }

    void IchthusVehicleInterfaceNode::publish_status_from_raw_odom()
    {
        auto t = this->now();

        auto velocity_report_msg = autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr(
            new autoware_auto_vehicle_msgs::msg::VelocityReport);
        auto steering_report_msg = autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr(
           new autoware_auto_vehicle_msgs::msg::SteeringReport);

        velocity_report_msg->header.stamp = t;
        steering_report_msg->stamp = t;
        velocity_report_msg->header.frame_id = "base_link";

        velocity_report_msg->heading_rate = DEGtoRAD(temp_yaw_rate);
        velocity_report_msg->longitudinal_velocity = KMPHtoMPS(temp_cur_vel);

        steering_report_msg->steering_tire_angle = DEGtoRAD(temp_tier_angle);
        pub_velocity_report_->publish(*velocity_report_msg);
        pub_steering_report_->publish(*steering_report_msg);
    }

    void IchthusVehicleInterfaceNode::ctrlCmdCB(
            const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg){
        std_msgs::msg::Float64 vel;
        std_msgs::msg::Float64 ang;

        vel.data = MPStoKMPH(msg->longitudinal.speed);
        ang.data = RADtoDEG(msg->lateral.steering_tire_angle) * STEERING_GEAR_RATIO; // is this rad?

        pub_ref_vel->publish(vel);
        pub_ref_ang->publish(ang);
    }

    void IchthusVehicleInterfaceNode::odomCB(const ichthus_msgs::msg::Can::SharedPtr msg){
        auto velocity_report_msg = autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr(
            new autoware_auto_vehicle_msgs::msg::VelocityReport);
        auto steering_report_msg = autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr(
           new autoware_auto_vehicle_msgs::msg::SteeringReport);
        auto twist_covariance_msg =  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr(
            new geometry_msgs::msg::TwistWithCovarianceStamped);
        auto velocity_mps_msg = std_msgs::msg::Float64::SharedPtr(
            new std_msgs::msg::Float64);


        for(size_t i = 0; i < msg->can_data.size(); i++)
        {
            if(msg->can_names[i] == "STR_ANG")
            {
                steer_wheel_angle = msg->can_data[i];
            }

            else if(msg->can_names[i] == "CUR_VEL")
            {
                cur_velocity = msg->can_data[i];
            }

            else if(msg->can_names[i] == "YAW_RATE")
            {
                yaw_rate = msg->can_data[i];
            }
        }
        
        auto t = msg->head.stamp;
        
        float steer_tire_angle = steer_wheel_angle / STEERING_GEAR_RATIO;

        //Velocity Report
        velocity_report_msg->header.stamp = t;
        velocity_report_msg->header.frame_id = "base_link";
        velocity_report_msg->heading_rate = DEGtoRAD(yaw_rate);
        velocity_report_msg->longitudinal_velocity = KMPHtoMPS(cur_velocity);
        
        //Steering Report
        steering_report_msg->stamp = t;
        steering_report_msg->steering_tire_angle = DEGtoRAD(steer_tire_angle);
        
        //TwistWithCovarianceStamped for Lidar localization
        twist_covariance_msg->header.stamp = t;
        twist_covariance_msg->header.frame_id = "base_link";
        twist_covariance_msg->twist.twist.linear.x = KMPHtoMPS(cur_velocity);
        twist_covariance_msg->twist.twist.linear.y = 0.0;
        twist_covariance_msg->twist.twist.angular.z = DEGtoRAD(yaw_rate);
        
        //Velocity Report for V2X
        velocity_mps_msg->data = KMPHtoMPS(cur_velocity);

        pub_velocity_report_->publish(*velocity_report_msg);
        pub_steering_report_->publish(*steering_report_msg);
        pub_yaw->publish(*twist_covariance_msg);
        pub_mps->publish(*velocity_mps_msg);
    }
} // namespace ichthus_vehicle_interface