#include <rclcpp/rclcpp.hpp>
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>

// For control msg
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "ichthus_msgs/msg/can.hpp"

using std::placeholders::_1;

#define KMPHtoMPS(kmph) ((kmph) / (3.6))
#define MPStoKMPH(mps) ((mps) * (3.6))
#define DEGtoRAD(deg) ((deg) * (0.017453))
#define RADtoDEG(rad) ((rad) / (0.017453))

#define STEERING_GEAR_RATIO 13.3

#define STR_ANG_IDX   0     //  deg
#define STR_VEL_IDX   1     //  deg/s or deg/h
#define CUR_VEL_IDX   2     //  km/h
#define LAT_ACC_IDX   3     //  m/s^2
#define LON_ACC_IDX   4     //  m/s^2
#define YAW_RATE_IDX  5     //  deg/s^2
#define GEAR_IDX      6     

namespace ichthus_vehicle_interface
{

    class IchthusVehicleInterfaceNode : public rclcpp::Node
    {
        private:
            // rclcpp::Publisher
            rclcpp::Publisher <autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr \
                pub_velocity_report_;
            rclcpp::Publisher <autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr \
                pub_steering_report_;
            rclcpp::Publisher <std_msgs::msg::Float64>::SharedPtr pub_ref_vel;
            rclcpp::Publisher <std_msgs::msg::Float64>::SharedPtr pub_ref_ang;


            rclcpp::Publisher <std_msgs::msg::Float64>::SharedPtr pub_cur_vel;
            rclcpp::Publisher <std_msgs::msg::Float64>::SharedPtr pub_cur_ang;

            rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr \
                pub_yaw;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_mps;

            // rclcpp::Subscription
            rclcpp::Subscription<ichthus_msgs::msg::Can>::SharedPtr sub_odom;
            rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr \
                sub_ctrl_cmd;

            rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_ESP;
            rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_SPD;
            rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_SAS;
            
        public:

            float steer_wheel_angle;
            //float steer_wheel_velocity;
            float cur_velocity;
            //float lon_acc;
            //float lat_acc;
            float yaw_rate;
            //float cur_gear;

            double temp_yaw_rate;
            double temp_cur_vel;
            double temp_tier_angle;
            bool use_raw_odom;
            // explicit IchthusVehicleInterfaceNode(const rclcpp::NodeOptions & node_options);
            IchthusVehicleInterfaceNode();

            void ctrlCmdCB(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr);
            void odomCB(const ichthus_msgs::msg::Can::SharedPtr);

            void callback_ESP(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
            void callback_SPD(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
            void callback_SAS(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
            void publish_status_from_raw_odom();
            /*
            void spdCB(const std_msgs::msg::Float64MultiArray::SharedPtr);
            void yawCB(const std_msgs::msg::Float64MultiArray::SharedPtr);
            void angCB(const std_msgs::msg::Float64MultiArray::SharedPtr);
            */

    };
}