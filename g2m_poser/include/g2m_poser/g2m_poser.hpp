#ifndef _G2M_POSER_HPP_
#define _G2M_POSER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <iomanip>

#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Geoid.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

#include <tf2/transform_datatypes.h>

#ifdef USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2/buffer_core.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

namespace g2m_poser
{
class G2MPoser : public rclcpp::Node
{
public:
  explicit G2MPoser(const rclcpp::NodeOptions &node_options);

private:
  void callbackFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr fix_msg);
  void callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);
  void callbackOdom(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr odom_msg);

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_fix_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr sub_odom_;
  
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_gnss_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_gnss_pose_cov_;

  tf2::BufferCore tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_broadcaster_;

  Eigen::Vector3d orig_pos_;

  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr prev_gnss_pose_cov_;

  geometry_msgs::msg::Quaternion::SharedPtr gnss_quat_;

  sensor_msgs::msg::Imu::ConstSharedPtr curr_imu_;
  autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr curr_odom_;

  std::string heading_source_str_;
  bool use_tf_publish_;
  std::string target_frame_id_;
  double pose_diff_for_heading_;
};
} // namespace g2m_poser

#endif // _G2M_POSER_HPP_
