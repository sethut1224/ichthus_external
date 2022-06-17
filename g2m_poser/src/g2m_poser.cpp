

#include <g2m_poser/g2m_poser.hpp>

namespace g2m_poser
{
G2MPoser::G2MPoser(const rclcpp::NodeOptions &node_options)
: rclcpp::Node("g2m_poser", node_options),
  tf2_listener_(tf2_buffer_, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false, 
    tf2_ros::DynamicListenerQoS().durability_volatile(), 
    tf2_ros::StaticListenerQoS().durability_volatile()),
  tf2_broadcaster_(*this),
  prev_gnss_pose_cov_(nullptr),
  gnss_quat_(new geometry_msgs::msg::Quaternion)
{
  use_tf_publish_ = this->declare_parameter("use_tf_publish", false); 
  target_frame_id_ = this->declare_parameter("target_frame_id", "gnss"); 
  pose_diff_for_heading_ = this->declare_parameter("pose_diff_for_heading", 0.4); 
  std::string orig_x = this->declare_parameter<std::string>("origin_x", "0.0");
  std::string orig_y = this->declare_parameter<std::string>("origin_y", "0.0");
  std::string orig_z = this->declare_parameter<std::string>("origin_z", "0.0");
  orig_pos_(0) = std::stod(orig_x);
  orig_pos_(1) = std::stod(orig_y);
  orig_pos_(2) = std::stod(orig_z);
 
  std::cout << std::setprecision(16);
  std::cout << "str origin_x: " << orig_x << std::endl;
  std::cout << "str origin_y: " << orig_y << std::endl;
  std::cout << "str origin_z: " << orig_z << std::endl;

  std::cout << "db origin_x: " << orig_pos_(0) << std::endl;
  std::cout << "db origin_y: " << orig_pos_(1) << std::endl;
  std::cout << "db origin_z: " << orig_pos_(2) << std::endl;

  sub_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    this->declare_parameter("fix_topic", "/fix"), 
    rclcpp::SensorDataQoS().keep_last(64),
    std::bind(&G2MPoser::callbackFix, this, std::placeholders::_1));

  heading_source_str_ = this->declare_parameter("heading_source", "NONE"); // NONE=0, IMU=1, ODOM=2
  if (heading_source_str_ == "IMU")
  {
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      this->declare_parameter("imu_topic", "/imu/data"), 
      rclcpp::SensorDataQoS().keep_last(64),
      std::bind(&G2MPoser::callbackImu, this, std::placeholders::_1));
    
    sub_odom_ = nullptr;
  }
  else if (heading_source_str_ == "ODOM")
  {
    sub_odom_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
      this->declare_parameter("odom_topic", "/output/velocity_report"), 
      rclcpp::SensorDataQoS().keep_last(64),
      std::bind(&G2MPoser::callbackOdom, this, std::placeholders::_1));
    
    sub_imu_ = nullptr;
  }
  else
  {
    /* do Nothing */ 
    sub_imu_ = nullptr;
    sub_odom_ = nullptr;

    prev_gnss_pose_cov_.reset(new geometry_msgs::msg::PoseWithCovarianceStamped);
  }

  pub_gnss_pose_cov_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    this->declare_parameter("pose_cov_topic", "/gnss_pose_cov"), 
    rclcpp::SensorDataQoS().keep_last(64));

  pub_gnss_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    this->declare_parameter("pose_topic", "/gnss_pose"), 
    rclcpp::SensorDataQoS().keep_last(64));
}

void G2MPoser::callbackFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr fix_msg) 
{
  geometry_msgs::msg::PoseStamped::UniquePtr curr_gnss_pose(new geometry_msgs::msg::PoseStamped);
  geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr curr_gnss_pose_cov(new geometry_msgs::msg::PoseWithCovarianceStamped);

  int zone;
  bool northp;
  try {
  GeographicLib::UTMUPS::Forward(fix_msg->latitude, fix_msg->longitude, 
    zone, northp,
    curr_gnss_pose_cov->pose.pose.position.x, curr_gnss_pose_cov->pose.pose.position.y);
  } catch (const GeographicLib::GeographicErr & err) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(), "Failed to convert from LLH to UTM" << err.what());
  }

  // try {
  //   GeographicLib::Geoid egm2008("egm2008-1");
  //   curr_gnss_pose_cov->pose.pose.position.z = egm2008.ConvertHeight(
  //     fix_msg->latitude, fix_msg->longitude, fix_msg->altitude,
  //     GeographicLib::Geoid::ELLIPSOIDTOGEOID);
  // } catch (const GeographicLib::GeographicErr & err) {
  //   RCLCPP_ERROR_STREAM(
  //     this->get_logger(), "Failed to convert Height from Ellipsoid to Orthometric" << err.what());
  // }
  curr_gnss_pose_cov->pose.pose.position.z = fix_msg->altitude;

  // std::cout << "utm: " << curr_gnss_pose_cov->pose.pose.position.x << " " 
  //   << curr_gnss_pose_cov->pose.pose.position.y << " " 
  //   << curr_gnss_pose_cov->pose.pose.position.z << std::endl; 
  
  curr_gnss_pose_cov->pose.pose.position.x -= orig_pos_(0);
  curr_gnss_pose_cov->pose.pose.position.y -= orig_pos_(1);
  curr_gnss_pose_cov->pose.pose.position.z -= orig_pos_(2);

  if (prev_gnss_pose_cov_ != nullptr &&
      gnss_quat_ != nullptr)
  {
    if (heading_source_str_ == "NONE")
    {
      // double distance = sqrt(pow(pose.pose.position.y - _prev_pose.pose.position.y, 2) +
                            //  pow(pose.pose.position.x - _prev_pose.pose.position.x, 2));
      double distance = std::hypot(curr_gnss_pose_cov->pose.pose.position.x - prev_gnss_pose_cov_->pose.pose.position.x,
                                  curr_gnss_pose_cov->pose.pose.position.y - prev_gnss_pose_cov_->pose.pose.position.y);
      std::cout << "distance : " << distance << std::endl;
      if (distance > pose_diff_for_heading_)
      {
        double yaw = std::atan2(curr_gnss_pose_cov->pose.pose.position.y - prev_gnss_pose_cov_->pose.pose.position.y, 
                                curr_gnss_pose_cov->pose.pose.position.x - prev_gnss_pose_cov_->pose.pose.position.x);
        
        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw);
        *gnss_quat_ = tf2::toMsg(quat);

        *prev_gnss_pose_cov_ = *curr_gnss_pose_cov;      
      }
    }

    curr_gnss_pose_cov->pose.pose.orientation = *gnss_quat_;
  }

  curr_gnss_pose->pose = curr_gnss_pose_cov->pose.pose;

  curr_gnss_pose->header.stamp = fix_msg->header.stamp;
  curr_gnss_pose->header.frame_id = "map";

  curr_gnss_pose_cov->header.stamp = fix_msg->header.stamp;
  curr_gnss_pose_cov->header.frame_id = "map";

  // Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  // Eigen::Quaternionf quat;
  // quat.normalize();
  // geometry_msgs::msg::Quaternion gnss_quat;
  // gnss_quat.w = quat.w();
  // gnss_quat.x = quat.x();
  // gnss_quat.y = quat.y();
  // gnss_quat.z = quat.z();
  if (use_tf_publish_)
  {
    geometry_msgs::msg::TransformStamped gnss_tf;
    gnss_tf.header.stamp = fix_msg->header.stamp;
    gnss_tf.header.frame_id = "map";
    gnss_tf.child_frame_id = target_frame_id_;

    gnss_tf.transform.translation.x = curr_gnss_pose_cov->pose.pose.position.x;
    gnss_tf.transform.translation.y = curr_gnss_pose_cov->pose.pose.position.y;
    gnss_tf.transform.translation.z = curr_gnss_pose_cov->pose.pose.position.z;
    gnss_tf.transform.rotation = curr_gnss_pose_cov->pose.pose.orientation;

    tf2_broadcaster_.sendTransform(gnss_tf);
  }
  pub_gnss_pose_->publish(std::move(curr_gnss_pose));
  pub_gnss_pose_cov_->publish(std::move(curr_gnss_pose_cov));
}

void G2MPoser::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) 
{
  curr_imu_ = imu_msg;
}

void G2MPoser::callbackOdom(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr odom_msg) 
{
  curr_odom_ = odom_msg;
}


}  // namespace g2m_poser

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(g2m_poser::G2MPoser)
