

#include <g2m_poser/g2m_poser.hpp>

namespace
{
// geometry_msgs::msg::TwistStamped calcTwist(
//   geometry_msgs::msg::PoseStamped::SharedPtr pose_a,
//   geometry_msgs::msg::PoseStamped::SharedPtr pose_b)
// {
//   const double dt =
//     (rclcpp::Time(pose_b->header.stamp) - rclcpp::Time(pose_a->header.stamp)).seconds();

//   if (dt == 0) {
//     geometry_msgs::msg::TwistStamped twist;
//     twist.header = pose_b->header;
//     return twist;
//   }

//   const auto pose_a_rpy = getRPY(pose_a);
//   const auto pose_b_rpy = getRPY(pose_b);

//   geometry_msgs::msg::Vector3 diff_xyz;
//   geometry_msgs::msg::Vector3 diff_rpy;

//   diff_xyz.x = pose_b->pose.position.x - pose_a->pose.position.x;
//   diff_xyz.y = pose_b->pose.position.y - pose_a->pose.position.y;
//   diff_xyz.z = pose_b->pose.position.z - pose_a->pose.position.z;
//   diff_rpy.x = calcDiffForRadian(pose_b_rpy.x, pose_a_rpy.x);
//   diff_rpy.y = calcDiffForRadian(pose_b_rpy.y, pose_a_rpy.y);
//   diff_rpy.z = calcDiffForRadian(pose_b_rpy.z, pose_a_rpy.z);

//   geometry_msgs::msg::TwistStamped twist;
//   twist.header = pose_b->header;
//   twist.twist.linear.x =
//     std::sqrt(std::pow(diff_xyz.x, 2.0) + std::pow(diff_xyz.y, 2.0) + std::pow(diff_xyz.z, 2.0)) /
//     dt;
//   twist.twist.linear.y = 0;
//   twist.twist.linear.z = 0;
//   twist.twist.angular.x = diff_rpy.x / dt;
//   twist.twist.angular.y = diff_rpy.y / dt;
//   twist.twist.angular.z = diff_rpy.z / dt;

//   return twist;
// }
}
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
  base_link_frame_ = this->declare_parameter("base_link_frame", "base_link"); 
  pose_diff_for_heading_ = this->declare_parameter("pose_diff_for_heading", 0.2); 
  max_buffer_size_ = this->declare_parameter("max_buffer_size", 3); 

  std::string orig_x = this->declare_parameter<std::string>("origin_x", "0.0");
  std::string orig_y = this->declare_parameter<std::string>("origin_y", "0.0");
  std::string orig_z = this->declare_parameter<std::string>("origin_z", "0.0");
  orig_pos_(0) = std::stod(orig_x);
  orig_pos_(1) = std::stod(orig_y);
  orig_pos_(2) = std::stod(orig_z);

  use_b2g_tf_listener_ = this->declare_parameter("use_b2g_tf_listener", true); 
  b2g_tf_ = Eigen::Matrix4d::Identity();
  b2g_tf_(0, 3) = this->declare_parameter("b2g_x", 0.0);
  b2g_tf_(1, 3) = this->declare_parameter("b2g_y", 0.0);
  b2g_tf_(2, 3) = this->declare_parameter("b2g_z", 0.0);
 
  std::cout << std::setprecision(16);
  std::cout << "str origin_x: " << orig_x << std::endl;
  std::cout << "str origin_y: " << orig_y << std::endl;
  std::cout << "str origin_z: " << orig_z << std::endl;

  std::cout << "db origin_x: " << orig_pos_(0) << std::endl;
  std::cout << "db origin_y: " << orig_pos_(1) << std::endl;
  std::cout << "db origin_z: " << orig_pos_(2) << std::endl;

  sub_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    this->declare_parameter("fix_topic", "/fix"), 
    rclcpp::SensorDataQoS().keep_last(1),
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
    // rclcpp::SensorDataQoS().keep_last(64));
    64);


  pub_gnss_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    this->declare_parameter("pose_topic", "/gnss_pose"), 
    // rclcpp::SensorDataQoS().keep_last(64));
    rclcpp::QoS(64));

  pub_gnss_twist_cov_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    this->declare_parameter("twist_cov_topic", "/gnss_twist_cov"), 
    rclcpp::SensorDataQoS().keep_last(64));
}

bool G2MPoser::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = this->get_clock()->now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = this->get_clock()->now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

void G2MPoser::callbackFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr fix_msg) 
{
  geometry_msgs::msg::PoseStamped::UniquePtr curr_gnss_pose(new geometry_msgs::msg::PoseStamped);
  geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr curr_gnss_pose_cov(new geometry_msgs::msg::PoseWithCovarianceStamped);
  geometry_msgs::msg::TwistWithCovarianceStamped::UniquePtr curr_gnss_twist_cov(new geometry_msgs::msg::TwistWithCovarianceStamped);

  curr_gnss_pose->header.stamp = fix_msg->header.stamp;
  curr_gnss_pose->header.frame_id = "map";

  curr_gnss_pose_cov->header.stamp = fix_msg->header.stamp;
  curr_gnss_pose_cov->header.frame_id = "map";

  curr_gnss_twist_cov->header.stamp = fix_msg->header.stamp;
  curr_gnss_twist_cov->header.frame_id = "map";

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
  curr_gnss_pose_cov->pose.pose.position.z = fix_msg->altitude;
  
  curr_gnss_pose_cov->pose.pose.position.x -= orig_pos_(0);
  curr_gnss_pose_cov->pose.pose.position.y -= orig_pos_(1);
  curr_gnss_pose_cov->pose.pose.position.z -= orig_pos_(2);

  if (prev_gnss_pose_cov_ != nullptr &&
      gnss_quat_ != nullptr)
  {

    // double diff_time = (rclcpp::Time(curr_gnss_pose_cov->header.stamp) - rclcpp::Time(prev_gnss_pose_cov_->header.stamp)).seconds();

    // double diff_x = curr_gnss_pose_cov->pose.pose.position.x - prev_gnss_pose_cov_->pose.pose.position.x;
    // double diff_y = curr_gnss_pose_cov->pose.pose.position.y - prev_gnss_pose_cov_->pose.pose.position.y;
    // double diff_z = curr_gnss_pose_cov->pose.pose.position.z - prev_gnss_pose_cov_->pose.pose.position.z;
    // double diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

    // double diff_yaw = tf2::getYaw(curr_gnss_pose_cov->pose.pose.orientation) - 
    //                   tf2::getYaw(prev_gnss_pose_cov_->pose.pose.orientation);

    // curr_gnss_twist_cov->twist.twist.linear.x = diff / diff_time;
    // curr_gnss_twist_cov->twist.twist.linear.y = 0.0;
    // curr_gnss_twist_cov->twist.twist.linear.z = 0.0;
    
    // curr_gnss_twist_cov->twist.twist.angular.x = 0.0;
    // curr_gnss_twist_cov->twist.twist.angular.y = 0.0;
    // curr_gnss_twist_cov->twist.twist.angular.z = diff_yaw / diff_time;

    if (heading_source_str_ == "NONE")
    {
      double distance = std::hypot(curr_gnss_pose_cov->pose.pose.position.x - prev_gnss_pose_cov_->pose.pose.position.x,
                                  curr_gnss_pose_cov->pose.pose.position.y - prev_gnss_pose_cov_->pose.pose.position.y);

      if (distance > pose_diff_for_heading_)
      {
        double yaw = std::atan2(curr_gnss_pose_cov->pose.pose.position.y - prev_gnss_pose_cov_->pose.pose.position.y, 
                                curr_gnss_pose_cov->pose.pose.position.x - prev_gnss_pose_cov_->pose.pose.position.x);
        
        gnss_yaw_buffer_.push_back(yaw);
        if (gnss_yaw_buffer_.size() > max_buffer_size_)
        {
          gnss_yaw_buffer_.pop_front();
        }

        double avg_yaw = std::accumulate(gnss_yaw_buffer_.begin(), gnss_yaw_buffer_.end(), 0.0) / 
          static_cast<double>(gnss_yaw_buffer_.size());

        tf2::Quaternion quat;
        quat.setRPY(0, 0, avg_yaw);
        *gnss_quat_ = tf2::toMsg(quat);
        *prev_gnss_pose_cov_ = *curr_gnss_pose_cov;      

        // double yaw = std::atan2(curr_gnss_pose_cov->pose.pose.position.y - prev_gnss_pose_cov_->pose.pose.position.y, 
        //                         curr_gnss_pose_cov->pose.pose.position.x - prev_gnss_pose_cov_->pose.pose.position.x);
        
        // tf2::Quaternion quat;
        // quat.setRPY(0, 0, yaw);
        // *gnss_quat_ = tf2::toMsg(quat);
        // *prev_gnss_pose_cov_ = *curr_gnss_pose_cov;      
      }
    }

    curr_gnss_pose_cov->pose.pose.orientation = *gnss_quat_;
  }

  if (use_b2g_tf_listener_)
  {
    geometry_msgs::msg::TransformStamped::SharedPtr tf_base2gnss_ptr =
      std::make_shared<geometry_msgs::msg::TransformStamped>();
    getTransform(base_link_frame_, "gnss", tf_base2gnss_ptr); 

    const auto yaw = std::atan2(tf_base2gnss_ptr->transform.translation.y, tf_base2gnss_ptr->transform.translation.x) +
      tf2::getYaw(curr_gnss_pose_cov->pose.pose.orientation);
    const auto b2g_xy_dist = std::hypot(tf_base2gnss_ptr->transform.translation.x, tf_base2gnss_ptr->transform.translation.y);
    curr_gnss_pose_cov->pose.pose.position.x += (-b2g_xy_dist * std::cos(yaw));  
    curr_gnss_pose_cov->pose.pose.position.y += (-b2g_xy_dist * std::sin(yaw));  
    curr_gnss_pose_cov->pose.pose.position.z += (-tf_base2gnss_ptr->transform.translation.z);  
  }
  else
  {
    Eigen::Translation3d tl(curr_gnss_pose_cov->pose.pose.position.x, 
                            curr_gnss_pose_cov->pose.pose.position.y, 
                            curr_gnss_pose_cov->pose.pose.position.z);
    Eigen::AngleAxisd rot_x(0.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rot_y(0.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_z(tf2::getYaw(curr_gnss_pose_cov->pose.pose.orientation), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d tfd_pose = ((tl * rot_z * rot_y * rot_x) * b2g_tf_.inverse()).matrix().block<3, 1>(0, 3);
    
    curr_gnss_pose_cov->pose.pose.position.x = tfd_pose(0);
    curr_gnss_pose_cov->pose.pose.position.y = tfd_pose(1);
    curr_gnss_pose_cov->pose.pose.position.z = tfd_pose(2);
  }

  curr_gnss_pose->pose = curr_gnss_pose_cov->pose.pose;

  if (use_tf_publish_)
  {
    geometry_msgs::msg::TransformStamped gnss_tf;
    gnss_tf.header.stamp = fix_msg->header.stamp;
    gnss_tf.header.frame_id = "map";
    gnss_tf.child_frame_id = base_link_frame_;

    gnss_tf.transform.translation.x = curr_gnss_pose_cov->pose.pose.position.x;
    gnss_tf.transform.translation.y = curr_gnss_pose_cov->pose.pose.position.y;
    gnss_tf.transform.translation.z = curr_gnss_pose_cov->pose.pose.position.z;
    gnss_tf.transform.rotation = curr_gnss_pose_cov->pose.pose.orientation;

    tf2_broadcaster_.sendTransform(gnss_tf);
  }

  /************************/
  /************************/
  // temporary hard coding
  curr_gnss_pose_cov->pose.covariance = {
    1.0,    0.0,     0.0,     0.0,     0.0,     0.0,
    0.0,    1.0,     0.0,     0.0,     0.0,     0.0,
    0.0,    0.0,     1.0,     0.0,     0.0,     0.0,
    0.0,    0.0,     0.0,     1.0,     0.0,     0.0,
    0.0,    0.0,     0.0,     0.0,     1.0,     0.0,
    0.0,    0.0,     0.0,     0.0,     0.0,     1.0};
  /************************/
  /************************/
  /************************/

  pub_gnss_pose_->publish(std::move(curr_gnss_pose));
  pub_gnss_pose_cov_->publish(std::move(curr_gnss_pose_cov));
  pub_gnss_twist_cov_->publish(std::move(curr_gnss_twist_cov));
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
