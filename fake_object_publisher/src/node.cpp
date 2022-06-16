// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fake_object_publisher/node.hpp"

#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

ObjectInfo::ObjectInfo(
  const dummy_perception_publisher::msg::Object & object, const rclcpp::Time & current_time, std::vector<double>& move_distance_vec)
: length(object.shape.dimensions.x),
  width(object.shape.dimensions.y),
  height(object.shape.dimensions.z),
  std_dev_x(std::sqrt(object.initial_state.pose_covariance.covariance[0])),
  std_dev_y(std::sqrt(object.initial_state.pose_covariance.covariance[7])),
  std_dev_z(std::sqrt(object.initial_state.pose_covariance.covariance[14])),
  std_dev_yaw(std::sqrt(object.initial_state.pose_covariance.covariance[35]))
{
  double move_distance = move_distance_vec[3] * (current_time.seconds() - rclcpp::Time(object.header.stamp).seconds());
  move_distance = move_distance_vec[3];

  move_distance_vec[0] = move_distance_vec[0] + move_distance;

  tf2::Transform tf_object_origin2moved_object;
  tf2::Transform tf_map2object_origin;
  {
    geometry_msgs::msg::Transform ros_object_origin2moved_object;
    ros_object_origin2moved_object.translation.x = move_distance_vec[0];
    ros_object_origin2moved_object.translation.y = move_distance_vec[1];
    ros_object_origin2moved_object.rotation.x = 0;
    ros_object_origin2moved_object.rotation.y = 0;
    ros_object_origin2moved_object.rotation.z = move_distance_vec[2];
    ros_object_origin2moved_object.rotation.w = 1;

    tf2::fromMsg(ros_object_origin2moved_object, tf_object_origin2moved_object);
  }
  tf2::fromMsg(object.initial_state.pose_covariance.pose, tf_map2object_origin);
  this->tf_map2moved_object = tf_map2object_origin * tf_object_origin2moved_object;
}

DummyPerceptionPublisherNode::DummyPerceptionPublisherNode()
: Node("fake_object_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  move_distance_vec_.resize(4, 0.0);

  visible_range_ = this->declare_parameter("visible_range", 100.0);
  detection_successful_rate_ = this->declare_parameter("detection_successful_rate", 0.8);
  enable_ray_tracing_ = this->declare_parameter("enable_ray_tracing", true);
  use_object_recognition_ = this->declare_parameter("use_object_recognition", true);

  const bool object_centric_pointcloud =
    this->declare_parameter("object_centric_pointcloud", false);

  if (object_centric_pointcloud) {
    pointcloud_creator_ =
      std::unique_ptr<PointCloudCreator>(new ObjectCentricPointCloudCreator(enable_ray_tracing_));
  } else {
    pointcloud_creator_ =
      std::unique_ptr<PointCloudCreator>(new EgoCentricPointCloudCreator(visible_range_));
  }

  // parameters for vehicle centric point cloud generation
  angle_increment_ = this->declare_parameter("angle_increment", 0.25 * M_PI / 180.0);

  std::random_device seed_gen;
  random_generator_.seed(seed_gen());

  rclcpp::QoS qos{1};
  qos.transient_local();
  detected_object_autoware_msg_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>("output/fake_objects", qos);

  detected_object_with_feature_pub_ =
    this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
      "output/dynamic_object", qos);
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/points_raw", qos);
  object_sub_ = this->create_subscription<dummy_perception_publisher::msg::Object>(
    "input/object", 100,
    std::bind(&DummyPerceptionPublisherNode::objectCallback, this, std::placeholders::_1));

  cmd_vel_sub_ = this->create_subscription<std_msgs::msg::String>(
    "teleop_key", 100,
    std::bind(&DummyPerceptionPublisherNode::cmdvelCallback, this, std::placeholders::_1));

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&DummyPerceptionPublisherNode::timerCallback, this));

  
  
}

void DummyPerceptionPublisherNode::timerCallback()
{
  // output msgs
  tier4_perception_msgs::msg::DetectedObjectsWithFeature output_dynamic_object_msg;
  geometry_msgs::msg::PoseStamped output_moved_object_pose;
  sensor_msgs::msg::PointCloud2 output_pointcloud_msg;
  std_msgs::msg::Header header;
  rclcpp::Time current_time = this->now();

  // avoid terminal contamination.
  static rclcpp::Time failed_tf_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  if ((this->now() - failed_tf_time).seconds() < 5.0) {
    return;
  }

  std::string error;
  if (!tf_buffer_.canTransform("base_link", /*src*/ "map", tf2::TimePointZero, &error)) {
    failed_tf_time = this->now();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "map->base_link is not available yet");
    return;
  }

  tf2::Transform tf_base_link2map;
  try {
    geometry_msgs::msg::TransformStamped ros_base_link2map;
    ros_base_link2map = tf_buffer_.lookupTransform(
      /*target*/ "base_link", /*src*/ "map", current_time, rclcpp::Duration::from_seconds(0.5));
    tf2::fromMsg(ros_base_link2map.transform, tf_base_link2map);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
    return;
  }

  std::vector<size_t> selected_indices{};
  static std::uniform_real_distribution<> detection_successful_random(0.0, 1.0);
  for (size_t i = 0; i < objects_.size(); ++i) {
    if (detection_successful_rate_ >= detection_successful_random(random_generator_)) {
      selected_indices.push_back(i);
    }
  }

  if (selected_indices.empty()) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::toROSMsg(*merged_pointcloud_ptr, output_pointcloud_msg);
  } else {
    std::vector<ObjectInfo> obj_infos;
    for (const auto selected_idx : selected_indices) {
      const auto obj_info = ObjectInfo(objects_.at(selected_idx), current_time, move_distance_vec_);
      tf2::toMsg(obj_info.tf_map2moved_object, output_moved_object_pose.pose);
      obj_infos.push_back(obj_info);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    const auto pointclouds = pointcloud_creator_->create_pointclouds(
      obj_infos, tf_base_link2map, random_generator_, merged_pointcloud_ptr);
    pcl::toROSMsg(*merged_pointcloud_ptr, output_pointcloud_msg);

    std::vector<size_t> delete_idxs;
    for (size_t i = 0; i < selected_indices.size(); ++i) {
      const auto pointcloud = pointclouds[i];
      const size_t selected_idx = selected_indices[i];
      const auto & object = objects_.at(selected_idx);
      const auto object_info = ObjectInfo(object, current_time, move_distance_vec_);
      // dynamic object
      std::normal_distribution<> x_random(0.0, object_info.std_dev_x);
      std::normal_distribution<> y_random(0.0, object_info.std_dev_y);
      std::normal_distribution<> yaw_random(0.0, object_info.std_dev_yaw);
      tf2::Quaternion noised_quat;
      noised_quat.setRPY(0, 0, yaw_random(random_generator_));
      tf2::Transform tf_moved_object2noised_moved_object(
        noised_quat, tf2::Vector3(x_random(random_generator_), y_random(random_generator_), 0.0));
      tf2::Transform tf_base_link2noised_moved_object;
      tf_base_link2noised_moved_object =
        tf_base_link2map * object_info.tf_map2moved_object * tf_moved_object2noised_moved_object;
      tier4_perception_msgs::msg::DetectedObjectWithFeature feature_object;
      feature_object.object.classification.push_back(object.classification);
      feature_object.object.kinematics.pose_with_covariance = object.initial_state.pose_covariance;
      feature_object.object.kinematics.twist_with_covariance =
        object.initial_state.twist_covariance;
      feature_object.object.kinematics.orientation_availability =
        autoware_auto_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE;
      feature_object.object.kinematics.has_twist = false;
      tf2::toMsg(
        tf_base_link2noised_moved_object,
        feature_object.object.kinematics.pose_with_covariance.pose);
      feature_object.object.shape = object.shape;
      pcl::toROSMsg(*pointcloud, feature_object.feature.cluster);
      output_dynamic_object_msg.feature_objects.push_back(feature_object);

      // check delete idx
      tf2::Transform tf_base_link2moved_object;
      tf_base_link2moved_object = tf_base_link2map * object_info.tf_map2moved_object;
      double dist = std::sqrt(
        tf_base_link2moved_object.getOrigin().x() * tf_base_link2moved_object.getOrigin().x() +
        tf_base_link2moved_object.getOrigin().y() * tf_base_link2moved_object.getOrigin().y());
      if (visible_range_ < dist) {
        delete_idxs.push_back(selected_idx);
      }
    }

    // delete
    for (int delete_idx = delete_idxs.size() - 1; 0 <= delete_idx; --delete_idx) {
      objects_.erase(objects_.begin() + delete_idxs.at(delete_idx));
    }
  }

  // create output header
  output_moved_object_pose.header.frame_id = "map";
  output_moved_object_pose.header.stamp = current_time;
  output_dynamic_object_msg.header.frame_id = "base_link";
  output_dynamic_object_msg.header.stamp = current_time;
  output_pointcloud_msg.header.frame_id = "base_link";
  output_pointcloud_msg.header.stamp = current_time;

  // publish
  pointcloud_pub_->publish(output_pointcloud_msg);
  if (use_object_recognition_) {
    detected_object_with_feature_pub_->publish(output_dynamic_object_msg);

    // tier4_msg to autoware_msgs
    autoware_auto_perception_msgs::msg::DetectedObjects output_msg;

    output_msg.header = output_dynamic_object_msg.header;

    for (size_t i = 0; i < output_dynamic_object_msg.feature_objects.size(); ++i) {
      autoware_auto_perception_msgs::msg::DetectedObject obj;
      obj.existence_probability = 1.0;
      obj.kinematics.orientation_availability = autoware_auto_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
      obj.classification = output_dynamic_object_msg.feature_objects[i].object.classification;
      obj.kinematics.pose_with_covariance.pose.position = output_dynamic_object_msg.feature_objects[i].object.kinematics.pose_with_covariance.pose.position;
      obj.kinematics.pose_with_covariance.pose.orientation = output_dynamic_object_msg.feature_objects[i].object.kinematics.pose_with_covariance.pose.orientation;
      obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
      obj.shape.dimensions = output_dynamic_object_msg.feature_objects[i].object.shape.dimensions;
      obj.kinematics.twist_with_covariance = output_dynamic_object_msg.feature_objects[i].object.kinematics.twist_with_covariance;
      obj.kinematics.has_twist = output_dynamic_object_msg.feature_objects[i].object.kinematics.has_twist;

      output_msg.objects.emplace_back(obj);
    }
    detected_object_autoware_msg_pub_->publish(output_msg);
  }
}

void DummyPerceptionPublisherNode::cmdvelCallback(
  const std_msgs::msg::String::ConstSharedPtr msg)  
{
  // 0 : x , 1 : y , 2 : yaw
  key_ = msg->data;

  if(key_ == "w")
  {
    move_distance_vec_[0] += 0.5;
    move_distance_vec_[1] += 0.0;
    move_distance_vec_[2] += 0.0;
    move_distance_vec_[3] += 0.0;
  }
  else if(key_ == "s")
  {
    move_distance_vec_[0] += -0.5;
    move_distance_vec_[1] += 0.0;
    move_distance_vec_[2] += 0.0;
    move_distance_vec_[3] += 0.0;
  }
  else if(key_ == "a")
  {
    move_distance_vec_[0] += 0.0;
    move_distance_vec_[1] += -0.5;
    move_distance_vec_[2] += 0.0;
    move_distance_vec_[3] += 0.0;
  }
  else if(key_ == "d")
  {
    move_distance_vec_[0] += 0.0;
    move_distance_vec_[1] += 0.5;
    move_distance_vec_[2] += 0.0;
    move_distance_vec_[3] += 0.0;
  }
  else if(key_ == "q")
  {
    move_distance_vec_[0] += 0.0;
    move_distance_vec_[1] += 0.0;
    move_distance_vec_[2] += 0.1;
    move_distance_vec_[3] += 0.0;
  }
  else if(key_ == "e")
  {
    move_distance_vec_[0] += 0.0;
    move_distance_vec_[1] += 0.0;
    move_distance_vec_[2] += -0.1;
    move_distance_vec_[3] += 0.0;
  }
  else if(key_ == "g")
  {
    move_distance_vec_[0] += 0.0;
    move_distance_vec_[1] += 0.0;
    move_distance_vec_[2] += 0.0;
    move_distance_vec_[3] += 0.1;
  }
  else if(key_ == "h")
  {
    move_distance_vec_[0] += 0.0;
    move_distance_vec_[1] += 0.0;
    move_distance_vec_[2] += 0.0;
    move_distance_vec_[3] += -0.1;
  }
  else if(key_ == "p")
  {
    move_distance_vec_[0] = 0.0;
    move_distance_vec_[1] = 0.0;
    move_distance_vec_[2] = 0.0;
    move_distance_vec_[3] = 0.0;
  }
  else
  {
    move_distance_vec_[0] += 0.0;
    move_distance_vec_[1] += 0.0;
    move_distance_vec_[2] += 0.0;
    move_distance_vec_[3] += 0.0;
  }
}

void DummyPerceptionPublisherNode::objectCallback(
  const dummy_perception_publisher::msg::Object::ConstSharedPtr msg)
{
  switch (msg->action) {
    case dummy_perception_publisher::msg::Object::ADD: {
      tf2::Transform tf_input2map;
      tf2::Transform tf_input2object_origin;
      tf2::Transform tf_map2object_origin;
      try {
        geometry_msgs::msg::TransformStamped ros_input2map;
        ros_input2map = tf_buffer_.lookupTransform(
          /*target*/ msg->header.frame_id, /*src*/ "map", msg->header.stamp,
          rclcpp::Duration::from_seconds(0.5));
        tf2::fromMsg(ros_input2map.transform, tf_input2map);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
        return;
      }
      tf2::fromMsg(msg->initial_state.pose_covariance.pose, tf_input2object_origin);
      tf_map2object_origin = tf_input2map.inverse() * tf_input2object_origin;
      dummy_perception_publisher::msg::Object object;
      object = *msg;
      tf2::toMsg(tf_map2object_origin, object.initial_state.pose_covariance.pose);

      // Use base_link Z
      geometry_msgs::msg::TransformStamped ros_map2base_link;
      try {
        ros_map2base_link = tf_buffer_.lookupTransform(
          "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
        object.initial_state.pose_covariance.pose.position.z =
          ros_map2base_link.transform.translation.z;
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
        return;
      }

      objects_.push_back(object);
      break;
    }
    case dummy_perception_publisher::msg::Object::DELETE: {
      std::cout << "DELETE" << std::endl;
      for (size_t i = 0; i < objects_.size(); ++i) {
        if (objects_.at(i).id.uuid == msg->id.uuid) {
          objects_.erase(objects_.begin() + i);
          break;
        }
      }
      break;
    }
    case dummy_perception_publisher::msg::Object::MODIFY: {
      std::cout << "MODIFY" << std::endl;
      for (size_t i = 0; i < objects_.size(); ++i) {
        if (objects_.at(i).id.uuid == msg->id.uuid) {
          tf2::Transform tf_input2map;
          tf2::Transform tf_input2object_origin;
          tf2::Transform tf_map2object_origin;
          try {
            geometry_msgs::msg::TransformStamped ros_input2map;
            ros_input2map = tf_buffer_.lookupTransform(
              /*target*/ msg->header.frame_id, /*src*/ "map", msg->header.stamp,
              rclcpp::Duration::from_seconds(0.5));
            tf2::fromMsg(ros_input2map.transform, tf_input2map);
          } catch (tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
            return;
          }
          tf2::fromMsg(msg->initial_state.pose_covariance.pose, tf_input2object_origin);
          tf_map2object_origin = tf_input2map.inverse() * tf_input2object_origin;
          dummy_perception_publisher::msg::Object object;
          objects_.at(i) = *msg;
          tf2::toMsg(tf_map2object_origin, objects_.at(i).initial_state.pose_covariance.pose);

          // Use base_link Z
          geometry_msgs::msg::TransformStamped ros_map2base_link;
          try {
            ros_map2base_link = tf_buffer_.lookupTransform(
              "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
            objects_.at(i).initial_state.pose_covariance.pose.position.z =
              ros_map2base_link.transform.translation.z;
          } catch (tf2::TransformException & ex) {
            RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
            return;
          }
          break;
        }
      }
      break;
    }
    case dummy_perception_publisher::msg::Object::DELETEALL: {
      objects_.clear();
      break;
    }
  }
}
