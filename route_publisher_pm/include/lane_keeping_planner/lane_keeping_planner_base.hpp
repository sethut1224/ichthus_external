// Copyright 2022. The ICHTHUS Project. All Rights Reserved.
// Taeho Han (sethut1224@gmail.com) and
// Kanghee Kim (kim.kanghee@gmail.com) all rights reserved
// MISYS Laboratory, Soongsil University.
// added by ICHTHUS, Taeho Hanon 20221026

#ifndef MISSION_PLANNER__MISSION_PLANNER_BASE_HPP_
#define MISSION_PLANNER__MISSION_PLANNER_BASE_HPP_

#include <string>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Autoware
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace route_publisher_pm
{
class LaneKeepingPlanner : public rclcpp::Node
{
protected:
  LaneKeepingPlanner(const std::string & node_name, const rclcpp::NodeOptions & node_options);

  geometry_msgs::msg::PoseStamped goal_pose_;
  geometry_msgs::msg::PoseStamped start_pose_;
  std::vector<geometry_msgs::msg::PoseStamped> checkpoints_;

  std::string base_link_frame_;
  std::string map_frame_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

  virtual bool isRoutingGraphReady() const = 0;
  virtual void visualizeRoute(
    const autoware_auto_planning_msgs::msg::HADMapRoute & route) const = 0;
  virtual void publishRoute(const autoware_auto_planning_msgs::msg::HADMapRoute & route) const;

private:
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::HADMapRoute>::SharedPtr route_publisher_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace lane_keeeping_planner
#endif
