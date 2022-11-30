// Copyright 2019 Autoware Foundation
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

#include "lane_keeping_planner/lane_keeping_planner_base.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>

#include <lanelet2_routing/Route.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <visualization_msgs/msg/marker_array.h>

#include <string>


namespace route_publisher_pm
{
LaneKeepingPlanner::LaneKeepingPlanner(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: Node(node_name, node_options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  map_frame_ = declare_parameter("map_frame", "map");
  base_link_frame_ = declare_parameter("base_link_frame", "base_link");

  using std::placeholders::_1;

  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  route_publisher_ =
    create_publisher<autoware_auto_planning_msgs::msg::HADMapRoute>("output/route", durable_qos);
  marker_publisher_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("debug/route_marker", durable_qos);
}

void LaneKeepingPlanner::publishRoute(const autoware_auto_planning_msgs::msg::HADMapRoute & route) const
{
  if (!route.segments.empty()) {
    RCLCPP_INFO(get_logger(), "Route successfully planned. Publishing...");
    route_publisher_->publish(route);
    visualizeRoute(route);
  } else {
    RCLCPP_ERROR(get_logger(), "Calculated route is empty!");
  }
}

}
