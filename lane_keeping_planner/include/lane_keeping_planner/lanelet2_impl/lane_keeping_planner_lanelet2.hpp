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

#ifndef MISSION_PLANNER__LANELET2_IMPL__MISSION_PLANNER_LANELET2_HPP_
#define MISSION_PLANNER__LANELET2_IMPL__MISSION_PLANNER_LANELET2_HPP_

#include <string>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Autoware
#include "lane_keeping_planner/lane_keeping_planner_base.hpp"

#include <route_handler/route_handler.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>

#include <tier4_planning_msgs/msg/approval.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>

// lanelet
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <nav_msgs/msg/path.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>

namespace lane_keeping_planner
{
using lanelet::utils::to2D;
using RouteSections = std::vector<autoware_auto_mapping_msgs::msg::HADMapSegment>;

enum TurnIndicator : uint8_t
{
  DISABLE = 1,
  LEFT = 2,
  RIGHT = 3,
};

class LaneKeepingPlannerLanelet2 : public LaneKeepingPlanner
{
public:
  explicit LaneKeepingPlannerLanelet2(const rclcpp::NodeOptions & node_options);

private:
  bool is_graph_ready_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::ConstLanelets road_lanelets_;
  lanelet::ConstLanelets shoulder_lanelets_;
  route_handler::RouteHandler route_handler_;

  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_subscriber_;

  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg);
  bool isGoalValid() const;

  // virtual functions
  bool isRoutingGraphReady() const;
  autoware_auto_planning_msgs::msg::HADMapRoute planRoute(geometry_msgs::msg::Pose s_pose, geometry_msgs::msg::Pose e_pose);
  void visualizeRoute(const autoware_auto_planning_msgs::msg::HADMapRoute & route) const;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_goal_pose_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_goal_pose_publisher2;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_lanelet_visualize;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lane_keeping_start_subscriber;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kinematic_state_subscriber;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_indicators_subscriber;

  void startCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void kinematicStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void turnIndicatorCallback(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::SharedPtr msg);
  void makePlan(const lanelet::ConstLanelet& goal_lanelet, geometry_msgs::msg::PoseStamped& goal);
  void findTargetLaneletAndIndex(const lanelet::ConstLanelet& target_start_lanelet, double target_length, lanelet::ConstLanelet* goal_lanelet, uint32_t* target_index);
  bool isLaneKeepingReady() const;

  bool is_lane_keeping_ready_;
  
  geometry_msgs::msg::PoseStamped::SharedPtr prev_pose_ptr;
  geometry_msgs::msg::PoseStamped lane_keeping_goal_pose;
  lanelet::Lanelet closest_lanelet;
  std::vector<uint32_t> target_lane_ids;
  std::deque<TurnIndicator> indicator_deque;
  bool left_lane_change;
  bool right_lane_change;

  TurnIndicator prev_indicator;
  TurnIndicator current_indicator;
  geometry_msgs::msg::PoseStamped current_pose;
};
}

#endif
