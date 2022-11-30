// Copyright 2022. The ICHTHUS Project. All Rights Reserved.
// Taeho Han (sethut1224@gmail.com) and
// Kanghee Kim (kim.kanghee@gmail.com) all rights reserved
// MISYS Laboratory, Soongsil University.
// added by ICHTHUS, Taeho Hanon 20221026

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
#include <mission_manager_msgs/msg/checkpoints_with_lane_id.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include "behavior_path_planner/scene_module/lane_change/lane_change_path.hpp"
#include "behavior_path_planner/scene_module/lane_change/util.hpp"
#include "behavior_path_planner/utilities.hpp"
#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/scene_module/lane_change/lane_change_module.hpp"
#include "behavior_path_planner/utilities.hpp"
#include "behavior_path_planner/path_utilities.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/bellman_ford_shortest_paths.hpp>
#include <boost/graph/graph_utility.hpp>
#include <graph_creator.h>
#include <bellman_ford.h>
#include <graph_printer.h>

namespace route_publisher_pm
{
using lanelet::utils::to2D;
using RouteSections = std::vector<autoware_auto_mapping_msgs::msg::HADMapSegment>;
using behavior_path_planner::PlannerData;
using behavior_path_planner::LaneChangeParameters;
using behavior_path_planner::LaneChangePath;
using autoware_auto_perception_msgs::msg::ObjectClassification;

using namespace boost;
using VertexPropertyType = property<vertex_name_t, std::string, property<vertex_color_t, default_color_type>>;
using EdgePropertyType = property<edge_weight_t, int, property<edge_color_t, default_color_type>>;
using DirectedGraphType = adjacency_list<vecS, vecS, directedS, VertexPropertyType, EdgePropertyType>;
using VertexDescriptor = graph_traits<DirectedGraphType>::vertex_descriptor;
// using WeightMap = boost::property_map<Graph, int EdgeProperties::*>::type;

enum TurnIndicator : uint8_t
{
  DISABLE = 1,
  LEFT = 2,
  RIGHT = 3,
};

enum LaneChangeResult : size_t
{
  SUCCESS = 1,
  NOT_YET = 2,
  BLOCKED = 3,
  CHANGE_PARAM = 4,
  NORMAL_FAIL = 5,
  BLOCKED_FAIL = 6,
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
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_lanelet_visualize;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kinematic_state_subscriber;
  rclcpp::Subscription<mission_manager_msgs::msg::CheckpointsWithLaneId>::SharedPtr checkpoint_with_lane_id_subscriber_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_subscriber_;

  void kinematicStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void checkPointInfoCallback(const mission_manager_msgs::msg::CheckpointsWithLaneId::SharedPtr checkpoints_info_msgs);
  void makePlan(const lanelet::ConstLanelet& goal_lanelet, geometry_msgs::msg::PoseStamped& goal);
  void findTargetLaneletAndIndex(const lanelet::ConstLanelet& target_start_lanelet, double target_length, lanelet::ConstLanelet* goal_lanelet, uint32_t* target_index);
  void trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory_msg_ptr);

  bool is_lane_keeping_ready_;

  geometry_msgs::msg::PoseStamped lane_keeping_goal_pose;
  
  BehaviorPathPlannerParameters getCommonParam();
  LaneChangeParameters getLaneChangeParam();

  std::vector<size_t> shortestPath();

  rclcpp::Publisher<autoware_auto_planning_msgs::msg::PathWithLaneId>::SharedPtr debug_lane_change_path_publisher_;
  geometry_msgs::msg::PoseStamped findLaneGoal(long int checkpoint_lane_id);
  size_t findLane(long int checkpoint_lane_id);
  size_t findLaneinSlope(long int checkpoint_lane_id);
  LaneChangeResult makeLaneChangePath(geometry_msgs::msg::Pose cur_checkpoint, geometry_msgs::msg::Pose next_checkpoint, size_t lane_diff, bool speed_up_lane, lanelet::ConstLanelet& second_lanelet);
  bool needNextCheckpointPlanning(autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory_ptr, geometry_msgs::msg::Pose checkpoint_pose);
  lanelet::ConstLanelets getCurrentLanes();
  lanelet::ConstLanelets getLaneChangeLanes(const lanelet::ConstLanelets & current_lanes, const double lane_change_lane_length);
  bool isLaneBlocked(const lanelet::ConstLanelets & lanes);
  std::pair<bool, bool> getSafePath(const lanelet::ConstLanelets & lane_change_lanes, const double check_distance,
  LaneChangePath & safe_path);
  size_t current_checkpoint_index_{1};
  std::vector<size_t> checkpoint_lane_indices_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::TwistStamped current_twist_;
  mission_manager_msgs::msg::CheckpointsWithLaneId checkpoints_info_msgs_;
  autoware_auto_planning_msgs::msg::HADMapRoute current_route_;
  bool current_pose_ok_{false};
  std::vector<std::vector<long int>> lane_id_infos_;
  std::vector<std::vector<long int>> slope_id_infos_;
  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory_;
  std::shared_ptr<PlannerData> planner_data_;
  LaneChangeParameters parameters_;
  lanelet::Lanelet closest_lanelet;
  std::vector<uint32_t> target_lane_ids;
  size_t prev_planning_lane_;
  bool left_lane_change;
  bool right_lane_change;
  int on_lane_count{0};
  LaneChangeResult last_result_{LaneChangeResult::SUCCESS};
  bool pub_end_goal_{false};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;


};
}

#endif
