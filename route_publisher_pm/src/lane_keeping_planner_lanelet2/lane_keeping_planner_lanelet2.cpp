
// Copyright 2022. The ICHTHUS Project. All Rights Reserved.
// Taeho Han (sethut1224@gmail.com) and
// Kanghee Kim (kim.kanghee@gmail.com) all rights reserved
// MISYS Laboratory, Soongsil University.
// added by ICHTHUS, Taeho Hanon 20221026
    
#include "lane_keeping_planner/lanelet2_impl/lane_keeping_planner_lanelet2.hpp"

#include "lane_keeping_planner/lanelet2_impl/utility_functions.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <tf2/utils.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <limits>
#include <memory>
#include <unordered_set>

#include <vehicle_info_util/vehicle_info_util.hpp>
#include <boost/graph/graphviz.hpp>

using vehicle_info_util::VehicleInfoUtil;

namespace
{
using RouteSections = std::vector<autoware_auto_mapping_msgs::msg::HADMapSegment>;
RouteSections combineConsecutiveRouteSections(
  const RouteSections & route_sections1, const RouteSections & route_sections2)
{
  RouteSections route_sections;
  route_sections.reserve(route_sections1.size() + route_sections2.size());
  if (!route_sections1.empty()) {
    // remove end route section because it is overlapping with first one in next route_section
    route_sections.insert(route_sections.end(), route_sections1.begin(), route_sections1.end() - 1);
  }
  if (!route_sections2.empty()) {
    route_sections.insert(route_sections.end(), route_sections2.begin(), route_sections2.end());
  }
  return route_sections;
}

bool isRouteLooped(const RouteSections & route_sections)
{
  for (std::size_t i = 0; i < route_sections.size(); i++) {
    const auto & route_section = route_sections.at(i);
    for (const auto & lane_id : route_section.primitives) {
      for (std::size_t j = i + 1; j < route_sections.size(); j++) {
        const auto & future_route_section = route_sections.at(j);
        if (exists(future_route_section.primitives, lane_id)) {
          return true;
        }
      }
    }
  }
  return false;
}

double normalizeRadian(const double rad, const double min_rad = -M_PI, const double max_rad = M_PI)
{
  const auto value = std::fmod(rad, 2 * M_PI);
  if (min_rad < value && value <= max_rad) {
    return value;
  } else {
    return value - std::copysign(2 * M_PI, value);
  }
}

bool isInLane(const lanelet::ConstLanelet & lanelet, const lanelet::ConstPoint3d & point)
{
  // check if goal is on a lane at appropriate angle
  const auto distance = boost::geometry::distance(
    lanelet.polygon2d().basicPolygon(), lanelet::utils::to2D(point).basicPoint());
  constexpr double th_distance = std::numeric_limits<double>::epsilon();
  return distance < th_distance;
}

bool isInParkingSpace(
  const lanelet::ConstLineStrings3d & parking_spaces, const lanelet::ConstPoint3d & point)
{
  for (const auto & parking_space : parking_spaces) {
    lanelet::ConstPolygon3d parking_space_polygon;
    if (!lanelet::utils::lineStringWithWidthToPolygon(parking_space, &parking_space_polygon)) {
      continue;
    }

    const double distance = boost::geometry::distance(
      lanelet::utils::to2D(parking_space_polygon).basicPolygon(),
      lanelet::utils::to2D(point).basicPoint());
    constexpr double th_distance = std::numeric_limits<double>::epsilon();
    if (distance < th_distance) {
      return true;
    }
  }
  return false;
}

bool isInParkingLot(
  const lanelet::ConstPolygons3d & parking_lots, const lanelet::ConstPoint3d & point)
{
  for (const auto & parking_lot : parking_lots) {
    const double distance = boost::geometry::distance(
      lanelet::utils::to2D(parking_lot).basicPolygon(), lanelet::utils::to2D(point).basicPoint());
    constexpr double th_distance = std::numeric_limits<double>::epsilon();
    if (distance < th_distance) {
      return true;
    }
  }
  return false;
}

geometry_msgs::msg::Quaternion QuaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
  
}

}  // anonymous namespace

namespace route_publisher_pm
{
LaneKeepingPlannerLanelet2::LaneKeepingPlannerLanelet2(const rclcpp::NodeOptions & node_options)
: LaneKeepingPlanner("lane_keeping_planner_node", node_options), is_graph_ready_(false), left_lane_change(false), right_lane_change(false)
{
  using std::placeholders::_1;
  map_subscriber_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "input/vector_map", rclcpp::QoS{10}.transient_local(),
    std::bind(&LaneKeepingPlannerLanelet2::mapCallback, this, _1));

  kinematic_state_subscriber = create_subscription<nav_msgs::msg::Odometry>("/localization/kinematic_state", 1, std::bind(&LaneKeepingPlannerLanelet2::kinematicStateCallback, this, _1)
  );
  trajectory_subscriber_ = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>("/planning/scenario_planning/trajectory", 1, std::bind(&LaneKeepingPlannerLanelet2::trajectoryCallback, this, _1));

  debug_goal_pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("/debug/goal_pose_visualizer",10);
  debug_lanelet_visualize = create_publisher<nav_msgs::msg::Path>("/debug/lanelet_visualize",10);
  debug_lane_change_path_publisher_ = create_publisher<autoware_auto_planning_msgs::msg::PathWithLaneId>("/debug/lane_change_path", rclcpp::QoS{10}.transient_local());
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vector_map_debug", 10);
  checkpoint_with_lane_id_subscriber_ = create_subscription<mission_manager_msgs::msg::CheckpointsWithLaneId>("/mission_manager_info", rclcpp::QoS{10}, std::bind(&LaneKeepingPlannerLanelet2::checkPointInfoCallback, this, _1));

  // std::vector<long int> third_lane_info{2347,206, 277, 516, 683, 927, 1147, 1271, 2382, 2419, 2620, 2911, 2944, 3009, 1625, 1746, 2072, 2338, 2383, 2631, 2628, 2639, 2640, 2634, 2724, 2719, 2710, 2896, 2895, 2894, 2922, 2910, 2952, 2327};

  std::vector<long int> third_lane_info {2347, 206, 277, 516, 3065, 3126, 3209, 3253, 927, 1147, 1271, 2382, 2419, 2620, 2911, 2944, 3009, 1625, 1746, 3350, 3383, 3426, 3513, 3599, 2327, 2338};

  std::vector<long int> second_lane_info{2348,  7,  159, 395, 3081, 3145, 3210, 3270, 879, 1146, 1268, 2484, 2497, 2688, 2965, 2990, 3010, 1624 ,1804, 3351, 3392, 3439, 3570, 3609, 2244, 2337};

  std::vector<long int> first_lane_info {2349, 431, 463, 571, 3094, 3162, 3224, 3325, 975, 1187, 1316, 2492, 2502, 2689, 2971, 2989, 3011, 1634, 1869, 3361, 3402, 3455, 3590, 3610, 2193, 2336};

  std::vector<long int> slope3 {2383, 2631, 2639, 2724, 2896, 2922, 2952};
  std::vector<long int> slope2 {2628, 2640, 2719, 2895, 2910};
  std::vector<long int> slope1 {2634, 2710, 2894};

  slope_id_infos_.push_back(slope1);
  slope_id_infos_.push_back(slope2);
  slope_id_infos_.push_back(slope3);

  lane_id_infos_.push_back(first_lane_info);
  lane_id_infos_.push_back(second_lane_info);
  lane_id_infos_.push_back(third_lane_info);

  planner_data_ = std::make_shared<PlannerData>();
  planner_data_->parameters = getCommonParam();

  parameters_ = getLaneChangeParam();
}

LaneChangeParameters LaneKeepingPlannerLanelet2::getLaneChangeParam()
{
  LaneChangeParameters p{};
  p.min_stop_distance = declare_parameter("min_stop_distance", 5.0);
  p.stop_time = declare_parameter("stop_time", 2.0);
  p.hysteresis_buffer_distance = declare_parameter("hysteresis_buffer_distance", 2.0);
  p.lane_change_prepare_duration = declare_parameter("lane_change_prepare_duration", 2.0);
  p.lane_changing_duration = declare_parameter("lane_changing_duration", 4.0);
  p.lane_change_finish_judge_buffer = declare_parameter("lane_change_finish_judge_buffer", 3.0);
  p.minimum_lane_change_velocity = declare_parameter("minimum_lane_change_velocity", 8.3);
  p.prediction_duration = declare_parameter("prediction_duration", 8.0);
  p.prediction_time_resolution = declare_parameter("prediction_time_resolution", 0.5);
  p.static_obstacle_velocity_thresh = declare_parameter("static_obstacle_velocity_thresh", 0.1);
  p.maximum_deceleration = declare_parameter("maximum_deceleration", 1.0);
  p.lane_change_sampling_num = declare_parameter("lane_change_sampling_num", 10);
  p.enable_abort_lane_change = declare_parameter("enable_abort_lane_change", true);
  p.enable_collision_check_at_prepare_phase = declare_parameter("enable_collision_check_at_prepare_phase", true);
  p.use_predicted_path_outside_lanelet = declare_parameter("use_predicted_path_outside_lanelet", true);
  p.use_all_predicted_path = declare_parameter("use_all_predicted_path", false);
  p.abort_lane_change_velocity_thresh = declare_parameter("abort_lane_change_velocity_thresh", 0.5);
  p.abort_lane_change_angle_thresh =
    declare_parameter("abort_lane_change_angle_thresh", tier4_autoware_utils::deg2rad(10.0));
  p.abort_lane_change_distance_thresh = declare_parameter("abort_lane_change_distance_thresh", 0.3);
  p.enable_blocked_by_obstacle = declare_parameter("enable_blocked_by_obstacle", false);
  p.lane_change_search_distance = declare_parameter("lane_change_search_distance", 30.0);
 
  // validation of parameters
  if (p.lane_change_sampling_num < 1) {
    RCLCPP_FATAL_STREAM(
      get_logger(), "lane_change_sampling_num must be positive integer. Given parameter: "
                      << p.lane_change_sampling_num << std::endl
                      << "Terminating the program...");
    exit(EXIT_FAILURE);
  }
  if (p.maximum_deceleration < 0.0) {
    RCLCPP_FATAL_STREAM(
      get_logger(), "maximum_deceleration cannot be negative value. Given parameter: "
                      << p.maximum_deceleration << std::endl
                      << "Terminating the program...");
    exit(EXIT_FAILURE);
  }

  return p;
}

BehaviorPathPlannerParameters LaneKeepingPlannerLanelet2::getCommonParam()
{
  // ROS parameters
  BehaviorPathPlannerParameters p{};
  p.backward_path_length = declare_parameter("backward_path_length", 5.0);
  p.forward_path_length = declare_parameter("forward_path_length", 100.0);
  p.backward_length_buffer_for_end_of_lane =
    declare_parameter("backward_length_buffer_for_end_of_lane", 5.0);
  p.backward_length_buffer_for_end_of_pull_over =
    declare_parameter("backward_length_buffer_for_end_of_pull_over", 5.0);
  p.backward_length_buffer_for_end_of_pull_out =
    declare_parameter("backward_length_buffer_for_end_of_pull_out", 5.0);
  p.minimum_lane_change_length = declare_parameter("minimum_lane_change_length", 8.0);
  p.minimum_pull_over_length = declare_parameter("minimum_pull_over_length", 15.0);
  p.drivable_area_resolution = declare_parameter<double>("drivable_area_resolution");
  p.drivable_lane_forward_length = declare_parameter<double>("drivable_lane_forward_length");
  p.drivable_lane_backward_length = declare_parameter<double>("drivable_lane_backward_length");
  p.drivable_lane_margin = declare_parameter<double>("drivable_lane_margin");
  p.drivable_area_margin = declare_parameter<double>("drivable_area_margin");
  p.refine_goal_search_radius_range = declare_parameter("refine_goal_search_radius_range", 7.5);
  p.turn_light_on_threshold_dis_lat = declare_parameter("turn_light_on_threshold_dis_lat", 0.3);
  p.turn_light_on_threshold_dis_long = declare_parameter("turn_light_on_threshold_dis_long", 10.0);
  p.turn_light_on_threshold_time = declare_parameter("turn_light_on_threshold_time", 3.0);
  p.visualize_drivable_area_for_shared_linestrings_lanelet =
    declare_parameter("visualize_drivable_area_for_shared_linestrings_lanelet", true);

  // vehicle info
  const auto vehicle_info = VehicleInfoUtil(*this).getVehicleInfo();
  p.vehicle_width = vehicle_info.vehicle_width_m;
  p.vehicle_length = vehicle_info.vehicle_length_m;
  p.wheel_tread = vehicle_info.wheel_tread_m;
  p.wheel_base = vehicle_info.wheel_base_m;
  p.front_overhang = vehicle_info.front_overhang_m;
  p.rear_overhang = vehicle_info.rear_overhang_m;
  p.left_over_hang = vehicle_info.left_overhang_m;
  p.right_over_hang = vehicle_info.right_overhang_m;
  p.base_link2front = vehicle_info.max_longitudinal_offset_m;
  p.base_link2rear = p.rear_overhang;

  return p;
}

void LaneKeepingPlannerLanelet2::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg)
{
  route_handler_.setMap(*msg);
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
  shoulder_lanelets_ = lanelet::utils::query::shoulderLanelets(all_lanelets);
  is_graph_ready_ = true;
}

bool LaneKeepingPlannerLanelet2::isRoutingGraphReady() const { return is_graph_ready_; }

size_t LaneKeepingPlannerLanelet2::findLaneinSlope(long int checkpoint_lane_id)
{
  for(size_t i = 0 ; i < slope_id_infos_.size(); i++)
  {
    for(size_t j = 0; j < slope_id_infos_.at(i).size(); j++)
    {
      if(checkpoint_lane_id == slope_id_infos_.at(i).at(j))
      {
        return i;
      }
    }
  }

  return 999;
}

size_t LaneKeepingPlannerLanelet2::findLane(long int checkpoint_lane_id)
{
  for(size_t i = 0 ; i < lane_id_infos_.size(); i++)
  {
    for(size_t j = 0; j < lane_id_infos_.at(i).size(); j++)
    {
      if(checkpoint_lane_id == lane_id_infos_.at(i).at(j))
      {
        return i;
      }
    }
  }

  return 1;
}


size_t findClosestIndexInLanelet(lanelet::ConstLanelet& lanelet, geometry_msgs::msg::Point position, double min_dist = 5.0)
{ 
  auto centerline = lanelet.centerline();
  size_t closest_index = 999;
  for (size_t i = 0; i < centerline.size(); i++)
  {
    double distance = std::hypot(
      position.x - centerline[i].x(),
      position.y - centerline[i].y()
    );
    if (distance < min_dist)
    {
      min_dist = distance;
      closest_index = i;
    }
  }
  return closest_index;
}

geometry_msgs::msg::PoseStamped LaneKeepingPlannerLanelet2::findLaneGoal(long int checkpoint_lane_id)
{
  auto lane = findLane(checkpoint_lane_id);
  
  long int goal_lane_id;

  bool pass_goal = false;

  lanelet::ConstLanelet closest_lanelet;
  lanelet::utils::query::getClosestLanelet(road_lanelets_, current_pose_.pose, &closest_lanelet);
  auto closest_lanelet_id = closest_lanelet.id();

  if(closest_lanelet_id == 2347 || closest_lanelet_id == 206 || closest_lanelet_id == 2348 || closest_lanelet_id == 7 || closest_lanelet_id == 2349 || closest_lanelet_id == 431)
  {
    goal_lane_id = lane_id_infos_.at(lane).back();
  }
  else
  {
    pass_goal = true;
    goal_lane_id = lane_id_infos_.at(lane).at(1);
  }
  
  lanelet::ConstLanelet goal_lane = route_handler_.getLaneletsFromId(goal_lane_id);
  auto centerline = goal_lane.centerline2d();


  auto temp_goal_pose = lanelet::utils::conversion::toGeomMsgPt(centerline[centerline.size()-2]);
  if(pass_goal)
    temp_goal_pose = lanelet::utils::conversion::toGeomMsgPt(centerline[static_cast<size_t>(centerline.size() / 2)]);
  auto temp_goal_yaw = lanelet::utils::getLaneletAngle(goal_lane, temp_goal_pose);

  geometry_msgs::msg::PoseStamped goal;
  goal.pose.orientation = QuaternionFromYaw(temp_goal_yaw);
  goal.pose.position = temp_goal_pose;

  return goal;
}

  bool LaneKeepingPlannerLanelet2::needNextCheckpointPlanning(autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory_ptr, geometry_msgs::msg::Pose checkpoint_pose)
  {
    auto current_idx = tier4_autoware_utils::findNearestIndex(trajectory_ptr->points, current_pose_.pose.position);
    auto checkpoint_idx = tier4_autoware_utils::findNearestIndex(trajectory_ptr->points, checkpoint_pose.position);

    auto current_idx2 = tier4_autoware_utils::findNearestIndex(trajectory_ptr->points, current_pose_.pose, 1.0);
    if(!current_idx2 == 0)
      return true;

    if(current_idx > checkpoint_idx)
      return true;
    
    if(checkpoint_idx == trajectory_ptr->points.size() -1)
      return false;

    return true;
  }


void LaneKeepingPlannerLanelet2::visualizeRoute(
  const autoware_auto_planning_msgs::msg::HADMapRoute & route) const
{
  lanelet::ConstLanelets route_lanelets;
  lanelet::ConstLanelets end_lanelets;
  lanelet::ConstLanelets normal_lanelets;
  lanelet::ConstLanelets goal_lanelets;

  for (const auto & route_section : route.segments) {
    for (const auto & lane_id : route_section.primitives) {
      auto lanelet = lanelet_map_ptr_->laneletLayer.get(lane_id.id);
      route_lanelets.push_back(lanelet);
      if (route_section.preferred_primitive_id == lane_id.id) {
        goal_lanelets.push_back(lanelet);
      } else {
        end_lanelets.push_back(lanelet);
      }
    }
  }

  std_msgs::msg::ColorRGBA cl_route, cl_ll_borders, cl_end, cl_normal, cl_goal;
  setColor(&cl_route, 0.2, 0.4, 0.2, 0.05);
  setColor(&cl_goal, 0.2, 0.4, 0.4, 0.05);
  setColor(&cl_end, 0.2, 0.2, 0.4, 0.05);
  setColor(&cl_normal, 0.2, 0.4, 0.2, 0.05);
  setColor(&cl_ll_borders, 1.0, 1.0, 1.0, 0.999);

  visualization_msgs::msg::MarkerArray route_marker_array;
  insertMarkerArray(
    &route_marker_array,
    lanelet::visualization::laneletsBoundaryAsMarkerArray(route_lanelets, cl_ll_borders, false));
  insertMarkerArray(
    &route_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                           "route_lanelets", route_lanelets, cl_route));
  insertMarkerArray(
    &route_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("end_lanelets", end_lanelets, cl_end));
  insertMarkerArray(
    &route_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                           "normal_lanelets", normal_lanelets, cl_normal));
  insertMarkerArray(
    &route_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("goal_lanelets", goal_lanelets, cl_goal));
  marker_publisher_->publish(route_marker_array);
}

bool LaneKeepingPlannerLanelet2::isGoalValid() const
{
  lanelet::Lanelet closest_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(
        road_lanelets_, goal_pose_.pose, &closest_lanelet)) {
    return false;
  }
  const auto goal_lanelet_pt = lanelet::utils::conversion::toLaneletPoint(goal_pose_.pose.position);

  if (isInLane(closest_lanelet, goal_lanelet_pt)) {
    const auto lane_yaw =
      lanelet::utils::getLaneletAngle(closest_lanelet, goal_pose_.pose.position);
    const auto goal_yaw = tf2::getYaw(goal_pose_.pose.orientation);
    const auto angle_diff = normalizeRadian(lane_yaw - goal_yaw);
    
    constexpr double th_angle = M_PI / 4;

    auto b =lanelet::utils::to2D(goal_lanelet_pt).basicPoint();
    std::cout<<"lanelet : "<<b[0]<<", "<<b[1]<<std::endl;
    if (std::abs(angle_diff) < th_angle) {
      return true;
    }
  }
  // check if goal is in parking space
  const auto parking_spaces = lanelet::utils::query::getAllParkingSpaces(lanelet_map_ptr_);
  if (isInParkingSpace(parking_spaces, goal_lanelet_pt)) {
    return true;
  }

  // check if goal is in parking lot
  const auto parking_lots = lanelet::utils::query::getAllParkingLots(lanelet_map_ptr_);
  if (isInParkingLot(parking_lots, goal_lanelet_pt)) {
    return true;
  }

  // check if goal is in shoulder lanelet
  lanelet::Lanelet closest_shoulder_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(
        shoulder_lanelets_, goal_pose_.pose, &closest_shoulder_lanelet)) {
    return false;
  }
  // check if goal pose is in shoulder lane
  if (isInLane(closest_shoulder_lanelet, goal_lanelet_pt)) {
    const auto lane_yaw =
      lanelet::utils::getLaneletAngle(closest_shoulder_lanelet, goal_pose_.pose.position);
    const auto goal_yaw = tf2::getYaw(goal_pose_.pose.orientation);
    const auto angle_diff = normalizeRadian(lane_yaw - goal_yaw);

    constexpr double th_angle = M_PI / 4;
    if (std::abs(angle_diff) < th_angle) {
      return true;
    }
  }

  return false;
}

autoware_auto_planning_msgs::msg::HADMapRoute LaneKeepingPlannerLanelet2::planRoute(geometry_msgs::msg::Pose s_pose, geometry_msgs::msg::Pose e_pose)
{
  autoware_auto_planning_msgs::msg::HADMapRoute route_msg;
  RouteSections route_sections;

  lanelet::ConstLanelets path_lanelets;
  if (!route_handler_.planPathLaneletsBetweenCheckpoints(
        s_pose, e_pose, &path_lanelets)) {
    return route_msg;
  }
  // create local route sections
  route_handler_.setRouteLanelets(path_lanelets);
  const auto local_route_sections = route_handler_.createMapSegments(path_lanelets);
  route_sections = combineConsecutiveRouteSections(route_sections, local_route_sections);
  
  if (isRouteLooped(route_sections)) {
    RCLCPP_WARN(
      get_logger(), "Loop detected within route! Be aware that looped route is not debugged!");
  }

  route_msg.header.stamp = this->now();
  route_msg.header.frame_id = map_frame_;
  route_msg.segments = route_sections;
  route_msg.goal_pose = e_pose;

  return route_msg;
}

void LaneKeepingPlannerLanelet2::trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory_msg_ptr)
{
  trajectory_ = std::make_shared<const autoware_auto_planning_msgs::msg::Trajectory>(*trajectory_msg_ptr);
}

std::vector<size_t> LaneKeepingPlannerLanelet2::shortestPath()
{
  std::map<size_t, std::vector<size_t>> node_level_map;
  std::vector<size_t> skip_list;

  size_t node_level = 0;

  for(size_t i = 0; i < checkpoints_info_msgs_.checkpoints.poses.size(); i++)
  {
    if(std::find(skip_list.begin(), skip_list.end(), i) != skip_list.end())
      continue;

    geometry_msgs::msg::Pose checkpoint_pose = checkpoints_info_msgs_.checkpoints.poses.at(i);
    long int checkpoint_lane_id = checkpoints_info_msgs_.checkpoints_laneid.at(i);
    size_t checkpoint_lane = findLane(checkpoint_lane_id);

    lanelet::ConstLanelet second_lane;
    long int second_lane_id;

    node_level_map.insert(std::pair<size_t, std::vector<size_t>>(node_level, std::vector<size_t>()));
    node_level_map[node_level].push_back(i);
    if(checkpoint_lane != 1)
    {
      for(size_t k = 0 ; i < lane_id_infos_.at(checkpoint_lane).size(); k++)
      {
        if(checkpoint_lane_id == lane_id_infos_.at(checkpoint_lane).at(k))
        {
          second_lane_id = lane_id_infos_.at(1).at(k);
          second_lane = route_handler_.getLaneletsFromId(second_lane_id);
          break;
        }
      }
    }
    else
    {
      second_lane_id = checkpoint_lane_id;
      second_lane = route_handler_.getLaneletsFromId(second_lane_id);
    }

    size_t current_checkpoint_lane_index = findClosestIndexInLanelet(second_lane, checkpoint_pose.position);
    // std::vector<std::vector<size_t>> node_level_vector;

    for(size_t j = i+1; j < checkpoints_info_msgs_.checkpoints.poses.size(); j++)
    {
      geometry_msgs::msg::Pose next_checkpoint_pose = checkpoints_info_msgs_.checkpoints.poses.at(j);

      auto next_checkpoint_lane_index = findClosestIndexInLanelet(second_lane, next_checkpoint_pose.position, 5.5);
      std::cout<<"next checkpoint : "<<second_lane.id()<<", "<<next_checkpoint_lane_index<<std::endl;

      if(next_checkpoint_lane_index == 999)
      {
        node_level++;
        break;
      }
      else
      {
        if((next_checkpoint_lane_index - current_checkpoint_lane_index) < 1)
        {
          node_level_map[node_level].push_back(j);
          skip_list.push_back(j);
        }
        else
        {
          node_level++;
          break;
        }
      }
    }
  }

  for(auto it : node_level_map)
  {
    std::cout<<it.first<<" : ";

    for(auto lane : it.second)
    {
      std::cout<<lane<<", ";
    }
    std::cout<<std::endl;
  }

  std::vector<std::pair<size_t, size_t>> edge_vector;
  std::vector<int> cost_vector;

  for(size_t i = 0 ; i < node_level_map.size()-1; i++)
  {
    for(size_t j = 0; j < node_level_map[i].size(); j++)
    {
      for(size_t k = 0; k < node_level_map[i+1].size(); k++)
      {
        auto start_index = node_level_map[i].at(j);
        auto start_checkpoint = checkpoints_info_msgs_.checkpoints.poses.at(start_index);
        auto start_checkpoint_lane_id = checkpoints_info_msgs_.checkpoints_laneid.at(start_index);
        auto start_checkpoint_lane = findLane(start_checkpoint_lane_id);

        auto end_index = node_level_map[i+1].at(k);
        auto end_checkpoint = checkpoints_info_msgs_.checkpoints.poses.at(end_index);
        auto end_checkpoint_lane_id = checkpoints_info_msgs_.checkpoints_laneid.at(end_index);
        auto end_checkpoint_lane = findLane(end_checkpoint_lane_id);

        int cost = checkpoints_info_msgs_.scores.at(end_index);

        std::cout<<start_index<<"--> "<<end_index<<std::endl;
        if(start_checkpoint_lane == end_checkpoint_lane)
        {
          cost = cost;
        }

        else
        {
          auto lane_diff = abs(static_cast<int>(start_checkpoint_lane - end_checkpoint_lane));
          double dist_cost = 0.0;
          double dist = std::hypot(start_checkpoint.position.x - end_checkpoint.position.x, start_checkpoint.position.y - end_checkpoint.position.y);
          // if(node_level_map[i+1].size() >= 2 && dist < 150.0)
          // {
            // double lane_offset = lane_diff * 3.0;
            double safe_margin = 15;
            // double x = sqrt(pow(lane_offset,2) + pow(dist,2));
            if(dist < (lane_diff * 40.0 + safe_margin * lane_diff))
              dist_cost = 2;
            // std::cout<<"nodt level : "<<i<<" --> "<<i+1<<std::endl;
            // std::cout<<"x : "<<x<<std::endl;
            // std::cout<<"my cost calc : "<<40.0 * lane_diff + safe_margin * lane_diff<<std::endl;

            // if(x < 40.0 * lane_diff + safe_margin * (lane_diff))
            //   dist_cost = 7;
          // }
          
          cost = cost - 1.5 * static_cast<int>(lane_diff) - dist_cost;
        }

        // auto edge = std::make_pair<size_t, size_t>(start_index, end_index);
        edge_vector.emplace_back(start_index, end_index);
        cost_vector.push_back(-cost);
        // add_edge(vertices.at(start_index), vertices.at(end_index), EdgePropertyType(-cost, black_color), g);
      }
    }
  }

  int edge_num = static_cast<int>(edge_vector.size());

  std::pair<size_t, size_t>edge_array[edge_num];
  int weight_array[edge_num];

  for(size_t i = 0 ; i < edge_num; i++)
  {
    edge_array[i] = edge_vector.at(i);
    weight_array[i] = cost_vector.at(i);
  }

  Graph G(edge_array, edge_array + edge_num, static_cast<int>(checkpoints_info_msgs_.checkpoints.poses.size()));

  EdgeIterator ei, ei_end;
  int i = 0;
  for (std::tie(ei, ei_end) = edges(G); ei != ei_end; ++ei, ++i)
        G[*ei].weight = weight_array[i];
      
  printGraphViz(G);
  
  unsigned long N = num_vertices(G);
  std::vector<unsigned long> pred(N);
  std::vector<long> dist(N, std::numeric_limits<int>::max()); // Always with INF
  WeightMap weightMap = get(&EdgeProperties::weight, G);

  bool res = bellmanFord(G, 0, weightMap, pred, dist);

  std::vector<int> label = CHECK_BELLMAN_FORD(G, 0, weightMap, pred, dist);
  std::cout << "[+] Test OK!" << std::endl;
  
  // Print shorthest path in graphviz format
  printGraphShortestPathViz(G, pred);
  // Print bellman ford results
  printGraphShortestPath(G, dist, pred, label);
  
  std::vector<size_t> my_checkpoints;
  
  int shortest = 0;
  std::pair<unsigned long, size_t> shortest_pair;
  size_t target_level = node_level_map.size()-2;
  for(size_t i = 0 ; i < num_vertices(G); i++)
  {
    if(pred[i] != i)
    {
      for(size_t j = 0 ; j < node_level_map[target_level].size(); j++)
      {
        if(node_level_map[target_level].at(j) == pred[i])
        {
          if(dist[i] < shortest)
          {
            shortest = dist[i];
            shortest_pair = std::pair<size_t, size_t>(static_cast<size_t>(pred[i]), i);
          }
        }
      }
    }
  }

  std::cout<<"shortest "<<shortest<<std::endl;
  std::cout<<"shortest pair : "<<shortest_pair.first<<"-->"<<shortest_pair.second<<std::endl;
  
  std::vector<size_t> shortest_path;

  shortest_path.push_back(static_cast<size_t>(shortest_pair.second));
  shortest_path.push_back(static_cast<size_t>(shortest_pair.first));
  
  while(true)
  {
    int shortest = 0;
    size_t index = 0;
    for(size_t i = 0 ; i < num_vertices(G); i++)
    {
      if(static_cast<size_t>(pred[i]) != i)
      {
        if(i == shortest_path.back())
        {
          if(dist[i] < shortest)
          {
            shortest = dist[i];
            index = i;
          }
        }
      }
    }
    shortest_path.push_back(static_cast<size_t>(pred[index]));
    if(shortest_path.back() == static_cast<size_t>(0))
      break;
  }

  std::reverse(shortest_path.begin(), shortest_path.end());

  for(size_t i = 0 ; i <shortest_path.size(); i++)
  {
    my_checkpoints.push_back(shortest_path.at(i));
  }

  for(size_t i = 0 ; i < my_checkpoints.size(); i++)
  {
    std::cout<<my_checkpoints.at(i)<<", ";
  }
  std::cout<<std::endl;
  
  return my_checkpoints;
}

// void LaneKeepingPlannerLanelet2::checkPointInfoCallback(const mission_manager_msgs::msg::CheckpointsWithLaneId::SharedPtr checkpoints_info_msgs)
// {
//   // if(!current_pose_ok_)
//   //   return;

//   checkpoints_.clear();

//   checkpoints_info_msgs_ = *checkpoints_info_msgs;
//   // lanelet::ConstLanelet closest_lanelet;

//   // lanelet::utils::query::getClosestLanelet(road_lanelets_, current_pose_.pose, &closest_lanelet);
//   // lanelet::ConstLanelets neighbor = lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, closest_lanelet);

//   // mission_manager_msgs::msg::NeighborLaneletsInfo neighbor_info;

//   // for (const auto & llt : neighbor)
//   //   neighbor_info.lanelets.push_back(llt.id());

//   // checkpoints_info_msgs_.checkpoints.poses.insert(checkpoints_info_msgs_.checkpoints.poses.begin(), current_pose_.pose);
//   // checkpoints_info_msgs_.neighbor_lanelets.insert(checkpoints_info_msgs_.neighbor_lanelets.begin(), neighbor_info);
//   // checkpoints_info_msgs_.checkpoints_laneid.insert(checkpoints_info_msgs_.checkpoints_laneid.begin(), closest_lanelet.id());
// }

void LaneKeepingPlannerLanelet2::checkPointInfoCallback(const mission_manager_msgs::msg::CheckpointsWithLaneId::SharedPtr checkpoints_info_msgs)
{
  if(!current_pose_ok_)
    return;

  checkpoints_.clear();

  checkpoints_info_msgs_ = *checkpoints_info_msgs;
  std::vector<size_t> shortest_path_index = shortestPath();

  mission_manager_msgs::msg::CheckpointsWithLaneId temp_info;

  for(size_t i = 0 ; i < checkpoints_info_msgs_.checkpoints.poses.size(); i++)
  {
    if(std::find(shortest_path_index.begin(), shortest_path_index.end(), i) != shortest_path_index.end())
    {
      temp_info.checkpoints.poses.push_back(checkpoints_info_msgs_.checkpoints.poses.at(i));
      temp_info.neighbor_lanelets.push_back(checkpoints_info_msgs_.neighbor_lanelets.at(i));
      temp_info.checkpoints_laneid.push_back(checkpoints_info_msgs_.checkpoints_laneid.at(i));
    }
  }

  checkpoints_info_msgs_ = temp_info;
  // checkpoints_info_msgs_.checkpoints.poses.erase(checkpoints_info_msgs_.checkpoints.poses.begin()+6);
  // checkpoints_info_msgs_.neighbor_lanelets.erase(checkpoints_info_msgs_.neighbor_lanelets.begin()+6);
  // checkpoints_info_msgs_.checkpoints_laneid.erase(checkpoints_info_msgs_.checkpoints_laneid.begin()+6);
  lanelet::ConstLanelet closest_lanelet;

  lanelet::utils::query::getClosestLanelet(road_lanelets_, current_pose_.pose, &closest_lanelet);
  lanelet::ConstLanelets neighbor = lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, closest_lanelet);

  mission_manager_msgs::msg::NeighborLaneletsInfo neighbor_info;

  for (const auto & llt : neighbor)
    neighbor_info.lanelets.push_back(llt.id());

  checkpoints_info_msgs_.checkpoints.poses.insert(checkpoints_info_msgs_.checkpoints.poses.begin(), current_pose_.pose);
  checkpoints_info_msgs_.neighbor_lanelets.insert(checkpoints_info_msgs_.neighbor_lanelets.begin(), neighbor_info);
  checkpoints_info_msgs_.checkpoints_laneid.insert(checkpoints_info_msgs_.checkpoints_laneid.begin(), closest_lanelet.id());

  current_checkpoint_index_ = 1;
  auto current_checkpoint_lane_id = checkpoints_info_msgs_.checkpoints_laneid.at(current_checkpoint_index_);
  prev_planning_lane_ = findLane(current_checkpoint_lane_id);

  goal_pose_ = findLaneGoal(current_checkpoint_lane_id);

  autoware_auto_planning_msgs::msg::HADMapRoute route = planRoute(current_pose_.pose, goal_pose_.pose);
  current_route_ = route;
  publishRoute(current_route_);
}

lanelet::ConstLanelets LaneKeepingPlannerLanelet2::getCurrentLanes()
{
  const auto common_parameters = planner_data_->parameters;

  lanelet::ConstLanelet current_lane;
  if (!route_handler_.getClosestLaneletWithinRoute(current_pose_.pose, &current_lane)) {
    RCLCPP_ERROR(get_logger(), "failed to find closest lanelet within route!!!");
    return {};  // TODO(Horibe) what should be returned?
  }

  // For current_lanes with desired length
  return route_handler_.getLaneletSequence(
    current_lane, current_pose_.pose, common_parameters.backward_path_length,
    common_parameters.forward_path_length);
}

lanelet::ConstLanelets LaneKeepingPlannerLanelet2::getLaneChangeLanes(
  const lanelet::ConstLanelets & current_lanes, const double lane_change_lane_length)
{
  lanelet::ConstLanelets lane_change_lanes;

  if (current_lanes.empty()) {
    return lane_change_lanes;
  }

  // Get lane change lanes
  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(
    current_lanes, current_pose_.pose, &current_lane);
  const double lane_change_prepare_length =
    current_twist_.twist.linear.x * (parameters_.lane_change_prepare_duration - 0.3);
  lanelet::ConstLanelets current_check_lanes =
    route_handler_.getLaneletSequence(current_lane, current_pose_.pose, 0.0, lane_change_prepare_length);
  lanelet::ConstLanelet lane_change_lane;
  if (route_handler_.getLaneChangeTarget(current_check_lanes, &lane_change_lane)) {
    lane_change_lanes = route_handler_.getLaneletSequence(
      lane_change_lane, current_pose_.pose, lane_change_lane_length, lane_change_lane_length);
  } else {
    lane_change_lanes.clear();
  }

  return lane_change_lanes;
}

std::pair<bool, bool> LaneKeepingPlannerLanelet2::getSafePath(
  const lanelet::ConstLanelets & lane_change_lanes, const double check_distance,
  LaneChangePath & safe_path)
{
  std::vector<LaneChangePath> valid_paths;
  
  const auto common_parameters = planner_data_->parameters;

  const auto current_lanes = getCurrentLanes();

  if (!lane_change_lanes.empty()) {
    // find candidate paths
    const auto lane_change_paths = behavior_path_planner::lane_change_utils::getLaneChangePaths(
      route_handler_, current_lanes, lane_change_lanes, current_pose_.pose, current_twist_.twist,
      common_parameters, parameters_);

    // get lanes used for detection
    lanelet::ConstLanelets check_lanes;
    if (!lane_change_paths.empty()) {
      const auto & longest_path = lane_change_paths.front();
      // we want to see check_distance [m] behind vehicle so add lane changing length
      const double check_distance_with_path =
        check_distance + longest_path.preparation_length + longest_path.lane_change_length;
      check_lanes = route_handler_.getCheckTargetLanesFromPath(
        longest_path.path, lane_change_lanes, check_distance_with_path);
    }

    // select valid path
    valid_paths = behavior_path_planner::lane_change_utils::selectValidPaths(
      lane_change_paths, current_lanes, check_lanes, *route_handler_.getOverallGraphPtr(),
      current_pose_.pose, route_handler_.isInGoalRouteSection(current_lanes.back()),
      route_handler_.getGoalPose());

    if (valid_paths.empty()) {
      return std::make_pair(false, false);
    }

    // select safe path
    bool found_safe_path = behavior_path_planner::lane_change_utils::selectSafePath(
      valid_paths, current_lanes, check_lanes, planner_data_->dynamic_object, current_pose_.pose,
      current_twist_.twist, common_parameters.vehicle_width, parameters_, &safe_path);
    return std::make_pair(true, found_safe_path);
  }

  return std::make_pair(false, false);
}

bool LaneKeepingPlannerLanelet2::isLaneBlocked(const lanelet::ConstLanelets & lanes)
{
  //taeho
  (void)lanes;
  const auto current_lanes = getCurrentLanes();

  const double lane_changeable_distance_left =
    route_handler_.getLaneChangeableDistance(current_pose_.pose, route_handler::LaneChangeDirection::LEFT);
  const double lane_changeable_distance_right =
    route_handler_.getLaneChangeableDistance(current_pose_.pose, route_handler::LaneChangeDirection::RIGHT);
  
  if(lane_changeable_distance_left == 0 && lane_changeable_distance_right == 0)
    return true;
  else
    return false;
  
}

void LaneKeepingPlannerLanelet2::kinematicStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if(!isRoutingGraphReady())
  {
    RCLCPP_INFO(this->get_logger(), "not ready for Planning");
    return ;
  }

  current_pose_ok_ = true;

  current_pose_.header = msg->header;
  current_pose_.pose.position = msg->pose.pose.position;
  current_pose_.pose.orientation = msg->pose.pose.orientation;

  current_twist_.header = msg->header;
  current_twist_.twist.linear = msg->twist.twist.linear;
  current_twist_.twist.angular = msg->twist.twist.angular;
  
  lanelet::ConstLanelet closest_lanelet;
  lanelet::utils::query::getClosestLanelet(road_lanelets_, current_pose_.pose, &closest_lanelet);

  if(!trajectory_)
    return;


  auto closest_lane_id = closest_lanelet.id();
  auto current_lane = findLane(closest_lane_id);

  auto current_checkpoint_pose = checkpoints_info_msgs_.checkpoints.poses.at(std::min(current_checkpoint_index_, checkpoints_info_msgs_.checkpoints.poses.size()-1));
  
  if(current_lane == prev_planning_lane_)
    on_lane_count++;
  else
    on_lane_count = 0;

  auto next_checkpoint_pose2 = checkpoints_info_msgs_.checkpoints.poses.at(std::min(current_checkpoint_index_+1, checkpoints_info_msgs_.checkpoints.poses.size()-1));

  visualization_msgs::msg::MarkerArray MA;
  geometry_msgs::msg::PoseArray PA;
  PA.poses.push_back(current_checkpoint_pose);
  PA.poses.push_back(next_checkpoint_pose2); 
  int id_ = 0;
  for(size_t i = 0; i < PA.poses.size(); ++i)
  {
      visualization_msgs::msg::Marker M;
      M.header.frame_id = "map";
      M.header.stamp = this->now();
      M.ns = "checkpoint";
      M.id = id_++;
      M.type = visualization_msgs::msg::Marker::MODIFY;
      M.action = visualization_msgs::msg::Marker::ADD;
      M.pose = PA.poses[i];
      M.scale.x = 3.0;
      M.scale.y = 3.0;
      M.scale.z = 1.0;
      M.color.a = 1.0;
      if(i==0)
      {
      M.color.r = 0.0;
      M.color.g = 1.0;
      M.color.b = 1.0;
      }
      else
      {
      M.color.r = 1.0;
      M.color.g = 1.0;
      M.color.b = 1.0;
      }
      MA.markers.push_back(M);
  }
  marker_pub_->publish(MA);  

  if(needNextCheckpointPlanning(trajectory_, current_checkpoint_pose) && !pub_end_goal_ && (on_lane_count > 5))
  {
    auto next_checkpoint_index = std::min(current_checkpoint_index_+1, checkpoints_info_msgs_.checkpoints.poses.size()-1);
    auto next_checkpoint_lane_id = checkpoints_info_msgs_.checkpoints_laneid.at(next_checkpoint_index);
    auto next_checkpoint_pose = checkpoints_info_msgs_.checkpoints.poses.at(next_checkpoint_index);
    
    auto lane = findLane(next_checkpoint_lane_id);

    bool need_replan = false;

    if(lane - current_lane == 0)
    {
      auto checkpoint_idx = tier4_autoware_utils::findNearestIndex(trajectory_->points, next_checkpoint_pose, 0.5);
      if(checkpoint_idx)
      {
        current_checkpoint_index_ ++;
      }
    }

    else  
    {
      goal_pose_ = findLaneGoal(next_checkpoint_lane_id);

      autoware_auto_planning_msgs::msg::HADMapRoute route = planRoute(current_pose_.pose, goal_pose_.pose);
      route_handler_.setRoute(route);
      
      auto lane_diff = std::max(current_lane, lane) - std::min(current_lane, lane);

      lanelet::ConstLanelet second_lane;
      
      auto res = makeLaneChangePath(current_checkpoint_pose, next_checkpoint_pose, lane_diff, true, second_lane);
      
      if(res == LaneChangeResult::SUCCESS)
      {
        need_replan = true;
        if(last_result_ != res)
          std::cout<<"SUCCESS"<<std::endl;
      }

      else if(res == LaneChangeResult::NOT_YET)
      {
        need_replan = false;
        if(last_result_ != res)
          std::cout<<"NOT YET"<<std::endl;
      }

      else if(res == LaneChangeResult::BLOCKED)
      {
        need_replan = false;
        if(last_result_ != res)
          std::cout<<"BLOCKED"<<std::endl;
      }

      else if(res == LaneChangeResult::BLOCKED_FAIL)
      {
        need_replan = false;
        current_checkpoint_index_ ++;
        if(last_result_ != res)
          std::cout<<"BLOCKED FAIL"<<std::endl;
      }

      else if(res == LaneChangeResult::CHANGE_PARAM)
      {
        need_replan = false;
        current_checkpoint_index_++;
        if(last_result_ != res)
        std::cout<<"CHANGE PARAM"<<std::endl;
      }

      else if(res == LaneChangeResult::NORMAL_FAIL)
      {
        current_checkpoint_index_ ++;
        need_replan = false;
        if(last_result_ != res)
        std::cout<<"FAIL"<<std::endl;
      }

      last_result_ = res;

      if(need_replan)
      {
        current_route_ = route;
        publishRoute(current_route_);
        prev_planning_lane_ = lane;
        on_lane_count=0;
        current_checkpoint_index_++;
      }
    }
  }
}

LaneChangeResult LaneKeepingPlannerLanelet2::makeLaneChangePath(geometry_msgs::msg::Pose cur_checkpoint, geometry_msgs::msg::Pose next_checkpoint, size_t lane_diff, bool speed_up_lane, lanelet::ConstLanelet& second_lanelet)
{
  (void)second_lanelet;
  (void)speed_up_lane;
  constexpr double RESAMPLE_INTERVAL = 1.0;
  double lane_change_lane_length = 200;

  const double check_distance = 100.0;

  const auto current_lanes = getCurrentLanes();
  const auto lane_change_lanes = getLaneChangeLanes(current_lanes, lane_change_lane_length);

  // Find lane change path
  bool found_valid_path, found_safe_path;
  LaneChangePath selected_path;
  std::tie(found_valid_path, found_safe_path) =
    getSafePath(lane_change_lanes, check_distance, selected_path);

  auto path = behavior_path_planner::util::resamplePathWithSpline(selected_path.path, RESAMPLE_INTERVAL);
  path.header = route_handler_.getRouteHeader();
  debug_lane_change_path_publisher_->publish(path);
  if(found_valid_path && path.points.size() > 0)
  {
    auto cur_closest_index = tier4_autoware_utils::findNearestIndex(path.points, cur_checkpoint, 0.49);
    auto next_closest_index = tier4_autoware_utils::findNearestIndex(path.points, next_checkpoint, 1.0);

    size_t cur_closest_index2 = tier4_autoware_utils::findNearestIndex(path.points, cur_checkpoint.position);
    size_t next_closest_index2 = tier4_autoware_utils::findNearestIndex(path.points, next_checkpoint.position);

    // std::cout<<cur_closest_index2<<", "<<next_closest_index2<<std::endl;
    // std::cout<<cur_closest_index<<", "<<next_closest_index<<std::endl;
    if(lane_diff == 1)
    {
      if(isLaneBlocked(lane_change_lanes))
      {
        if(tier4_autoware_utils::findNearestIndex(trajectory_->points, next_checkpoint.position) == 0)
          return LaneChangeResult::BLOCKED_FAIL;
        return LaneChangeResult::BLOCKED;
      }

      else if(cur_closest_index != boost::none && next_closest_index != boost::none)
      {
        return SUCCESS;
      }

      else if(cur_closest_index != boost::none && next_closest_index == boost::none)
      {
        if(next_closest_index2 == path.points.size()-1) //to far next point
        {
          return LaneChangeResult::SUCCESS;
        }
        else
        {
          size_t vehicle_closest_index = tier4_autoware_utils::findNearestIndex(path.points, current_pose_.pose.position);
          size_t vehicle_predict_closest_index = vehicle_closest_index + current_twist_.twist.linear.x * 0.85 * parameters_.lane_change_prepare_duration;

          if(vehicle_predict_closest_index >= cur_closest_index2)
            std::cout<<"need prepare distance down"<<std::endl;
          else
            std::cout<<"need change duration down "<<std::endl;
          return LaneChangeResult::CHANGE_PARAM;
        }
      }

      else if(cur_closest_index == boost::none && next_closest_index != boost::none)
      {
        if(cur_closest_index2 == 0)
        {
          return LaneChangeResult::SUCCESS;
        }

        else
        {
          std::cout<<"can't pass cur not find , find not yet"<<std::endl;
          return LaneChangeResult::NOT_YET;
        }
      }

      else
      {
        if(cur_closest_index2 == 0 && next_closest_index2 == path.points.size()-1)
          return LaneChangeResult::SUCCESS;
        
        if(cur_closest_index2 != 0 && next_closest_index2 == path.points.size()-1)
          return LaneChangeResult::NOT_YET;
        
        if(cur_closest_index2 != 0 && next_closest_index2 != path.points.size()-1)
        {

          return LaneChangeResult::NOT_YET;
        }
        
        if(cur_closest_index2 == 0 && next_closest_index != path.points.size() -1)
        {
          std::cout<<"cur passed change param can't arrvie at next"<<std::endl;
          return LaneChangeResult::CHANGE_PARAM;
        }

        return LaneChangeResult::NOT_YET;
      }
    }

    else if(lane_diff == 2)
    {
      // double dist = std::hypot(cur_checkpoint.position.x - next_checkpoint.position.x, cur_checkpoint.position.y - next_checkpoint.position.y);
      size_t second_lanelet_closest_index = tier4_autoware_utils::findNearestIndex(path.points, next_checkpoint.position);
      if(cur_closest_index)
      {
        if(second_lanelet_closest_index > (cur_closest_index2 + static_cast<size_t>(100)))
        {
          return LaneChangeResult::SUCCESS;
        }
        else
        {
          return LaneChangeResult::NORMAL_FAIL;
        }
      }

      else if(!cur_closest_index && cur_closest_index2 == 0)
      {
        size_t current_pose_closest_index = tier4_autoware_utils::findNearestIndex(path.points, current_pose_.pose.position);

        if(second_lanelet_closest_index > (current_pose_closest_index + static_cast<size_t>(100)))
        {
          return LaneChangeResult::SUCCESS;
        }
        else
        {
          return LaneChangeResult::NORMAL_FAIL;
        }
      }

      else if(!cur_closest_index && cur_closest_index2 != 0)
      {
        return LaneChangeResult::NOT_YET;
      }

      else if(!cur_closest_index)
      {
        size_t current_pose_closest_index = tier4_autoware_utils::findNearestIndex(path.points, current_pose_.pose.position);

        if(second_lanelet_closest_index > (current_pose_closest_index + static_cast<size_t>(100)))
        {
          return LaneChangeResult::SUCCESS;
        }
      }
    }
  }

  double dis = std::hypot(current_pose_.pose.position.x - next_checkpoint.position.x, current_pose_.pose.position.y - next_checkpoint.position.y);

  if(tier4_autoware_utils::findNearestIndex(trajectory_->points, next_checkpoint.position) == 0 && dis < 16.0)  
  {
    std::cout<<"may be blocked fail"<<std::endl;
    return LaneChangeResult::BLOCKED_FAIL;
  }
  return LaneChangeResult::BLOCKED;
}

}  // namespace mission_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(route_publisher_pm::LaneKeepingPlannerLanelet2)
