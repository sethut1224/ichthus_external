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

uint32_t findClosestIndexInLanelet(lanelet::ConstLanelet& lanelet, geometry_msgs::msg::Point position)
{ 
  auto centerline = lanelet.centerline();
  double min_dist = std::numeric_limits<double>::max();
  uint32_t closest_index = 0;
  for (uint32_t i = 0; i < centerline.size(); i++)
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

bool isLaneletMatchedTargetArcLength(lanelet::ConstLanelet& lanelet, uint32_t s_idx, double target_length, double* arc_length, uint32_t* t_idx)
{ 
  auto centerline = lanelet.centerline();
  bool find_target = false;
  for(size_t i = s_idx; i < centerline.size(); i++)
  {
    const lanelet::ConstPoint3d pt = centerline[i];
    lanelet::ConstPoint3d next_pt = (i + 1 < centerline.size()) ? centerline[i+1] : centerline[i];
    *arc_length += lanelet::geometry::distance2d(lanelet::utils::to2D(pt), lanelet::utils::to2D(next_pt));
    if(*arc_length >= target_length)
    {
      find_target = true;
      *t_idx = i;
      break;
    }
  }
  return find_target;
}

double calcArcLengthStartToGoal(lanelet::LaneletMapPtr lanelet_map_ptr, std::vector<uint32_t> target_lane_ids, lanelet::ConstLanelet current_llt, uint32_t current_pose_index, uint32_t goal_pose_index, double thres)
{
  uint32_t current_lane_id_index = 0;

  for(size_t i = 0; i < target_lane_ids.size(); i++)
  {
    if(target_lane_ids.at(i) == current_llt.id())
    {
      current_lane_id_index = i;
      break;
    }
  }

  double arc_length = 0.0;

  for(size_t i = current_lane_id_index; i < target_lane_ids.size(); i++)
  {
    auto lane_id = target_lane_ids[i];
    auto current_lane_id = current_llt.id();
    auto goal_lane_id = target_lane_ids[target_lane_ids.size()-1];
    auto lane = lanelet_map_ptr->laneletLayer.get(lane_id);
    auto centerline = lane.centerline();

    size_t start_index = lane_id == current_lane_id ? current_pose_index : 0;
    size_t end_index = lane_id == goal_lane_id ? goal_pose_index : centerline.size();

    for(size_t j = start_index; j < end_index; j++)
    {
      const lanelet::ConstPoint3d pt = centerline[i];
      lanelet::ConstPoint3d next_pt = (i + 1 < centerline.size()) ? centerline[i+1] : centerline[i];
      arc_length +=  lanelet::geometry::distance2d(lanelet::utils::to2D(pt), lanelet::utils::to2D(next_pt));
      if(arc_length > thres)
      {
        break;
      }
    }
  }

  return arc_length;
}

geometry_msgs::msg::Quaternion QuaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
  
}

nav_msgs::msg::Path visualizeLanelet(lanelet::ConstLanelet& lanelet, rclcpp::Time t)
{
  nav_msgs::msg::Path msg;
  auto centerline = lanelet.centerline();
  
  for(size_t i = 0; i < centerline.size(); i++)
  {
    auto geompt = lanelet::utils::conversion::toGeomMsgPt(centerline[i]);
    geometry_msgs::msg::PoseStamped pos;
    pos.pose.position = geompt;
    msg.poses.push_back(pos);
  }

  msg.header.frame_id = "map";
  msg.header.stamp = t;

  return msg;
}

}  // anonymous namespace

namespace lane_keeping_planner
{
LaneKeepingPlannerLanelet2::LaneKeepingPlannerLanelet2(const rclcpp::NodeOptions & node_options)
: LaneKeepingPlanner("lane_keeping_planner_node", node_options), is_graph_ready_(false), left_lane_change(false), right_lane_change(false), prev_indicator(TurnIndicator::DISABLE), current_indicator(TurnIndicator::DISABLE)
{
  using std::placeholders::_1;
  map_subscriber_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "input/vector_map", rclcpp::QoS{10}.transient_local(),
    std::bind(&LaneKeepingPlannerLanelet2::mapCallback, this, _1));

  is_lane_keeping_ready_ = false;

  lane_keeping_start_subscriber = create_subscription<std_msgs::msg::Bool>(
    "/lane_keeping_start", 10, std::bind(&LaneKeepingPlannerLanelet2::startCallback, this, _1)
  );

  turn_indicators_subscriber = create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>("/vehicle/status/turn_indicators_status", 10, std::bind(&LaneKeepingPlannerLanelet2::turnIndicatorCallback, this, _1));

  kinematic_state_subscriber = create_subscription<nav_msgs::msg::Odometry>("/localization/kinematic_state", 1, std::bind(&LaneKeepingPlannerLanelet2::kinematicStateCallback, this, _1)
  );

  debug_goal_pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("/debug/goal_pose_visualizer",10);
  debug_goal_pose_publisher2 = create_publisher<geometry_msgs::msg::PoseStamped>("/debug/goal_pose_visualizer2",10);

  debug_lanelet_visualize = create_publisher<nav_msgs::msg::Path>("/debug/lanelet_visualize",10);

  for(int i = 0 ; i < 10; i++)
  {
    indicator_deque.push_back(TurnIndicator::DISABLE);
  }

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
bool LaneKeepingPlannerLanelet2::isLaneKeepingReady() const {return is_lane_keeping_ready_; }
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

// void LaneKeepingPlannerLanelet2::turnIndicatorCallback(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::SharedPtr msg)
// {

//   auto indicator = static_cast<TurnIndicator>(msg->report);
//   indicator_deque.pop_front();
//   indicator_deque.push_back(indicator);
  
//   auto first_ind = indicator_deque.front();

//   if(indicator != TurnIndicator::DISABLE)
//   {
//     for(auto ind : indicator_deque)
//     {
//       if(ind != first_ind)
//       {
//         std::cout<<"not same indi : "<<first_ind<<", "<<ind<<std::endl;
//         return;
//       }
//     }
//   }

//   if(prev_indicator != indicator)
//   {
//     current_indicator = indicator;

//     if(current_indicator == TurnIndicator::LEFT)
//     {
//       left_lane_change = true;
//       right_lane_change = !left_lane_change;
//     }

//     else if(current_indicator == TurnIndicator::RIGHT)
//     {
//       right_lane_change = true;
//       left_lane_change = !right_lane_change;
//     }

//     else
//     { 
//       right_lane_change = false;
//       left_lane_change = false;
//     }
//   }
//   prev_indicator = indicator;
// }

void LaneKeepingPlannerLanelet2::turnIndicatorCallback(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::SharedPtr msg)
{

  auto indicator = static_cast<TurnIndicator>(msg->report);

  if(indicator == TurnIndicator::LEFT || indicator == TurnIndicator::RIGHT)
  {
    if(prev_indicator != TurnIndicator::DISABLE)
    {
      return;
    }
    else
    {
      current_indicator = indicator;
    }
  } // left or right indicator is accepted when previous indicator is DISABLE

  else
  {
    current_indicator = indicator;
  }

  if(current_indicator == TurnIndicator::LEFT)
  {
    left_lane_change = true;
    right_lane_change = !left_lane_change;
  }

  else if(current_indicator == TurnIndicator::RIGHT)
  {
    right_lane_change = true;
    left_lane_change = !right_lane_change;
  }

  else
  {
    left_lane_change = false;
    right_lane_change = false;
  }

  prev_indicator = current_indicator;


  // indicator_deque.pop_front();
  // indicator_deque.push_back(indicator);
  
  // auto first_ind = indicator_deque.front();

  // if(indicator != TurnIndicator::DISABLE)
  // {
  //   for(auto ind : indicator_deque)
  //   {
  //     if(ind != first_ind)
  //     {
  //       std::cout<<"not same indi : "<<first_ind<<", "<<ind<<std::endl;
  //       return;
  //     }
  //   }
  // }

  // if(prev_indicator != indicator)
  // {
  //   current_indicator = indicator;

  //   if(current_indicator == TurnIndicator::LEFT)
  //   {
  //     left_lane_change = true;
  //     right_lane_change = !left_lane_change;
  //   }

  //   else if(current_indicator == TurnIndicator::RIGHT)
  //   {
  //     right_lane_change = true;
  //     left_lane_change = !right_lane_change;
  //   }

  //   else
  //   { 
  //     right_lane_change = false;
  //     left_lane_change = false;
  //   }
  // }
  // prev_indicator = indicator;
}

void LaneKeepingPlannerLanelet2::startCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  is_lane_keeping_ready_ = msg->data;
}

void LaneKeepingPlannerLanelet2::kinematicStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if(!isRoutingGraphReady() || !isLaneKeepingReady())
  {
    RCLCPP_INFO(this->get_logger(), "not ready for LaneKeeping");
    return ;
  }

  geometry_msgs::msg::PoseStamped current_pose;

  current_pose.header = msg->header;
  current_pose.pose.position = msg->pose.pose.position;
  current_pose.pose.orientation = msg->pose.pose.orientation;
  
  if(!lanelet::utils::query::getClosestLanelet(road_lanelets_, current_pose.pose, &closest_lanelet))
  {
    RCLCPP_INFO(this->get_logger(), "can not find nearest lanelet");
    return;
  }

  double arc_length = 0.0;
  double target_length = 300.0;

  lanelet::ConstLanelet goal_lanelet;

  if(target_lane_ids.size() == 0) //first planning
  {
    goal_lanelet = closest_lanelet;
    auto current_pose_index = findClosestIndexInLanelet(closest_lanelet, current_pose.pose.position);
    uint32_t target_index = 0;
    
    while(arc_length <= target_length)
    {
      uint32_t start_index = goal_lanelet.id() == closest_lanelet.id() ? current_pose_index : 0;
      isLaneletMatchedTargetArcLength(goal_lanelet, start_index, target_length, &arc_length, &target_index);
      if (arc_length > target_length)
        break;
      goal_lanelet = routing_graph_ptr_->following(goal_lanelet)[0];
    }

    auto goal_lanelet_pt = goal_lanelet.centerline()[target_index];
    lane_keeping_goal_pose.pose.position = lanelet::utils::conversion::toGeomMsgPt(goal_lanelet_pt);
  }

  else
  {
    auto current_lane_id = closest_lanelet.id();
    if(std::find(target_lane_ids.begin(), target_lane_ids.end(), current_lane_id) == target_lane_ids.end())
    {
      RCLCPP_INFO(this->get_logger(), "current lane is not a target lane");
      return;
    }

    auto current_pose_index = findClosestIndexInLanelet(closest_lanelet, current_pose.pose.position);
    boost::optional<lanelet::ConstLanelet> adjacent_lanelet_ptr;

    if(right_lane_change)
    {
      adjacent_lanelet_ptr = routing_graph_ptr_->right(closest_lanelet);      
    }

    if(left_lane_change)
    {
      adjacent_lanelet_ptr = routing_graph_ptr_->left(closest_lanelet);
    }

    if(adjacent_lanelet_ptr)
    {
      auto adjacent_lanelet = adjacent_lanelet_ptr.get();
      goal_lanelet = adjacent_lanelet;
      auto adjacent_closest_index = findClosestIndexInLanelet(goal_lanelet, current_pose.pose.position);
      uint32_t target_index = 0;

      while(arc_length <= target_length)
      {
        uint32_t start_index = goal_lanelet.id() == adjacent_lanelet.id() ? adjacent_closest_index : 0;
        isLaneletMatchedTargetArcLength(goal_lanelet, start_index, target_length, &arc_length, &target_index);
        if (arc_length > target_length)
          break;
        goal_lanelet = routing_graph_ptr_->following(goal_lanelet)[0];
      }

      auto msg = visualizeLanelet(goal_lanelet, this->now()); 
      auto goal_lanelet_pt = goal_lanelet.centerline()[target_index];

      lane_keeping_goal_pose.pose.position = lanelet::utils::conversion::toGeomMsgPt(goal_lanelet_pt);
      // debug_lanelet_visualize->publish(msg);
      // geometry_msgs::msg::PoseStamped temp_goal_pose;
      // temp_goal_pose.header.stamp = this->now();
      // temp_goal_pose.header.frame_id = "map";
      // debug_goal_pose_publisher2->publish(temp_goal_pose);
      // debug_goal_pose_publisher->publish(lane_keeping_goal_pose);

      if(left_lane_change) 
        left_lane_change = !left_lane_change;
      if(right_lane_change) 
        right_lane_change = !right_lane_change;
    }
    else
    {
      auto lane_keeping_goal_lanelet = lanelet_map_ptr_->laneletLayer.get(target_lane_ids.back());
      auto goal_pose_index = findClosestIndexInLanelet(lane_keeping_goal_lanelet, lane_keeping_goal_pose.pose.position);
      
      if (calcArcLengthStartToGoal(lanelet_map_ptr_, target_lane_ids, closest_lanelet, current_pose_index, goal_pose_index, target_length / 2.0) > target_length / 2.0)
      {
        return;
      }

      goal_lanelet = closest_lanelet;
      uint32_t target_index = 0;

      while(arc_length <= target_length)
      {
        uint32_t start_index = goal_lanelet.id() == closest_lanelet.id() ? current_pose_index : 0;
        isLaneletMatchedTargetArcLength(goal_lanelet, start_index, target_length, &arc_length, &target_index);
        if (arc_length > target_length)
          break;
        goal_lanelet = routing_graph_ptr_->following(goal_lanelet)[0];
      }
    
      auto goal_lanelet_pt = goal_lanelet.centerline()[target_index];
      lane_keeping_goal_pose.pose.position = lanelet::utils::conversion::toGeomMsgPt(goal_lanelet_pt);
    }
  }

  double lane_yaw = lanelet::utils::getLaneletAngle(goal_lanelet, lane_keeping_goal_pose.pose.position);
  lane_keeping_goal_pose.pose.orientation = QuaternionFromYaw(lane_yaw);
  
  lane_keeping_goal_pose.header.stamp = this->now();
  lane_keeping_goal_pose.header.frame_id = "map";
  debug_goal_pose_publisher->publish(lane_keeping_goal_pose);

  autoware_auto_planning_msgs::msg::HADMapRoute route = planRoute(current_pose.pose, lane_keeping_goal_pose.pose);
  
  target_lane_ids.clear();

  for(size_t i = 0; i < route.segments.size(); i++)
  {
    auto primitives = route.segments[i];
    auto preferred_id = primitives.preferred_primitive_id;
    target_lane_ids.push_back(preferred_id);
  }

  usleep(100000);
  publishRoute(route);
}

}  // namespace mission_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lane_keeping_planner::LaneKeepingPlannerLanelet2)
