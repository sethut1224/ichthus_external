cmake_minimum_required(VERSION 3.14)
project(lane_keeping_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(lane_keeping_planner_node SHARED
  lib/lane_keeping_planner_base.cpp
  src/lane_keeping_planner_lanelet2/lane_keeping_planner_lanelet2.cpp
  src/lane_keeping_planner_lanelet2/utility_functions.cpp
)

rclcpp_components_register_node(lane_keeping_planner_node
  PLUGIN "lane_keeping_planner::LaneKeepingPlannerLanelet2"
  EXECUTABLE lane_keeping_planner
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
