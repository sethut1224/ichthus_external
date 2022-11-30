cmake_minimum_required(VERSION 3.14)
project(route_publisher_pm2)

find_package(autoware_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
autoware_package()

ament_auto_add_library(route_publisher_pm2_node SHARED
  lib/lane_keeping_planner_base.cpp
  src/lane_keeping_planner_lanelet2/lane_keeping_planner_lanelet2.cpp
  src/lane_keeping_planner_lanelet2/utility_functions.cpp
  src/bellman_ford.cpp
  src/graph_creator.cpp
  src/graph_printer.cpp
)

rclcpp_components_register_node(route_publisher_pm2_node
  PLUGIN "route_publisher_pm::LaneKeepingPlannerLanelet2"
  EXECUTABLE route_publisher_pm2
)

target_link_libraries(route_publisher_pm2_node
  ${OpenCV_LIBRARIES}
)


ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
