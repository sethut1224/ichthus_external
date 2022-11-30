// Copyright 2022. The ICHTHUS Project. All Rights Reserved.
// Taeho Han (sethut1224@gmail.com) and
// Kanghee Kim (kim.kanghee@gmail.com) all rights reserved
// MISYS Laboratory, Soongsil University.
// added by ICHTHUS, Taeho Hanon 20221026

#ifndef MISSION_PLANNER__LANELET2_IMPL__UTILITY_FUNCTIONS_HPP_
#define MISSION_PLANNER__LANELET2_IMPL__UTILITY_FUNCTIONS_HPP_
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LaneletSequence.h>

#include <string>
#include <unordered_set>
#include <vector>
bool exists(const std::unordered_set<lanelet::Id> & set, const lanelet::Id & id);

template <typename T>
bool exists(const std::vector<T> & vectors, const T & item)
{
  for (const auto & i : vectors) {
    if (i == item) {
      return true;
    }
  }
  return false;
}

void setColor(std_msgs::msg::ColorRGBA * cl, double r, double g, double b, double a);
void insertMarkerArray(
  visualization_msgs::msg::MarkerArray * a1, const visualization_msgs::msg::MarkerArray & a2);
std::string toString(const geometry_msgs::msg::Pose & pose);
#endif  // MISSION_PLANNER__LANELET2_IMPL__UTILITY_FUNCTIONS_HPP_
