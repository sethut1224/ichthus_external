/* ===========================================================================
Copyright 2022. The ICHTHUS Project. All Rights Reserved.
Eunseo Choi (eunseo.choi.d@gmail.com) and Kanghee Kim (kim.kanghee@gmail.com).
Mobility Intelligence & Computing Systems Laboratory, Soongsil University.
=========================================================================== */

#ifndef __SIM_MISSION_GEN_HPP_
#define __SIM_MISSION_GEN_HPP_

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <random>

#include <rclcpp/rclcpp.hpp>

#include <kiapi_msgs/msg/mission_list_stage1.hpp>
#include <kiapi_msgs/msg/mission_route_data.hpp>
#include <kiapi_msgs/msg/mission_list_stage2.hpp>
#include <kiapi_msgs/msg/item_data.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

namespace sim_stage_mission
{
    class SimMissionGen : public rclcpp::Node
    {
    public:
        enum NodeType
        {
            ROUTE,
            START,
            END,
        };
        enum ItemType
        {
            PLUS,
            MINUS,
            BOOST,
        };

        SimMissionGen();
        ~SimMissionGen();

        void run();

    private:
        std::string save_dir_;
        std::string mission_type_;
        std::vector<int> pm_score_list_;

        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pose_list_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_clicked_point_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_item_index_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_save_mission_list_;

        geometry_msgs::msg::PoseArray pose_list_; // RViz

        std::vector<kiapi_msgs::msg::MissionRouteData> route_node_list_;
        std::vector<kiapi_msgs::msg::ItemData> item_data_list_;
        std::vector<int> item_index_list_;

        void callbackClickedPoint(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
        void callbackSaveMissionList(const std_msgs::msg::String::ConstSharedPtr msg);
        void callbackItemIndex(const std_msgs::msg::Int32::ConstSharedPtr msg);

        int doubleToInt(const double num);
    };
}

#endif // __SIM_MISSION_GEN_HPP_