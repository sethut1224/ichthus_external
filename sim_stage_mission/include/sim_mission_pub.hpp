/* ===========================================================================
Copyright 2022. The ICHTHUS Project. All Rights Reserved.
Eunseo Choi (eunseo.choi.d@gmail.com) and Kanghee Kim (kim.kanghee@gmail.com).
Mobility Intelligence & Computing Systems Laboratory, Soongsil University.
=========================================================================== */

#ifndef __SIM_MISSION_PUB_HPP_
#define __SIM_MISSION_PUB_HPP_

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
#include <std_msgs/msg/string.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

namespace sim_stage_mission
{
    class SimMissionPub : public rclcpp::Node
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

        SimMissionPub();
        ~SimMissionPub();

        void run();

        void readAMScenarioCsv(const std::string &path, kiapi_msgs::msg::MissionListStage1 &output);
        void readPMScenarioCsv(const std::string &path, kiapi_msgs::msg::MissionListStage2 &output);

    private:
        std::string mission_type_;
        std::string save_dir_;

        rclcpp::Publisher<kiapi_msgs::msg::MissionListStage1>::SharedPtr pub_am_mission_;
        rclcpp::Publisher<kiapi_msgs::msg::MissionListStage2>::SharedPtr pub_pm_mission_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_checkpoint_debug_;

        kiapi_msgs::msg::MissionListStage1 mission_list_am_;
        kiapi_msgs::msg::MissionListStage2 mission_list_pm_;

        visualization_msgs::msg::MarkerArray ma_;

        inline std::vector<std::string>
        split(const std::string &input, char delimiter)
        {
            std::istringstream stream(input);
            std::string field;
            std::vector<std::string> result;
            while (std::getline(stream, field, delimiter))
            {
                result.push_back(field);
            }
            return result;
        }

        double intToDouble(const int num, const size_t int_digits);
        void GPSToPMap(const double pos_lat, const double pos_lon, double &point_map_x, double &point_map_y);

        void visualizeCheckPointAM(geometry_msgs::msg::PoseArray pa);
        void visualizeCheckPointPM(geometry_msgs::msg::PoseArray pa, std::vector<int> &score_list);
    };
}

#endif // __SIM_MISSION_PUB_HPP_