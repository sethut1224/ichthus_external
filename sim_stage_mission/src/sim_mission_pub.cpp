/* ===========================================================================
Copyright 2022. The ICHTHUS Project. All Rights Reserved.
Eunseo Choi (eunseo.choi.d@gmail.com) and Kanghee Kim (kim.kanghee@gmail.com).
Mobility Intelligence & Computing Systems Laboratory, Soongsil University.
[Licensed under the Apache License, Version 2.0]
=========================================================================== */

#include <sim_mission_pub.hpp>

namespace sim_stage_mission
{
    SimMissionPub::SimMissionPub() : Node("sim_mission_pub")
    {
        mission_type_ = this->declare_parameter("mission_type", "AM");
        std::string mission_file = this->declare_parameter("mission_file", "/root/shared_dir/eunseo/mission.csv");

        rclcpp::QoS qos{1};
        // qos.transient_local();
        pub_am_mission_ = create_publisher<kiapi_msgs::msg::MissionListStage1>("/v2x/stage1_mission_list", qos);
        pub_pm_mission_ = create_publisher<kiapi_msgs::msg::MissionListStage2>("/v2x/stage2_mission_list", qos);
        pub_checkpoint_debug_ = create_publisher<visualization_msgs::msg::MarkerArray>("/checkpoint_debug_markers", qos);

        if (mission_type_ == "AM")
        {
            std::cout << "##########################" << std::endl;
            std::cout << "### sim_mission_pub AM ###" << std::endl;
            std::cout << "##########################" << std::endl;
            std::cout << "mission_file: " << mission_file << std::endl;

            readAMScenarioCsv(mission_file, mission_list_am_);
        }
        else if (mission_type_ == "PM")
        {
            std::cout << "##########################" << std::endl;
            std::cout << "### sim_mission_pub PM ###" << std::endl;
            std::cout << "##########################" << std::endl;
            std::cout << "mission_file: " << mission_file << std::endl;

            readPMScenarioCsv(mission_file, mission_list_pm_);
        }
        else
        {
            /* do nothing */
        }
    }

    void SimMissionPub::run()
    {
        if (mission_type_ == "AM")
        {
            mission_list_am_.stamp = this->get_clock()->now();
            mission_list_am_.mission_status = 4;
            pub_am_mission_->publish(mission_list_am_);
            pub_checkpoint_debug_->publish(ma_);
        }
        else if (mission_type_ == "PM")
        {
            mission_list_pm_.stamp = this->get_clock()->now();
            mission_list_pm_.mission_status = 4;
            pub_pm_mission_->publish(mission_list_pm_);
            pub_checkpoint_debug_->publish(ma_);
        }
        else
        {
            /* do nothing */
        }
    }

    SimMissionPub::~SimMissionPub()
    {
    }

    double SimMissionPub::intToDouble(const int num, const size_t int_digits)
    {
        std::string str = std::to_string(num);
        size_t cnt = str.size();

        double result = static_cast<double>(num) * std::pow(0.1, (cnt - int_digits));
        return result;
    }

    void SimMissionPub::GPSToPMap(const double pos_lat, const double pos_lon, double &point_map_x, double &point_map_y)
    {
        /* GPS to UTM */
        int zone = 52;
        bool northp = true;
        lanelet::BasicPoint3d utm_point{0.0, 0.0, 0.0};

        GeographicLib::UTMUPS::Forward(pos_lat, pos_lon, zone, northp, utm_point.x(), utm_point.y());

        // /* UTM to PointMap */
        point_map_x = utm_point.x() - 445815.539508;
        point_map_y = utm_point.y() - 3944953.128090;
    }

    void SimMissionPub::visualizeCheckPointAM(geometry_msgs::msg::PoseArray pa)
    {
        int id_ = 0;

        for (size_t i = 0; i < pa.poses.size(); ++i)
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = this->now();
            m.ns = "checkpoint";
            m.id = id_++;
            m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose = pa.poses[i];
            m.scale.x = 5.0;
            m.scale.y = 5.0;
            m.scale.z = 5.0;
            m.color.a = 1.0;
            m.color.r = 1.0;
            m.color.g = 1.0;
            m.color.b = 0.0;

            m.text = std::to_string(id_);

            ma_.markers.push_back(m);
        }
    }

    void SimMissionPub::visualizeCheckPointPM(geometry_msgs::msg::PoseArray pa, std::vector<int> &score_list)
    {
        if (pa.poses.size() != score_list.size())
        {
            std::cout << "[WARN] Cannot visualize checkpoint: \n";
            std::cout << "visualizeCheckPointPM => pa.poses.size() != score_list.size()\n";
            return;
        }

        int id_ = 0;
        int score_ = 0;

        for (size_t i = 0; i < pa.poses.size(); ++i)
        {
            score_ = score_list[i];

            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = this->now();
            m.ns = "checkpoint";
            m.id = id_++;
            m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose = pa.poses[i];
            m.scale.x = 5.0;
            m.scale.y = 5.0;
            m.scale.z = 5.0;
            m.color.a = 1.0;
            m.color.r = 1.0;
            m.color.g = 1.0;
            m.color.b = 0.0;

            m.text = std::to_string(id_) + " (" + std::to_string(score_) + ")";

            ma_.markers.push_back(m);
        }
    }

    void SimMissionPub::readAMScenarioCsv(const std::string &path, kiapi_msgs::msg::MissionListStage1 &output)
    {
        std::ifstream is(path);
        std::string line;

        if (!is.good())
        {
            std::cout << "[ERROR] Failed readAMScenarioCsv" << std::endl;
            exit(1);
        }

        kiapi_msgs::msg::MissionData md;
        geometry_msgs::msg::PoseArray pa;
        bool exist_start = false;
        bool exist_end = false;

        while (std::getline(is, line))
        {
            std::vector<std::string> str_vec = split(line, ',');
            if (str_vec.size() != 3)
            {
                std::cout << "[ERROR] splited data size => str_vec.size() != 3" << std::endl;
                exit(1);
            }

            kiapi_msgs::msg::MissionRouteData mrd;
            mrd.route_node_pos_lat = std::stoi(str_vec[0]);
            mrd.route_node_pos_lon = std::stoi(str_vec[1]);
            mrd.route_node_type = static_cast<uint8_t>(std::stoi(str_vec[2]));
            mrd.mission_route_id = 3;
            output.mission_route_list.push_back(mrd);

            if (mrd.route_node_type == NodeType::START)
            {
                md.start_lat = mrd.route_node_pos_lat;
                md.start_lon = mrd.route_node_pos_lon;
                exist_start = true;
            }
            else if (mrd.route_node_type == NodeType::END)
            {
                md.end_lat = mrd.route_node_pos_lat;
                md.end_lon = mrd.route_node_pos_lon;
                exist_end = true;
            }
            else
            {
                /* do nothing */
            }

            double pos_lat = intToDouble(mrd.route_node_pos_lat, 2);
            double pos_lon = intToDouble(mrd.route_node_pos_lon, 3);
            double point_map_x{0.0};
            double point_map_y{0.0};

            GPSToPMap(pos_lat, pos_lon, point_map_x, point_map_y);

            geometry_msgs::msg::Pose checkpoint;
            checkpoint.position.x = point_map_x;
            checkpoint.position.y = point_map_y;
            checkpoint.position.z = 0.0;
            checkpoint.orientation.x = 0.0;
            checkpoint.orientation.y = 0.0;
            checkpoint.orientation.z = 0.0;
            checkpoint.orientation.w = 0.0;

            pa.poses.push_back(checkpoint);
        }
        md.mission_id = 3;
        output.mission_list.push_back(md);
        output.mission_count = output.mission_route_list.size();

        if (!exist_start || !exist_end)
        {
            std::cout << "[ERROR] !exist_start || !exist_end" << std::endl;
            exit(1);
        }

        visualizeCheckPointAM(pa);
        std::cout << "output.mission_route_list.size(): " << output.mission_route_list.size() << std::endl;
    }

    void SimMissionPub::readPMScenarioCsv(const std::string &path, kiapi_msgs::msg::MissionListStage2 &output)
    {
        std::ifstream is(path);
        std::string line;

        if (!is.good())
        {
            std::cout << "[ERROR] Failed readPMScenarioCsv" << std::endl;
            exit(1);
        }

        geometry_msgs::msg::PoseArray pa;
        std::vector<int> score_list;

        while (std::getline(is, line))
        {
            std::vector<std::string> str_vec = split(line, ',');
            if (str_vec.size() != 5)
            {
                std::cout << "[ERROR] splited data size => str_vec.size() != 5" << std::endl;
                exit(1);
            }

            kiapi_msgs::msg::ItemData id;
            id.item_id = static_cast<uint8_t>(std::stoi(str_vec[0]) + 1);
            id.item_type = static_cast<uint8_t>(std::stoi(str_vec[1]) + 1);
            id.item_status = 0;
            id.score = std::stoi(str_vec[2]);
            id.speed = 0;
            id.duration = 0;
            id.pos_lat = std::stoi(str_vec[3]);
            id.pos_long = std::stoi(str_vec[4]);
            id.extend = 0;
            output.item_list.push_back(id);

            double pos_lat = intToDouble(id.pos_lat, 2);
            double pos_lon = intToDouble(id.pos_long, 3);
            double point_map_x{0.0};
            double point_map_y{0.0};

            GPSToPMap(pos_lat, pos_lon, point_map_x, point_map_y);

            geometry_msgs::msg::Pose checkpoint;
            checkpoint.position.x = point_map_x;
            checkpoint.position.y = point_map_y;
            checkpoint.position.z = 0.0;
            checkpoint.orientation.x = 0.0;
            checkpoint.orientation.y = 0.0;
            checkpoint.orientation.z = 0.0;
            checkpoint.orientation.w = 0.0;

            pa.poses.push_back(checkpoint);
            score_list.push_back(id.score);
        }
        output.item_count = output.item_list.size();
        std::cout << "output.item_list.size(): " << output.item_list.size() << std::endl;

        visualizeCheckPointPM(pa, score_list);
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sim_stage_mission::SimMissionPub>();

    rclcpp::Rate rate(5);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        node->run();
        rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
