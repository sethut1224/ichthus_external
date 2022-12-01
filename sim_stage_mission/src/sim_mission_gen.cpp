/* ===========================================================================
Copyright 2022. The ICHTHUS Project. All Rights Reserved.
Eunseo Choi (eunseo.choi.d@gmail.com) and Kanghee Kim (kim.kanghee@gmail.com).
Mobility Intelligence & Computing Systems Laboratory, Soongsil University.
=========================================================================== */

#include <sim_mission_gen.hpp>

namespace sim_stage_mission
{
    SimMissionGen::SimMissionGen() : Node("stage_mission_list_test")
    {
        // orig_x_ = this->declare_parameter("orig_x", 0.0); // TODO
        // orig_y_ = this->declare_parameter("orig_y", 0.0); // TODO
        // orig_z_ = this->declare_parameter("orig_z", 0.0); // TODO
        save_dir_ = this->declare_parameter("save_dir", "/root/shared_dir/eunseo/");
        mission_type_ = this->declare_parameter("mission_type", "AM");

        std::cout << "item_index_list_.size() = " << item_index_list_.size() << std::endl;

        pm_score_list_ = {-5, -5, -5, -10, -5, -10, -8, -5, 5, 5, 5, 10};

        rclcpp::QoS qos{1};
        qos.transient_local();

        pub_pose_list_ = create_publisher<geometry_msgs::msg::PoseArray>("/mission_poses", qos);

        sub_save_mission_list_ = create_subscription<std_msgs::msg::String>(
            "/save_mission_list",
            rclcpp::QoS(1),
            std::bind(&SimMissionGen::callbackSaveMissionList, this, std::placeholders::_1));

        sub_clicked_point_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/planning/mission_planning/goal", // TODO: clicked_point
            rclcpp::QoS(1),
            std::bind(&SimMissionGen::callbackClickedPoint, this, std::placeholders::_1));

        sub_item_index_ = create_subscription<std_msgs::msg::Int32>(
            "/item_index",
            rclcpp::QoS(1),
            std::bind(&SimMissionGen::callbackItemIndex, this, std::placeholders::_1));
    }

    SimMissionGen::~SimMissionGen()
    {
    }

    void SimMissionGen::callbackSaveMissionList(const std_msgs::msg::String::ConstSharedPtr msg)
    {
        std::cout << "[INFO] save mission list" << std::endl;

        if (mission_type_ == "AM")
        {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dis(0, static_cast<int>((route_node_list_.size() - 1) / 2));
            int start_node = dis(gen);

            std::string file_name = msg->data;
            std::ofstream os(save_dir_ + file_name);
            for (uint32_t i = 0; i < route_node_list_.size(); i++)
            {
                os << route_node_list_[i].route_node_pos_lat << ","
                   << route_node_list_[i].route_node_pos_lon << ",";
                if (static_cast<int>(i) == start_node)
                {
                    os << NodeType::START << "\n";
                }
                else if (i == route_node_list_.size() - 1)
                {
                    os << NodeType::END << "\n";
                }
                else
                {
                    os << NodeType::ROUTE << "\n";
                }
            }

            route_node_list_.clear();
            pose_list_.poses.clear();
        }
        else if (mission_type_ == "PM")
        {
            std::string file_name = msg->data;
            std::ofstream os(save_dir_ + file_name);

            if (item_data_list_.size() != item_index_list_.size())
            {
                std::cout << "[ERROR] item_data_list_.size() != item_index_list_.size()\n";
                exit(1);
            }

            for (uint32_t i = 0; i < item_data_list_.size(); i++)
            {
                if ((item_index_list_[i] < 0) || (item_index_list_[i] > 12))
                {
                    std::cout << "[ERROR] item_index_list_[i] out of range\n";
                    exit(1);
                }

                os << i << ",";
                if (item_index_list_[i] < 8)
                {
                    int score = pm_score_list_[item_index_list_[i]];
                    os << ItemType::PLUS << ","
                       << score << ",";
                }
                else if (item_index_list_[i] < 12)
                {
                    int score = pm_score_list_[item_index_list_[i]];
                    os << ItemType::MINUS << ","
                       << score << ",";
                }
                else if (item_index_list_[i] == 12)
                {
                    int score = 0;
                    os << ItemType::BOOST << ","
                       << score << ",";
                }

                os << item_data_list_[i].pos_lat << ","
                   << item_data_list_[i].pos_long << "\n";
            }

            item_index_list_.clear();
            item_data_list_.clear();
            pose_list_.poses.clear();
        }
    }

    void SimMissionGen::callbackClickedPoint(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        // std::cout << "callbackClickedPoint\n";

        double point_map_x = msg->pose.position.x;
        double point_map_y = msg->pose.position.y;

        RCLCPP_INFO(this->get_logger(), "point_map x: %lf", point_map_x);
        RCLCPP_INFO(this->get_logger(), "point_map y: %lf", point_map_y);

        pose_list_.header = msg->header;
        pose_list_.poses.push_back(msg->pose);
        pub_pose_list_->publish(pose_list_);

        /* PointMap to UTM */
        lanelet::BasicPoint3d utm_point{point_map_x + 445815.539508, point_map_y + 3944953.128090, 0.0};

        RCLCPP_INFO(this->get_logger(), "utm_point x: %lf", utm_point.x());
        RCLCPP_INFO(this->get_logger(), "utm_point y: %lf", utm_point.y());

        /* UTM to GPS */
        int zone = 52;
        bool northp = true;
        double pos_lat{};
        double pos_lon{};
        GeographicLib::UTMUPS::Reverse(zone, northp, utm_point.x(), utm_point.y(), pos_lat, pos_lon);

        RCLCPP_INFO(this->get_logger(), "GPS x: %lf", pos_lat);
        RCLCPP_INFO(this->get_logger(), "GPS y: %lf", pos_lon);

        if (mission_type_ == "AM")
        {
            kiapi_msgs::msg::MissionRouteData md;
            md.route_node_pos_lat = doubleToInt(pos_lat);
            md.route_node_pos_lon = doubleToInt(pos_lon);
            route_node_list_.push_back(md);

            RCLCPP_INFO(this->get_logger(), "[INFO] route_node_list_.size(): %ld", route_node_list_.size());
        }
        else if (mission_type_ == "PM")
        {
            kiapi_msgs::msg::ItemData id;
            // id.item_id = 0; // TODO
            // id.item_type = 0; // TODO
            // id.item_status = 0;
            // id.score = 0; // TODO
            // id.speed = 0;
            // id.duration = 0;
            id.pos_lat = doubleToInt(pos_lat);
            id.pos_long = doubleToInt(pos_lon);
            // id.extend = 0;
            item_data_list_.push_back(id);

            RCLCPP_INFO(this->get_logger(), "[INFO] item_data_list_.size(): %ld", item_data_list_.size());
        }
    }

    void SimMissionGen::callbackItemIndex(const std_msgs::msg::Int32::ConstSharedPtr msg)
    {
        item_index_list_.push_back(msg->data);

        RCLCPP_INFO(this->get_logger(), "[INFO] item_index_list_.size(): %ld", item_index_list_.size());
    }

    int SimMissionGen::doubleToInt(const double num)
    {
        char num_str[256];
        char *num_p;
        int cnt = 0;
        int result = 0;

        sprintf(num_str, "%lf", num);
        num_p = num_str;

        while (*num_p != '.')
        {
            ++num_p;
        }
        ++num_p;

        while (*num_p)
        {
            ++cnt;
            ++num_p;
        }

        result = num * pow(10, cnt);
        return result;
    }

    void SimMissionGen::run()
    {
        return;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sim_stage_mission::SimMissionGen>();

    rclcpp::Rate rate(5);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        // node->run();
        rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
