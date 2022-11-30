/*  ===========================================================================
    Copyright 2022. The ICHTHUS Project. All Rights Reserved.
    Jaemin Lee (xoz1206@gmail.com),
    Youngjun Han (young@ssu.ac.kr)
    Vision System Laboratory, Soongsil University.
    [Licensed under the MIT License]  
    ===========================================================================*/

#include <mission_manager_am.hpp>
#include <vector>
#include <iostream>

namespace mission_manager
{
    MissionManagerNode::MissionManagerNode() : Node("mission_manager_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // mission planning
        sub_stage1_mission_list_ = this->create_subscription<kiapi_msgs::msg::MissionListStage1>("/v2x/stage1_mission_list", 1,
                std::bind(&MissionManagerNode::stage1MissionListsCallback, this, std::placeholders::_1));

        sub_stage2_mission_list_ = this->create_subscription<kiapi_msgs::msg::MissionListStage2>("/v2x/stage2_mission_list", 1,
                std::bind(&MissionManagerNode::stage2MissionListsCallback, this, std::placeholders::_1));

        sub_my_pose_ = this->create_subscription<nav_msgs::msg::Odometry>("/localization/kinematic_state", 1, 
                std::bind(&MissionManagerNode::poseCallback, this, std::placeholders::_1));

        sub_arrive_start_pose_success_ = this->create_subscription<std_msgs::msg::UInt8>("/v2x/arrive_response", 1, 
                std::bind(&MissionManagerNode::successArriveStartPoseCallback, this, std::placeholders::_1));

        map_subscriber_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
            "/map/vector_map", rclcpp::QoS{10}.transient_local(),
            std::bind(&MissionManagerNode::mapCallback, this, std::placeholders::_1));

        sub_tracking_objects_ = this->create_subscription<autoware_auto_perception_msgs::msg::TrackedObjects>("/perception/object_recognition/tracking/objects", 1,
                std::bind(&MissionManagerNode::trackingObjectsCallback, this, std::placeholders::_1));

	    sub_speed_bust_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/fusion/speed_bust_poses", 1,
                std::bind(&MissionManagerNode::speedBustCallback, this, std::placeholders::_1));
                
        sub_lidar_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/no_ground_pointcloud", rclcpp::SensorDataQoS(),
                std::bind(&MissionManagerNode::pointcloudCallback, this, std::placeholders::_1));

        sub_trajectory_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>("/planning/scenario_planning/trajectory", 1,
                std::bind(&MissionManagerNode::trajectoryCallback, this, std::placeholders::_1));

        sub_behavior_path_ = this->create_subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>("/path_with_lane_id", 1,
                std::bind(&MissionManagerNode::PathCallback, this, std::placeholders::_1));
        
        // sub_behavior_path_ = this->create_subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>("/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id", 1,
                // std::bind(&MissionManagerNode::PathCallback, this, std::placeholders::_1));

        
        
        rclcpp::QoS qos{1};
        qos.transient_local();

        pub_checkpoint_ = create_publisher<geometry_msgs::msg::PoseArray>("/mission_manager_am/checkpoints", qos);
        pub_checkpoint_first_debug_ = create_publisher<visualization_msgs::msg::MarkerArray>("/first_checkpoint_debug_markers", qos);
        pub_checkpoint_second_debug_ = create_publisher<visualization_msgs::msg::MarkerArray>("/second_checkpoint_debug_markers", qos);
        pub_checkpoint_original_debug_ = create_publisher<visualization_msgs::msg::MarkerArray>("/original_checkpoint_debug_markers", qos);
        pub_item_list_debug_ = create_publisher<visualization_msgs::msg::MarkerArray>("/item_list", qos);
        pub_mission_manager_msgs_ = create_publisher<mission_manager_msgs::msg::CheckpointsWithLaneId>("/mission_manager_info", 1);
        pub_start_checkpoint_arrive_ = create_publisher<std_msgs::msg::Bool>("/v2x/starting_point_arrive", qos);
        pub_goal_checkpoint_arrive_ = create_publisher<std_msgs::msg::Bool>("/v2x/goal_arrive", qos);
        pub_mission_difficulty_ = create_publisher<std_msgs::msg::UInt8>("/v2x/selected_mission", qos);
        pub_detection_area_debug_ = create_publisher<visualization_msgs::msg::MarkerArray>("/detection_area_debug", qos);
        pub_detection_area_pointcloud_debug_ = create_publisher<sensor_msgs::msg::PointCloud2>("/detection_area_pointcloud_debug_", qos);
        pub_euclidean_objects_debug_ = create_publisher<sensor_msgs::msg::PointCloud2>("/euclidean_objects_debug", qos);
        pub_fake_objects_ = create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>("/lidar_front/detected_objects", qos);
        pub_speed_bust_debug_ = create_publisher<visualization_msgs::msg::MarkerArray>("/selected_speed_bust", qos);
        pub_estop_flag_= create_publisher<std_msgs::msg::Bool>("/ichthus_estop", qos);
        pub_velocity_limit_ = create_publisher<tier4_planning_msgs::msg::VelocityLimit>("/planning/scenario_planning/max_velocity", qos);

        bool use_height = true;
        int min_cluster_size = 0.005;
        int max_cluster_size = 500;
        float tolerance = 0.5;
        float voxel_leaf_size = 0.2;
        int min_points_number_per_voxel = 1;

        cluster_ = std::make_shared<euclidean_cluster::VoxelGridBasedEuclideanCluster>(use_height, min_cluster_size, max_cluster_size, tolerance, voxel_leaf_size, min_points_number_per_voxel);

        isprocessed_ = false;
        getCurrentPose_ = false;
        is_graph_ready_ = false;
        pose_ok_ = false;
        now_checkpoint_index_ = 0;
        now_goal_index_ = 0;

        is_find_start_index_ = false;
        is_find_goal_index_ = false;

        is_published = false;
        is_final_path_publish_ = false;

        start_pose_arrive_ = false;
        goal_pose_arrive_ = false;
        get_tracked_objects_ = false;

        time_count_ = 0;

        mission_name_ = "stage2";
        mission_difficulty_ = 3;

        make_item_lane_priority_ = "left";
        
        is_get_pointcloud_ = false;

        now_velocity_limit_ = 28.0 / 3.6;
    }

    MissionManagerNode::~MissionManagerNode()
    {

    }

    void MissionManagerNode::PathCallback(const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg)
    {
        // std::cout << pose_ok_ << " , " << isprocessed_ << " , " << is_get_pointcloud_ << " , " << is_graph_ready_ << std::endl;
        if(pose_ok_ == false || isprocessed_ == false || is_get_pointcloud_ == false || is_graph_ready_ == false) return;

        if(mission_name_ != "stage1") return;

        std::vector<lanelet::ConstLanelet> crosswalks = getCrosswalksOnPath(*msg, route_handler_.getLaneletMapPtr(), route_handler_.getOverallGraphPtr());
   
        // std::cout << "crosswalk size : " << crosswalks.size() << std::endl;
        // std::cout << "------------------------------" << std::endl;
        // for(size_t i = 0; i < crosswalks.size(); ++i)
        // {
        //     std::cout << crosswalks[i].id() << std::endl;
        // }

        if(crosswalks.size() == 0) return;
        
        std::cout << "------------------------------" << std::endl;

        pcl::PointCloud<pcl::PointXYZ> input_cloud, filtered_cloud, filtered_cloud_map_frame, last_pointcloud;
        pcl::fromROSMsg(origin_pcd_, input_cloud);
            
        pcl::PointCloud<pcl::PointXYZ> voxelized_input_cloud = applyVoxelGridFilter(input_cloud);
            
        for(size_t i = 0; i < voxelized_input_cloud.points.size(); ++i)
        {
            double dist = std::sqrt(std::pow(voxelized_input_cloud.points[i].x, 2) + std::pow(voxelized_input_cloud.points[i].y, 2));

            if(voxelized_input_cloud.points[i].x > -5 && voxelized_input_cloud.points[i].z < 3 && dist < 50)
            {
                filtered_cloud.points.push_back(voxelized_input_cloud.points[i]);   
            }                 
        }

        std::cout << "second step finish ( filtered_cloud_points size ) :" << voxelized_input_cloud.points.size() << std::endl;

        // 3.LiDAR transform to map frame
        geometry_msgs::msg::TransformStamped::SharedPtr LiDAR2map_transform = std::make_shared<geometry_msgs::msg::TransformStamped>();
        getTransform("map", msg->header.frame_id, LiDAR2map_transform);

        Eigen::Matrix4f affine_matrix = tf2::transformToEigen(LiDAR2map_transform->transform).matrix().cast<float>();

        pcl::transformPointCloud(filtered_cloud, filtered_cloud_map_frame, affine_matrix);

        autoware_auto_perception_msgs::msg::DetectedObjects person_objects;
        person_objects.header.frame_id = "base_link";
        person_objects.header.stamp = this->now();

        for(size_t i = 0; i < crosswalks.size(); ++i)
        {
            lanelet::ConstLanelet closest_crosswalk = crosswalks[i];

            const auto polygon2d = closest_crosswalk.polygon2d().basicPolygon();

            // tier4_autoware_utils::Polygon2d crosswalk_polygon;
            // for (const auto & crosswalk_point : polygon2d) {
            //     crosswalk_polygon.outer().emplace_back(crosswalk_point.x(), crosswalk_point.y());
            // }

            // crosswalk_polygon.outer().push_back(crosswalk_polygon.outer().front());

            // std::vector<geometry_msgs::msg::Point> crosswalk_polygon;
            // for(const auto & crosswalk_point : polygon2d)
            // {
            //     geometry_msgs::msg::Point crosswalk_polygon_point;
            //     crosswalk_polygon_point.x = crosswalk_point.x();
            //     crosswalk_polygon_point.y = crosswalk_point.y();
            //     crosswalk_polygon_point.z = 0.0;
            //     crosswalk_polygon.push_back(crosswalk_polygon_point);
            //     std::cout << "polygon : " << crosswalk_polygon_point.x << ", " << crosswalk_polygon_point.y << std::endl; 
            // }

            pcl::PointCloud<pcl::PointXYZ> polygon;
            for(const auto & crosswalk_point : polygon2d)
            {
                pcl::PointXYZ crosswalk_polygon_point;
                crosswalk_polygon_point.x = crosswalk_point.x();
                crosswalk_polygon_point.y = crosswalk_point.y();
                crosswalk_polygon_point.z = 0.0;
                polygon.points.push_back(crosswalk_polygon_point);
            }

            polygon.points.push_back(polygon.points[0]);
            
            pcl::PointCloud<pcl::PointXYZ> last_pointcloud;

            for(size_t k = 0; k < filtered_cloud_map_frame.points.size(); ++k)
            {
                // 4-1. my_detection_area filtering
                cv::Point2f pt;
                pt.x = filtered_cloud_map_frame.points[k].x;
                pt.y = filtered_cloud_map_frame.points[k].y;
                
                geometry_msgs::msg::Pose map_pt;
                map_pt.position.x = filtered_cloud_map_frame.points[k].x;
                map_pt.position.y = filtered_cloud_map_frame.points[k].y;

                // bool isin_crosswalk_lanelet = lanelet::utils::isInLanelet(map_pt, closest_crosswalk, 0.01);
                // geometry_msgs::msg::Point temp_pt;
                // temp_pt.x = pt.x;
                // temp_pt.y = pt.y;
                // temp_pt.z = 0.0;
// 
                // bool isin_crosswalk_lanelet = isInPolygon(crosswalk_polygon, temp_pt);
                // std::cout << temp_pt.x << " , " << temp_pt.y << std::endl;

                pcl::PointXYZ temp_pt;
                temp_pt.x = pt.x;
                temp_pt.y = pt.y;
                temp_pt.z = 0.0;

                bool isin_crosswalk_lanelet = pcl::isXYPointIn2DXYPolygon(temp_pt, polygon);
                
                // std::cout << isin_crosswalk_lanelet  << std::endl;
                
                if(isin_crosswalk_lanelet == false) continue;

                geometry_msgs::msg::Pose point_pose;
                point_pose.position.x = pt.x;
                point_pose.position.y = pt.y;
                point_pose.position.z = 0.0;
                point_pose.orientation.w = 0.0; 
                point_pose.orientation.x = 0.0;
                point_pose.orientation.y = 0.0;
                point_pose.orientation.z = 0.0;

                last_pointcloud.points.push_back(filtered_cloud_map_frame.points[k]);
            }

            std::cout << last_pointcloud.points.size() << std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr last_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(last_pointcloud));
            std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
            cluster_->cluster(last_pointcloud_ptr, clusters);

            std::cout << clusters.size() << std::endl;

            geometry_msgs::msg::TransformStamped::SharedPtr map2Lidar_transform = std::make_shared<geometry_msgs::msg::TransformStamped>();
            getTransform(msg->header.frame_id, "map" , map2Lidar_transform);

            Eigen::Matrix4f map2lidar_affine_matrix = tf2::transformToEigen(map2Lidar_transform->transform).matrix().cast<float>();

            for(size_t m = 0; m < clusters.size(); ++m )
            {
                if(clusters[m].points.size() < 2) continue;

                pcl::CentroidPoint<pcl::PointXYZ> centroid;
                for (const auto p : clusters[m].points) {
                    centroid.add(p);
                }
                pcl::PointXYZ centroid_point;
                centroid.get(centroid_point);

                pcl::PointCloud<pcl::PointXYZ> in_pcd;
                pcl::PointCloud<pcl::PointXYZ> out_pcd;
                in_pcd.points.push_back(centroid_point);

                pcl::transformPointCloud(in_pcd, out_pcd, map2lidar_affine_matrix);

                autoware_auto_perception_msgs::msg::DetectedObject obj;
                autoware_auto_perception_msgs::msg::ObjectClassification oc;
                oc.label = 7;
                oc.probability = 1.0;
                obj.existence_probability = 0.0;
                obj.kinematics.orientation_availability = 0;
                obj.classification.push_back(oc);
                obj.kinematics.pose_with_covariance.pose.position.x = out_pcd.points[0].x;
                obj.kinematics.pose_with_covariance.pose.position.y = out_pcd.points[0].y;
                obj.kinematics.pose_with_covariance.pose.position.z = out_pcd.points[0].z;
                obj.kinematics.pose_with_covariance.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(0.0);
                obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
                obj.shape.dimensions.x = 1.0;
                obj.shape.dimensions.y = 1.0;
                obj.shape.dimensions.z = 1.5;
                obj.kinematics.twist_with_covariance.twist.linear.x = 0.0;
                obj.kinematics.twist_with_covariance.twist.linear.y = 0.0;
                obj.kinematics.twist_with_covariance.twist.linear.z = 0.0;
                obj.kinematics.twist_with_covariance.twist.angular.x = 0.0;
                obj.kinematics.twist_with_covariance.twist.angular.y = 0.0;
                obj.kinematics.twist_with_covariance.twist.angular.z = 0.0;
                obj.kinematics.has_twist = false;

                person_objects.objects.push_back(obj);
            }
        }
    
        std::cout << "detected person objects : " << person_objects.objects.size() << std::endl;
        pub_fake_objects_->publish(person_objects);
    }

    std::vector<lanelet::ConstLanelet> MissionManagerNode::getCrosswalksOnPath(
        const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
        const lanelet::LaneletMapPtr lanelet_map,
        const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs)
    {
        std::vector<lanelet::ConstLanelet> crosswalks;
        std::set<int64_t> unique_lane_ids;

        auto nearest_segment_idx = tier4_autoware_utils::findNearestSegmentIndex(
            path.points, my_pose_, std::numeric_limits<double>::max(), M_PI_2);

        // Add current lane id
        lanelet::ConstLanelets current_lanes;
        if (
            lanelet::utils::query::getCurrentLanelets(
            lanelet::utils::query::laneletLayer(lanelet_map), my_pose_,
            &current_lanes) &&
            nearest_segment_idx) {
            for (const auto & ll : current_lanes) {
                if (
                    ll.id() == path.points.at(*nearest_segment_idx).lane_ids.at(0) ||
                    ll.id() == path.points.at(*nearest_segment_idx + 1).lane_ids.at(0)) {
                    unique_lane_ids.insert(ll.id());
                }
            }
        }

        // Add forward path lane_id
        const size_t start_idx = *nearest_segment_idx ? *nearest_segment_idx + 1 : 0;
        for (size_t i = start_idx; i < path.points.size(); i++) {
            unique_lane_ids.insert(
            path.points.at(i).lane_ids.at(0));  // should we iterate ids? keep as it was.
        }

        for (const auto lane_id : unique_lane_ids) {
            const auto ll = lanelet_map->laneletLayer.get(lane_id);

                constexpr int PEDESTRIAN_GRAPH_ID = 1;
                const auto conflicting_crosswalks = overall_graphs->conflictingInGraph(ll, PEDESTRIAN_GRAPH_ID);
                for (const auto & crosswalk : conflicting_crosswalks) {
                crosswalks.push_back(crosswalk);
            }
        }

        return crosswalks;
    }

    bool MissionManagerNode::isInPolygon(const std::vector<geometry_msgs::msg::Point> & polygon, const geometry_msgs::msg::Point & point)
    {   
        std::vector<tf2::Vector3> polygon_conv;
        for (const auto & el : polygon) {
            polygon_conv.emplace_back(el.x, el.y, el.z);
        }

        tf2::Vector3 point_conv = tf2::Vector3(point.x, point.y, point.z);

        return isInPolygon(polygon_conv, point_conv);
    }

    bool MissionManagerNode::isInPolygon(const std::vector<tf2::Vector3> & polygon, const tf2::Vector3 & point)
    {
    // polygons with fewer than 3 sides are excluded
        if (polygon.size() < 3) {
            return false;
        }

        bool in_poly = false;
        double x1, x2, y1, y2;

        uint32_t nr_poly_points = polygon.size();
        // start with the last point to make the check last point<->first point the first one
        double xold = polygon.at(nr_poly_points - 1).x();
        double yold = polygon.at(nr_poly_points - 1).y();
        for (const auto & poly_p : polygon) {
                double xnew = poly_p.x();
                double ynew = poly_p.y();
                if (xnew > xold) {
                    x1 = xold;
                    x2 = xnew;
                    y1 = yold;
                    y2 = ynew;
                } else {
                    x1 = xnew;
                    x2 = xold;
                    y1 = ynew;
                    y2 = yold;
                }

                if (
                (xnew < point.x()) == (point.x() <= xold) &&
                (point.y() - y1) * (x2 - x1) < (y2 - y1) * (point.x() - x1)) {
                in_poly = !in_poly;
                }
                xold = xnew;
                yold = ynew;
            }
        return in_poly;
    }

    void MissionManagerNode::trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
    {
        // std::cout << "callback trajectory " << std::endl;
        current_trajectory_ = resampleTrajectory(*msg, 0.3);
        
    }

    autoware_auto_planning_msgs::msg::Trajectory MissionManagerNode::resampleTrajectory(
        const autoware_auto_planning_msgs::msg::Trajectory & trajectory, const double interval)
        {
            autoware_auto_planning_msgs::msg::Trajectory resampled;
            resampled.header = trajectory.header;

            if(trajectory.points.size() == 0)
                return resampled;

            resampled.points.push_back(trajectory.points.front());
            for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
                const auto & point = trajectory.points.at(i);

                const auto p1 = tier4_autoware_utils::fromMsg(resampled.points.back().pose.position).to_2d();
                const auto p2 = tier4_autoware_utils::fromMsg(point.pose.position).to_2d();

                if (boost::geometry::distance(p1, p2) > interval) {
                    resampled.points.push_back(point);
                }
            }
            resampled.points.push_back(trajectory.points.back());

            return resampled;
        }

    void MissionManagerNode::speedBustCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr msg)
    {
        // 1. find closest point
        if(pose_ok_ == false || isprocessed_ == false) return;

        if(mission_name_ != "stage1") return;

        if(msg->poses.size() == 0) 
        {
            if(time_count_ < 30) 
            {
                time_count_++;
                return;
            }

            if(now_velocity_limit_ >= (28.0 / 3.6)) return;
            
            tier4_planning_msgs::msg::VelocityLimit velocity_limit_msg;
            velocity_limit_msg.stamp = now();
            velocity_limit_msg.max_velocity = 28.0 / 3.6;
            pub_velocity_limit_->publish(velocity_limit_msg);
            std::cout << "external velocity limit publish ! [ "  << velocity_limit_msg.max_velocity << " ] " << std::endl;

            return;
        }
        geometry_msgs::msg::TransformStamped::SharedPtr LiDAR2map_transform = std::make_shared<geometry_msgs::msg::TransformStamped>();
        getTransform("map", "base_link", LiDAR2map_transform);
        Eigen::Affine3f affine_matrix = tf2::transformToEigen(LiDAR2map_transform->transform).cast<float>();

        geometry_msgs::msg::PoseArray speedBust_msg = *msg;

        for(size_t i = 0; i < speedBust_msg.poses.size(); ++i)
        {
            pcl::PointXYZ in_pt;
            in_pt.x = speedBust_msg.poses[i].position.x;
            in_pt.y = speedBust_msg.poses[i].position.y;
            in_pt.z = speedBust_msg.poses[i].position.z;

            pcl::PointXYZ out_pt = pcl::transformPoint(in_pt, affine_matrix);

            std::cout << out_pt << std::endl;

            geometry_msgs::msg::Pose map_frame_speed_bust_location;
            map_frame_speed_bust_location.position.x = out_pt.x;
            map_frame_speed_bust_location.position.y = out_pt.y;
            map_frame_speed_bust_location.position.z = out_pt.z;

            lanelet::ConstLanelet speed_bust_lanelet;
            const geometry_msgs::msg::Pose const_speed_bust_lanelet_pose(map_frame_speed_bust_location);
            lanelet::utils::query::getClosestLanelet(road_lanelets_, const_speed_bust_lanelet_pose, &speed_bust_lanelet);
            lanelet::ConstLineString2d speed_bust_lanelet_centerline = resample(speed_bust_lanelet.centerline2d(), 0.3);

            double min_dist = 999999.0;
            geometry_msgs::msg::Pose speed_bust_point;

            for (const auto & point : speed_bust_lanelet_centerline)
            {
                double dist = std::sqrt(std::pow(const_speed_bust_lanelet_pose.position.x - point.x(), 2) + std::pow(const_speed_bust_lanelet_pose.position.y - point.y(), 2));
            
                if(dist < min_dist)
                {
                    min_dist = dist;
                    speed_bust_point.position.x = point.x();
                    speed_bust_point.position.y = point.y();
                    speed_bust_point.position.z = 0.0;
                    speed_bust_point.orientation.x = 0.0;
                    speed_bust_point.orientation.y = 0.0;
                    speed_bust_point.orientation.z = 0.0;
                    speed_bust_point.orientation.w = 0.0;
                }
            }

            // only select is in 5m
            if(min_dist < 5)
            {
                bool isin = false;

                for(size_t m = 0; m < speed_bust_point_vec_.size(); ++m)
                {
                    double dist = std::sqrt(std::pow(speed_bust_point_vec_[m].first.position.x - speed_bust_point.position.x, 2) + std::pow(speed_bust_point_vec_[m].first.position.y - speed_bust_point.position.y, 2));

                    if(dist < 2)
                    {
                        isin = true;
                        speed_bust_point_vec_[m].second++;
                        break;
                    }
                }

                if(isin == false)
                    speed_bust_point_vec_.push_back(std::make_pair(speed_bust_point, 0));
            }
        }

        // 2. select high score speed bust point
        
        geometry_msgs::msg::Pose selected_speed_bust_point;

        int max_count = -1;

        for(size_t i = 0; i < speed_bust_point_vec_.size(); ++i)
        {
            if(speed_bust_point_vec_[i].second > max_count)
            {
                selected_speed_bust_point = speed_bust_point_vec_[i].first;
                max_count = speed_bust_point_vec_[i].second;
            }
        }

        if(max_count < 3)
            return;

        // 3. visualization selected_speed_bust_point
        
        
        visualization_msgs::msg::MarkerArray MA;
        
        for(size_t i = 0; i < speed_bust_point_vec_.size(); ++i)
        {
            visualization_msgs::msg::Marker M;
            M.header.frame_id = "map";
            M.header.stamp = this->now();
            M.ns = "speed_bust";
            M.id = 0;
            M.type = visualization_msgs::msg::Marker::CUBE;
            M.action = visualization_msgs::msg::Marker::ADD;
            M.pose = speed_bust_point_vec_[i].first;
            M.scale.x = 2.0;
            M.scale.y = 2.0;
            M.scale.z = 2.0;
            M.color.a = 1.0;
            M.color.r = 0.0;
            M.color.g = 0.0;
            M.color.b = 1.0;

            MA.markers.push_back(M);
        }
        
        visualization_msgs::msg::Marker M;
        M.header.frame_id = "map";
        M.header.stamp = this->now();
        M.ns = "speed_bust";
        M.id = 0;
        M.type = visualization_msgs::msg::Marker::CUBE;
        M.action = visualization_msgs::msg::Marker::ADD;
        M.pose = selected_speed_bust_point;
        M.scale.x = 2.0;
        M.scale.y = 2.0;
        M.scale.z = 2.0;
        M.color.a = 1.0;
        M.color.r = 1.0;
        M.color.g = 0.0;
        M.color.b = 0.0;

        MA.markers.push_back(M);

        pub_speed_bust_debug_->publish(MA);

        // 4. is in my local trajectory
        
        bool is_in_trajectory = false;

        std::cout << "trajectory" << std::endl;
        for(size_t i = 0; i < current_trajectory_.points.size(); ++i)
        {
            // std::cout << current_trajectory_.points[i].pose.position.x << " , " << current_trajectory_.points[i].pose.position.y << std::endl;

            double dist = std::sqrt(std::pow(selected_speed_bust_point.position.x - current_trajectory_.points[i].pose.position.x, 2) + std::pow(selected_speed_bust_point.position.y - current_trajectory_.points[i].pose.position.y, 2));
            std::cout << "check distance trajectory : " <<  dist << std::endl;
            if(dist < 2)
            {
                is_in_trajectory = true;
                std::cout << "speed bust is in my path !!" << std::endl;
                break;
            }
        }

        double my_pose_dist = std::sqrt(std::pow(selected_speed_bust_point.position.x - my_pose_.position.x, 2) + std::pow(selected_speed_bust_point.position.y - my_pose_.position.y, 2));
        std::cout << "my_pose dist : " << my_pose_dist << std::endl;

        // 5. external velocity limit publish
        if(is_in_trajectory == true && my_pose_dist < 50)
        {
            if(now_velocity_limit_ == (9.0 / 3.6))
            {
                time_count_ = 0;
                return;
            }
            
            tier4_planning_msgs::msg::VelocityLimit velocity_limit_msg;
            velocity_limit_msg.stamp = now();
            velocity_limit_msg.max_velocity = 9.0 / 3.6;
            pub_velocity_limit_->publish(velocity_limit_msg);
            std::cout << "external velocity limit publish ! [ "  << velocity_limit_msg.max_velocity << " ] " << std::endl;
            now_velocity_limit_ = 9.0 / 3.6;
            
        }
        else
        {

            time_count_++;

            if(now_velocity_limit_ == (28.0 / 3.6)) return;

            tier4_planning_msgs::msg::VelocityLimit velocity_limit_msg;
            velocity_limit_msg.stamp = now();
            velocity_limit_msg.max_velocity = 28.0 / 3.6;
            pub_velocity_limit_->publish(velocity_limit_msg);
            std::cout << "external velocity limit publish ! [ "  << velocity_limit_msg.max_velocity << " ] " << std::endl;
            now_velocity_limit_ = 28.0 / 3.6;
        }
    }
    
    void MissionManagerNode::trackingObjectsCallback(const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr msg)
    {
        tracked_objects_ = *msg;
        
        get_tracked_objects_ = true;

    }

    void MissionManagerNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        // if(pose_ok_ == false || get_tracked_objects_ == false || isprocessed_ == false) return;

        is_get_pointcloud_ = true;
        origin_pcd_ = *msg;

        if(pose_ok_ == false || isprocessed_ == false) return;

        if(mission_name_ != "stage1") return;

        lanelet::ConstLanelet my_lanelet;
        const geometry_msgs::msg::Pose const_my_pose(my_pose_);
        lanelet::utils::query::getClosestLanelet(road_lanelets_, const_my_pose, &my_lanelet);
        
        lanelet::ConstLanelet next_lanelet = routing_graph_ptr_->following(my_lanelet)[0];

        std::vector<lanelet::ConstPoint2d> lanelet_point2d_vec;

        lanelet::ConstLineString2d my_lanelet_centerline = my_lanelet.centerline2d();
        lanelet::ConstLineString2d next_lanelet_centerline = next_lanelet.centerline2d();
        
        lanelet::ConstLineString2d resampled_my_lanelet_centerline = resample(my_lanelet_centerline, 0.3);
        lanelet::ConstLineString2d resampled_next_lanelet_centerline= resample(next_lanelet_centerline, 0.3);

        bool is_in_intersection_me = false;

        for (const auto & point : my_lanelet_centerline)
        {
            if(point == my_lanelet_centerline[0]) continue;
            if(point == my_lanelet_centerline[my_lanelet_centerline.size() - 1]) continue;

            geometry_msgs::msg::Pose next_pt;
            next_pt.position.x = point.x();
            next_pt.position.y = point.y();
            next_pt.position.z = 0.0;

            const geometry_msgs::msg::Pose const_next_pt(next_pt);   
            int next_closest_lanelets_num = calcClosestLaneletsNum(road_lanelets_, const_next_pt);

            if(next_closest_lanelets_num != 1)
            {
                // std::cout << "my lane is in intersection !" << std::endl;
                is_in_intersection_me = true;
                break;
            }
        }
        
        if(is_in_intersection_me == false)
        {
            for (const auto & point : resampled_my_lanelet_centerline)
                lanelet_point2d_vec.push_back(point);
        }
        
        bool is_in_intersection = false;
        
        for (const auto & point : next_lanelet_centerline)
        {
            if(point == next_lanelet_centerline[0]) continue;
            if(point == next_lanelet_centerline[next_lanelet_centerline.size() - 1]) continue;

            geometry_msgs::msg::Pose next_pt;
            next_pt.position.x = point.x();
            next_pt.position.y = point.y();
            next_pt.position.z = 0.0;

            const geometry_msgs::msg::Pose const_next_pt(next_pt);
            int next_closest_lanelets_num = calcClosestLaneletsNum(road_lanelets_, const_next_pt);

            if(next_closest_lanelets_num != 1)
            {
                is_in_intersection = true;
                // std::cout << "next lane is in intersection !" << std::endl;
                break;
            }
        }

        if(is_in_intersection == false)
        {
            for (const auto & point : resampled_next_lanelet_centerline)
                lanelet_point2d_vec.push_back(point);
        }

        if(is_in_intersection == true && is_in_intersection_me == true)
            return;

        // 2.[LiDAR frame] z < 0 && x > 0 && my pose "30m" area
        pcl::PointCloud<pcl::PointXYZ> input_cloud, filtered_cloud, filtered_cloud_map_frame, last_pointcloud;
        pcl::fromROSMsg(*msg, input_cloud);
        
        pcl::PointCloud<pcl::PointXYZ> voxelized_input_cloud = applyVoxelGridFilter(input_cloud);

        for(size_t i = 0; i < voxelized_input_cloud.points.size(); ++i)
        {
            double dist = std::sqrt(std::pow(voxelized_input_cloud.points[i].x, 2) + std::pow(voxelized_input_cloud.points[i].y, 2));

            if(voxelized_input_cloud.points[i].x > 0 && voxelized_input_cloud.points[i].z < 2 && dist < 30)
            {
                filtered_cloud.points.push_back(voxelized_input_cloud.points[i]);   
            }                 
        }

        // std::cout << "second step finish ( filtered_cloud_points size ) :" << voxelized_input_cloud.points.size() << std::endl;

        // 3.LiDAR transform to map frame
        geometry_msgs::msg::TransformStamped::SharedPtr LiDAR2map_transform = std::make_shared<geometry_msgs::msg::TransformStamped>();
        getTransform("map", msg->header.frame_id, LiDAR2map_transform);

        Eigen::Matrix4f affine_matrix = tf2::transformToEigen(LiDAR2map_transform->transform).matrix().cast<float>();

        pcl::transformPointCloud(filtered_cloud, filtered_cloud_map_frame, affine_matrix);
        
        // std::cout << "third step finish ( filtered_cloud_map_frame_points size ) : " << filtered_cloud_map_frame.points.size() << std::endl;
        // 4.[Map frame] find pointcloud in my detection area and out of tracked objects and my lanelet
        for(size_t i = 0; i < filtered_cloud_map_frame.points.size(); ++i)
        {
            // 4-1. my_detection_area filtering
            cv::Point2f pt;
            pt.x = filtered_cloud_map_frame.points[i].x;
            pt.y = filtered_cloud_map_frame.points[i].y;
            
            geometry_msgs::msg::Pose map_pt;
            map_pt.position.x = filtered_cloud_map_frame.points[i].x;
            map_pt.position.y = filtered_cloud_map_frame.points[i].y;

            bool isin_my_lanelet = lanelet::utils::isInLanelet(map_pt, my_lanelet, 0.01);
            bool isin_next_lanelet = lanelet::utils::isInLanelet(map_pt, next_lanelet, 0.01);
            
            const geometry_msgs::msg::Pose const_map_pt(map_pt);
            int closest_lanelets_num = calcClosestLaneletsNum(road_lanelets_, const_map_pt);
            
            if(isin_my_lanelet == false && isin_next_lanelet == false) continue;
            if(closest_lanelets_num > 2) continue;

            geometry_msgs::msg::Pose point_pose;
            point_pose.position.x = pt.x;
            point_pose.position.y = pt.y;
            point_pose.position.z = 0.0;
            point_pose.orientation.w = 0.0; 
            point_pose.orientation.x = 0.0;
            point_pose.orientation.y = 0.0;
            point_pose.orientation.z = 0.0;

            // 4-2. lanelet filtering
            lanelet::ConstLanelet point_lanelet;
            const geometry_msgs::msg::Pose const_point_pose(point_pose);
            lanelet::utils::query::getClosestLanelet(road_lanelets_, const_point_pose, &point_lanelet);
            
            if(point_lanelet.id() != my_lanelet.id() && point_lanelet.id() != next_lanelet.id())
                continue;
            
            last_pointcloud.points.push_back(filtered_cloud_map_frame.points[i]);
        }

        // debug 4
        sensor_msgs::msg::PointCloud2 detection_area_pointcloud_msg;
        pcl::toROSMsg(last_pointcloud, detection_area_pointcloud_msg);
        detection_area_pointcloud_msg.header.frame_id = "map";
        detection_area_pointcloud_msg.header.stamp = msg->header.stamp;

        pub_detection_area_pointcloud_debug_->publish(detection_area_pointcloud_msg);

        // std::cout << "fourth step finish ( last_pointcloud point size ) : " << last_pointcloud.points.size() << std::endl;
        // 5.check estop
        
        // start check pointcloud 
        // using euclidean clustering
        pcl::PointCloud<pcl::PointXYZ>::Ptr last_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(last_pointcloud));
        std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
        cluster_->cluster(last_pointcloud_ptr, clusters);

        // std::cout << "detected cluster size : " << clusters.size() << std::endl;

        pcl::PointCloud<pcl::PointXYZRGB> out_cloud;
        
        std::vector<pcl::PointCloud<pcl::PointXYZ>> filtered_clusters;

        for(size_t i = 0; i < clusters.size(); ++i)
        {
            constexpr uint8_t color_data[] = {200, 0,   0, 0,   200, 0,   0, 0,   200,
                                    200, 200, 0, 200, 0,   200, 0, 200, 200};  // 6 pattern

            // [clustering] noise filtering
            if(clusters[i].points.size() < 2) continue; 

            // [clustering] Pole filtering
            pcl::PointXYZ min_pt;
            pcl::PointXYZ max_pt;
            pcl::getMinMax3D(clusters[i], min_pt, max_pt);
            
            pcl::CentroidPoint<pcl::PointXYZ> centroid;
            for (const auto p : clusters[i].points) {
                centroid.add(p);
            }
            pcl::PointXYZ centroid_point;
            centroid.get(centroid_point);

            double max_dist = -999.999;
            double max_z = -999.999;
            double min_z = 999.999;

            for(size_t k = 0; k < clusters[i].points.size(); ++k)
            {
                double pt2center_dist = std::sqrt(std::pow(clusters[i].points[k].x - centroid_point.x, 2) + std::pow(clusters[i].points[k].y - centroid_point.y, 2));

                if(pt2center_dist > max_dist)
                    max_dist = pt2center_dist;
                if(clusters[i].points[k].z > max_z)
                    max_z = clusters[i].points[k].z;
                if(clusters[i].points[k].z < min_z)
                    min_z = clusters[i].points[k].z;
            }
            
            // [clustering] pole filtering
            if(max_dist < 0.25) continue;                                     

            // [clustering] car filtering
            if(max_z > my_pose_.position.z + 0.8) continue;

            // [clustering] side object filtering
            double total_min_dist = 99999.9999;

            for(size_t k = 0; k < clusters[i].points.size(); ++k)
            {
                double min_dist = 9999.9999;

                for(size_t m = 0; m < lanelet_point2d_vec.size(); ++m)
                {
                    double temp_dist = std::sqrt(std::pow(clusters[i].points[k].x - lanelet_point2d_vec[m].x(), 2) + std::pow(clusters[i].points[k].y - lanelet_point2d_vec[m].y(), 2));
                    if(min_dist > temp_dist)
                        min_dist = temp_dist;
                }

                if(total_min_dist > min_dist)
                {
                    total_min_dist = min_dist;
                }
            }
            
            if(total_min_dist > 1.0) continue;

            // [clustering] finish

            filtered_clusters.push_back(clusters[i]);

            pcl::PointXYZRGB pt;
            for(size_t k = 0; k < clusters[i].points.size(); ++k)
            {
                pt.x = clusters[i].points[k].x;
                pt.y = clusters[i].points[k].y;
                pt.z = clusters[i].points[k].z;
                pt.r = color_data[3 * (i % 6) + 0];
                pt.g = color_data[3 * (i % 6) + 1];
                pt.b = color_data[3 * (i % 6) + 2];

                out_cloud.points.push_back(pt);
            }
        }

        sensor_msgs::msg::PointCloud2 ros_pointcloud;
        pcl::toROSMsg(out_cloud, ros_pointcloud);
        ros_pointcloud.header = msg->header;
        ros_pointcloud.header.frame_id = "map";

        pub_euclidean_objects_debug_->publish(ros_pointcloud);


        if(filtered_clusters.size() != 0)
        {
            geometry_msgs::msg::TransformStamped::SharedPtr map2Lidar_transform = std::make_shared<geometry_msgs::msg::TransformStamped>();
            getTransform(msg->header.frame_id, "map" , map2Lidar_transform);

            Eigen::Matrix4f map2lidar_affine_matrix = tf2::transformToEigen(map2Lidar_transform->transform).matrix().cast<float>();

            std::cout << "carry_object_detection !!" << std::endl;
            autoware_auto_perception_msgs::msg::DetectedObjects carry_object_kinematics;
            carry_object_kinematics.header.frame_id = "base_link";
            carry_object_kinematics.header.stamp = this->now();

            for(size_t i = 0; i < filtered_clusters.size(); ++i)
            {
                pcl::CentroidPoint<pcl::PointXYZ> carray_object_centroid;
                for (const auto p : filtered_clusters[i].points) {
                    carray_object_centroid.add(p);
                }
                pcl::PointXYZ carray_object_centroid_point;
                carray_object_centroid.get(carray_object_centroid_point);
                
                pcl::PointCloud<pcl::PointXYZ> in_pcd;
                pcl::PointCloud<pcl::PointXYZ> out_pcd;
                in_pcd.points.push_back(carray_object_centroid_point);

                pcl::transformPointCloud(in_pcd, out_pcd, map2lidar_affine_matrix);
                
                autoware_auto_perception_msgs::msg::DetectedObject obj;
                autoware_auto_perception_msgs::msg::ObjectClassification oc;
                oc.label = 7;
                oc.probability = 1.0;
                obj.existence_probability = 0.0;
                obj.kinematics.orientation_availability = 0;
                obj.classification.push_back(oc);
                obj.kinematics.pose_with_covariance.pose.position.x = out_pcd.points[0].x;
                obj.kinematics.pose_with_covariance.pose.position.y = out_pcd.points[0].y;
                obj.kinematics.pose_with_covariance.pose.position.z = out_pcd.points[0].z;
                obj.kinematics.pose_with_covariance.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(0.0);
                obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
                obj.shape.dimensions.x = 1.0;
                obj.shape.dimensions.y = 1.0;
                obj.shape.dimensions.z = 1.5;
                obj.kinematics.twist_with_covariance.twist.linear.x = 0.0;
                obj.kinematics.twist_with_covariance.twist.linear.y = 0.0;
                obj.kinematics.twist_with_covariance.twist.linear.z = 0.0;
                obj.kinematics.twist_with_covariance.twist.angular.x = 0.0;
                obj.kinematics.twist_with_covariance.twist.angular.y = 0.0;
                obj.kinematics.twist_with_covariance.twist.angular.z = 0.0;
                obj.kinematics.has_twist = false;

                carry_object_kinematics.objects.emplace_back(obj);
            }

            pub_fake_objects_->publish(carry_object_kinematics);
        }
            
            
    }

    lanelet::ConstLineString2d MissionManagerNode::resample(lanelet::ConstLineString2d lanelet_boundary, double resolution)
    {
        lanelet::LineString2d new_lanelet_boundary;

        for(size_t i = 0; i < lanelet_boundary.size() - 1; ++i)
        {
            lanelet::Point2d temp_pt;
            temp_pt.x() = lanelet_boundary[i].x();
            temp_pt.y() = lanelet_boundary[i].y();

            new_lanelet_boundary.push_back(temp_pt);

            double dist = std::sqrt(std::pow((lanelet_boundary[i].x() - lanelet_boundary[i+1].x()), 2) + std::pow((lanelet_boundary[i].y() - lanelet_boundary[i+1].y()), 2));
            // std::cout << dist << std::endl;

            if(dist < resolution)
                continue;
            else
            {
                double dx = (lanelet_boundary[i+1].x() - lanelet_boundary[i].x()) / 100.0;
                double dy = (lanelet_boundary[i+1].y() - lanelet_boundary[i].y()) / 100.0;
                
                double move_distance = 0.0;
                
                double current_x = lanelet_boundary[i].x();
                double current_y = lanelet_boundary[i].y();

                while(1)
                {
                    move_distance = move_distance + std::sqrt(dx * dx + dy * dy);
                    if(move_distance > dist)
                        break;

                    lanelet::Point2d pt;
                    pt.x() = current_x + dx;
                    pt.y() = current_y + dy;

                    new_lanelet_boundary.push_back(pt);
                    
                    current_x = pt.x();
                    current_y = pt.y();
                }
            }
        }

        lanelet::Point2d temp_pt;
        temp_pt.x() = lanelet_boundary[lanelet_boundary.size() - 1].x();
        temp_pt.y() = lanelet_boundary[lanelet_boundary.size() - 1].y();

        new_lanelet_boundary.push_back(temp_pt);

        lanelet::ConstLineString2d const_new_lanelet_boundary(new_lanelet_boundary);

        return const_new_lanelet_boundary;
    }

    geometry_msgs::msg::Pose MissionManagerNode::calcClosestPose(lanelet::ConstLineString2d points, geometry_msgs::msg::Pose critic_point)
    {
        double min_distance = 99999.9999;

        geometry_msgs::msg::Pose closest_pose;

        for (const auto & point : points) 
        {
            double distance = std::sqrt(std::pow((critic_point.position.x - point.x()), 2) + std::pow((critic_point.position.y - point.y()), 2));
            if(min_distance > distance)
            {
                closest_pose.position.x = point.x();
                closest_pose.position.y = point.y();
                closest_pose.position.z = 0.0;
                closest_pose.orientation.x = 0.0;
                closest_pose.orientation.y = 0.0;
                closest_pose.orientation.z = 0.0;
                closest_pose.orientation.w = 0.0;
                            
                min_distance = distance;
            }
        }

        return closest_pose;
    }

    std::vector<lanelet::ConstPoint2d> MissionManagerNode::calcBoundaryPoints(lanelet::ConstLineString2d start_lanelet, lanelet::ConstLineString2d end_lanelet, geometry_msgs::msg::Pose start_pose, geometry_msgs::msg::Pose end_pose, bool lane_is_same)
    {
        std::vector<lanelet::ConstPoint2d> vec;

        if(lane_is_same == true) // start_lanelet == end_lanelet
        {
            bool find_start_pose = false;
            bool find_end_pose = false;

            for (const auto & point : start_lanelet)
            {
                if(point.x() == start_pose.position.x && point.y() == start_pose.position.y)
                    find_start_pose = true;
                
                if(point.x() == end_pose.position.x && point.y() == end_pose.position.y)
                    find_end_pose = true;
                
                if(find_start_pose == true && find_end_pose == false)
                    vec.push_back(point);
            }
        }   
        else
        {
            bool find_start_pose = false;
            bool find_end_pose = false;

            for (const auto & point : start_lanelet)
            {
                if(point.x() == start_pose.position.x && point.y() == start_pose.position.y)
                    find_start_pose = true;
                
                if(find_start_pose == true)
                    vec.push_back(point);
            }

            for (const auto & point : end_lanelet)
            {
                if(point.x() == end_pose.position.x && point.y() == end_pose.position.y)
                    find_end_pose = true;
                
                if(find_end_pose == false)
                    vec.push_back(point);
            }
        }    

        return vec;
    }

    void MissionManagerNode::visualizeDetectionArea(std::vector<std::vector<cv::Point2f>> convex_hull_vec)
    {
        visualization_msgs::msg::MarkerArray MA;
        
        int id_ = 0;

        double r = 1.0;
        double g = 0.5;
        double b = 0.0;

        for(size_t k = 0; k < convex_hull_vec.size(); ++k)
        {
            for(size_t i = 0; i < convex_hull_vec[k].size(); ++i)
            {
                visualization_msgs::msg::Marker M;
                M.header.frame_id = "map";
                M.header.stamp = this->now();
                M.ns = "detection_object";
                M.id = id_++;
                M.type = visualization_msgs::msg::Marker::SPHERE;
                M.action = visualization_msgs::msg::Marker::MODIFY;
                M.pose.position.x = convex_hull_vec[k][i].x;
                M.pose.position.y = convex_hull_vec[k][i].y;
                M.pose.position.z = 0.0;
                M.scale.x = 0.5;
                M.scale.y = 0.5;
                M.scale.z = 0.5;
                M.color.a = 1.0;
                M.color.r = r;
                M.color.g = g;
                M.color.b = b;

                MA.markers.push_back(M);
            }

            r = r + 0.5;
            g = g + 0.5;
            b = b + 0.5;

            if(r > 1.0) r = 0.0;
            if(g > 1.0) g = 0.0;
            if(b > 1.0) b = 0.0;
        }
    
        pub_detection_area_debug_->publish(MA);
    }
    
    void MissionManagerNode::poseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        my_pose_ = msg->pose.pose;
        pose_ok_ = true;
        
        
        if(isprocessed_ == true && mission_name_ == "stage1")
        {
            geometry_msgs::msg::Pose checkpoint = PA_.poses[now_checkpoint_index_];
            double next_checkpoint_distance = std::sqrt(std::pow((checkpoint.position.x - my_pose_.position.x ), 2) + std::pow((checkpoint.position.y - my_pose_.position.y), 2));

            if(next_checkpoint_distance < 2)
            {
                std::cout << "number " <<  now_checkpoint_index_ << " checkpoint pass ! " << std::endl;
                now_checkpoint_index_ = now_checkpoint_index_ + 1;
            }
                
            geometry_msgs::msg::Pose start_pose = checkpoints_info_msg_total_.checkpoints.poses[start_pose_index_];
            geometry_msgs::msg::Pose goal_pose = checkpoints_info_msg_total_.checkpoints.poses[goal_pose_index_];

            double to_start_distance = std::sqrt(std::pow((start_pose.position.x - my_pose_.position.x ), 2) + std::pow((start_pose.position.y - my_pose_.position.y), 2));
            double to_goal_distance= std::sqrt(std::pow((goal_pose.position.x - my_pose_.position.x ), 2) + std::pow((goal_pose.position.y - my_pose_.position.y), 2));

            if(to_start_distance < 2 && start_pose_arrive_ == false && start_pose_index_ == now_checkpoint_index_ - 1)
            {
                std_msgs::msg::Bool arrive_msg;
                arrive_msg.data = true;
                pub_start_checkpoint_arrive_->publish(arrive_msg);
                std::cout << "start pose arrive !! send v2x msg !" << std::endl;
                start_pose_arrive_ = true;
            }

            if(to_goal_distance < 2 && goal_pose_arrive_ == false && start_pose_arrive_ == true && goal_pose_index_ == now_checkpoint_index_ - 1)
            {
                std_msgs::msg::Bool arrive_msg;
                arrive_msg.data = true;
                pub_goal_checkpoint_arrive_->publish(arrive_msg);
                std::cout << "Goal pose arrive !! send v2x msg !" << std::endl;
                goal_pose_arrive_ = true;
            }
        }

        if(isprocessed_ == true && mission_name_ == "stage2")
        {
            geometry_msgs::msg::Pose checkpoint = PA_.poses[now_checkpoint_index_];
            double next_checkpoint_distance = std::sqrt(std::pow((checkpoint.position.x - my_pose_.position.x ), 2) + std::pow((checkpoint.position.y - my_pose_.position.y), 2));

            if(next_checkpoint_distance < 3)
            {
                std::cout << "number " <<  now_checkpoint_index_ << " item pass ! " << std::endl;
                now_checkpoint_index_ = now_checkpoint_index_ + 1;
            }
        }
    }

    void MissionManagerNode::successArriveStartPoseCallback(const std_msgs::msg::UInt8::ConstSharedPtr msg)
    {
        if(msg->data == 1 && is_final_path_publish_ == false)
        {
            pub_mission_manager_msgs_->publish(checkpoints_info_msg_second_);
            pub_checkpoint_->publish(checkpoints_info_msg_second_.checkpoints);

            std::cout << "final_path_publish!!" << std::endl;

            is_final_path_publish_ = true;
        }
        if(msg->data == 2 && is_final_path_publish_ == true)
        {
            std::cout << "goal pose arrive !! stage 1 mission clear." << std::endl; 
        }
    }

    void MissionManagerNode::stage1MissionListsCallback(const kiapi_msgs::msg::MissionListStage1::ConstSharedPtr msg)
    {
        if(mission_name_ != "stage1") return;

        if(msg->mission_status == 0 || msg->mission_status == 4)
        {
            std::cout << "Mission Ready Status" << std::endl;

            geometry_msgs::msg::PoseArray temp_PA; // temp code

            checkpoints_info_msg_total_.header.frame_id = "map";
            checkpoints_info_msg_total_.header.stamp = this->now();

            if(isprocessed_ == false && is_graph_ready_ == true && pose_ok_ == true)
            {
                std_msgs::msg::Bool estop;
                estop.data = false;
                pub_estop_flag_->publish(estop);

                int start_lat = 0;
                int start_lon = 0;
                int goal_lat = 0;
                int goal_lon = 0;

                for(size_t i = 0; i < msg->mission_list.size(); ++i)
                {
                    if(msg->mission_list[i].mission_id == mission_difficulty_)
                    {
                        start_lat = msg->mission_list[i].start_lat;
                        start_lon = msg->mission_list[i].start_lon;
                        goal_lat  = msg->mission_list[i].end_lat;
                        goal_lon  = msg->mission_list[i].end_lon;
                        std::cout << "start_lat  : " << start_lat << std::endl;
                        std::cout << "start_lon  : " << start_lon << std::endl;
                        std::cout << "goal_lat  : " << goal_lat << std::endl;
                        std::cout << "goal_lon  : " << goal_lon << std::endl;
                    }
                }

                int index = 0;

                if(msg->mission_route_list.size() == 0)
                {
                    std::cout << "get mission_route_list size  0 !! " << std::endl;
                    return;
                }
                else
                {
                    std::cout << "get mission_route_list size : " <<msg->mission_route_list.size() << std::endl;
                }

                for(size_t i = 0; i < msg->mission_route_list.size(); ++i)
                {
                    if(msg->mission_route_list[i].mission_route_id != mission_difficulty_) continue;
                
                    int lat = msg->mission_route_list[i].route_node_pos_lat;
                    int lon = msg->mission_route_list[i].route_node_pos_lon;

                    double dlat = 0.0;
                    double dlon = 0.0;

                    std::string lat_str = std::to_string(lat);
                    // lat digits fix 10 
                    while(1)
                    {
                        if(lat_str.size() == 9)
                        {
                            dlat = std::stod(lat_str) / 1e+7;
                            break;
                        }

                        if(lat_str.size() < 9)
                        {
                            lat_str = lat_str + "0";
                        }
                        
                        if(lat_str.size() > 9)
                        {
                            lat_str = lat_str.substr(0, 9);
                        }
                    }

                    std::string lon_str = std::to_string(lon);

                    // lat digits fix 10 
                    while(1)
                    {
                        if(lon_str.size() == 10)
                        {
                            dlon = std::stod(lon_str) / 1e+7;
                            break;
                        }
                            
                        if(lon_str.size() < 10)
                        {
                            lon_str = lon_str + "0";
                        }
                        
                        if(lon_str.size() > 10)
                        {
                            lon_str = lon_str.substr(0, 10);
                        }
                    }
                    
                    int zone = 52;
                    bool northup = true;

                    lanelet::BasicPoint3d utm_point{0.0, 0.0, 0.0};

                    GeographicLib::UTMUPS::Forward(dlat, dlon, zone, northup, utm_point.x(), utm_point.y());

                    double point_map_x = utm_point.x() - 445815.539508;
                    double point_map_y = utm_point.y() - 3944953.128090;

                    // find nearest lane and center point 
                    lanelet::Lanelet closest_lanelet;

                    geometry_msgs::msg::Pose gps_pose;
                    gps_pose.position.x = point_map_x;
                    gps_pose.position.y = point_map_y;
                    gps_pose.position.z = 0.0;
                    gps_pose.orientation.x = 0.0;
                    gps_pose.orientation.y = 0.0;
                    gps_pose.orientation.z = 0.0;
                    gps_pose.orientation.w = 0.0;

                    temp_PA.poses.push_back(gps_pose); // temp code

                    const geometry_msgs::msg::Pose const_gps_pose(gps_pose);

                    bool ok = lanelet::utils::query::getClosestLanelet(road_lanelets_, const_gps_pose, &closest_lanelet);

                    if(ok == false)
                    {
                        std::cout << "cannot find closest lanelet! "<< std::endl;
                        continue;
                    }
                                    
                    lanelet::ConstLineString2d lanelet_centerline = closest_lanelet.centerline2d();
                    lanelet::ConstLineString2d resampled_lanelet_centerline = resample(lanelet_centerline, 0.3); 

                    double min_distance = 99999.9999;

                    geometry_msgs::msg::Pose checkpoint;

                    for (const auto & point : resampled_lanelet_centerline) 
                    {
                        double distance = std::sqrt(std::pow((gps_pose.position.x - point.x()), 2) + std::pow((gps_pose.position.y - point.y()), 2));
                        if(min_distance > distance)
                        {
                            checkpoint.position.x = point.x();
                            checkpoint.position.y = point.y();
                            checkpoint.position.z = 0.0;
                            checkpoint.orientation.x = 0.0;
                            checkpoint.orientation.y = 0.0;
                            checkpoint.orientation.z = 0.0;
                            checkpoint.orientation.w = 0.0;
                            
                            min_distance = distance;
                        }
                    }

                    const double lane_yaw = lanelet::utils::getLaneletAngle(closest_lanelet, checkpoint.position);
                    checkpoint.orientation = tier4_autoware_utils::createQuaternionFromYaw(lane_yaw);

                    // intersection filtering
                    const geometry_msgs::msg::Pose pt(checkpoint);
                    int closest_lanelets_num = calcClosestLaneletsNum(road_lanelets_, pt);


                    if(closest_lanelets_num > 2) 
                    {
                        // start and goal pose not pass
                        if(!(lat == start_lat && lon == start_lon) && !(lat == goal_lat && lon == goal_lon)) 
                            continue;
                    }

                    lanelet::ConstLanelets neighbor_lanelets = lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, closest_lanelet);
                    neighbor_lanelets_.push_back(neighbor_lanelets);

                    PA_.poses.push_back(checkpoint);
                    total_checkpoints_size_ = PA_.poses.size() ;

                    std::cout << "total checkpoint size  / neighbor_lanelets size : " <<  PA_.poses.size() << " / " << neighbor_lanelets_.size() <<  std::endl;
                    std::cout <<" ( " << checkpoint.position.x << "," << checkpoint.position.y << " ) " << std::endl;

                    // msg info fill
                    checkpoints_info_msg_total_.checkpoints.poses.push_back(checkpoint);
                        
                    mission_manager_msgs::msg::NeighborLaneletsInfo neighbor_info;

                    for (const auto & llt : neighbor_lanelets)
                        neighbor_info.lanelets.push_back(llt.id());
                        
                    checkpoints_info_msg_total_.neighbor_lanelets.push_back(neighbor_info);
                    checkpoints_info_msg_total_.checkpoints_laneid.push_back(closest_lanelet.id());

                    // find start pose goal pose
                    if(lat == start_lat && lon == start_lon && is_find_start_index_== false)
                    {
                        start_pose_index_ = index;
                        is_find_start_index_ = true;
                        std::cout << "start_pose_index : " << start_pose_index_ << std::endl;
                    }
                    
                    if(lat == goal_lat && lon == goal_lon)
                    {
                        goal_pose_index_ = index;
                        std::cout << "goal_pose_index : " << goal_pose_index_ << std::endl;
                    }    
                    index = index + 1;
                }

                visualizeCheckPointOriginal(temp_PA);
                visualizeCheckPoint(PA_);

                for(size_t i = 0; i < checkpoints_info_msg_total_.checkpoints_laneid.size(); ++i)
                {
                    if(i <= start_pose_index_)
                    {
                        checkpoints_info_msg_first_.checkpoints.poses.push_back(checkpoints_info_msg_total_.checkpoints.poses[i]);
                        checkpoints_info_msg_first_.neighbor_lanelets.push_back(checkpoints_info_msg_total_.neighbor_lanelets[i]);
                        checkpoints_info_msg_first_.checkpoints_laneid.push_back(checkpoints_info_msg_total_.checkpoints_laneid[i]);
                    }
                    
                    if (i > start_pose_index_)
                    {
                        checkpoints_info_msg_second_.checkpoints.poses.push_back(checkpoints_info_msg_total_.checkpoints.poses[i]);
                        checkpoints_info_msg_second_.neighbor_lanelets.push_back(checkpoints_info_msg_total_.neighbor_lanelets[i]);
                        checkpoints_info_msg_second_.checkpoints_laneid.push_back(checkpoints_info_msg_total_.checkpoints_laneid[i]);
                    }
                }

                pub_checkpoint_->publish(checkpoints_info_msg_first_.checkpoints);

                if(msg->mission_status == 4)
                {
                    pub_mission_manager_msgs_->publish(checkpoints_info_msg_first_); // planning start -> planning finish -> control start
                    
                    std::cout << "now status is test mode, start_path_publish !!" << std::endl;
                }
                else
                {
                    std::cout << "now status is standby, waiting for mission_status [0]" << std::endl;
                }

                std::cout << "stage1 mission difficulty publish ! " << std::endl;
                std_msgs::msg::UInt8 mission_difficulty_msg;
                mission_difficulty_msg.data = mission_difficulty_;
                pub_mission_difficulty_->publish(mission_difficulty_msg);

                isprocessed_ = true;
            }
        }

        if(msg->mission_status == 1 && isprocessed_ == true)
        {
            // std::cout << "Mission Start Status ! Engage True Publish !" << std::endl;
            // engage publish
            
            if(is_published == false)
            {
                for(size_t i = 0; i < msg->mission_list.size(); ++i)
                {
                    if(msg->mission_list[i].mission_id == mission_difficulty_)
                    {
                        if(msg->mission_list[i].status == 1)
                        {
                            std::cout << "mission select complete" << std::endl;
                        }
                        else if(msg->mission_list[i].status != 1)
                        {
                            std::cout << "mission select imcomplete" << std::endl;
                            return;
                        }
                    }
                }

                std_msgs::msg::Bool estop;
                estop.data = false;
                pub_estop_flag_->publish(estop);

                std::cout << "now status is Mission Start ! start_path_publish !!" << std::endl;
                pub_mission_manager_msgs_->publish(checkpoints_info_msg_first_); // planning start -> planning finish -> control start
                is_published = true;
            }
        }

        if(msg->mission_status == 2)
        {   
            // std::cout << "Mission Pause Status" << std::endl;
            // estop publish
            std_msgs::msg::Bool estop;
            estop.data = true;
            pub_estop_flag_->publish(estop);
        }

        if(msg->mission_status == 3)
        {
            std::cout << "Mission Finish Status" << std::endl;
        }
        
    }
    
    void MissionManagerNode::stage2MissionListsCallback(const kiapi_msgs::msg::MissionListStage2::ConstSharedPtr msg)
    {
        if(mission_name_ != "stage2") return;

        if((msg->mission_status == 0 || msg->mission_status == 4 ) && isprocessed_ == false)
        {
            std::cout << "Mission Ready Status" << std::endl;
            
            geometry_msgs::msg::PoseArray temp_PA; // temp code

            if(isprocessed_ == false && is_graph_ready_ == true && pose_ok_ == true)
            {
                

                std_msgs::msg::Bool estop;
                estop.data = false;
                pub_estop_flag_->publish(estop);

                if(msg->item_list.size() == 0)
                {
                    std::cout << "get item_list size  0 !! " << std::endl;
                    return;
                }
                else
                {
                    std::cout << "get item_list size : " <<msg->item_list.size() << std::endl;
                }
                
                PA_.poses.push_back(my_pose_);
                PA_score_.push_back(1);

                // check minus item

                for(size_t i = 0; i < msg->item_list.size(); ++i)
                {
                    int lat = msg->item_list[i].pos_lat;
                    int lon = msg->item_list[i].pos_long;

                    double dlat = 0.0;
                    double dlon = 0.0;

                    std::string lat_str = std::to_string(lat);
                    // lat digits fix 10 
                    while(1)
                    {
                        if(lat_str.size() == 9)
                        {
                            dlat = std::stod(lat_str) / 1e+7;
                            break;
                        }

                        if(lat_str.size() < 9)
                        {
                            lat_str = lat_str + "0";
                        }
                        
                        if(lat_str.size() > 9)
                        {
                            lat_str = lat_str.substr(0, 9);
                        }
                    }

                    std::string lon_str = std::to_string(lon);

                    // lat digits fix 10 
                    while(1)
                    {
                        if(lon_str.size() == 10)
                        {
                            dlon = std::stod(lon_str) / 1e+7;
                            break;
                        }
                            
                        if(lon_str.size() < 10)
                        {
                            lon_str = lon_str + "0";
                        }
                        
                        if(lon_str.size() > 10)
                        {
                            lon_str = lon_str.substr(0, 10);
                        }
                    }

                    int zone = 52;
                    bool northup = true;

                    lanelet::BasicPoint3d utm_point{0.0, 0.0, 0.0};

                    GeographicLib::UTMUPS::Forward(dlat, dlon, zone, northup, utm_point.x(), utm_point.y());

                    double point_map_x = utm_point.x() - 445815.539508;
                    double point_map_y = utm_point.y() - 3944953.128090;

                    // find nearest lane and center point 
                    lanelet::Lanelet closest_lanelet;

                    geometry_msgs::msg::Pose gps_pose;
                    gps_pose.position.x = point_map_x;
                    gps_pose.position.y = point_map_y;
                    gps_pose.position.z = 0.0;
                    gps_pose.orientation.x = 0.0;
                    gps_pose.orientation.y = 0.0;
                    gps_pose.orientation.z = 0.0;
                    gps_pose.orientation.w = 0.0;

                    const geometry_msgs::msg::Pose const_gps_pose(gps_pose);

                    bool ok = lanelet::utils::query::getClosestLanelet(road_lanelets_, const_gps_pose, &closest_lanelet);

                    if(ok == false)
                    {
                        std::cout << "cannot find closest lanelet! "<< std::endl;
                        continue;
                    }
                    
                    lanelet::ConstLineString2d lanelet_centerline = closest_lanelet.centerline2d();
                    // lanelet::ConstLineString2d resampled_lanelet_centerline = resample(lanelet_centerline, 0.3);

                    double min_distance = 99999.9999;

                    geometry_msgs::msg::Pose checkpoint;

                    for (const auto & point : lanelet_centerline) 
                    {
                        double distance = std::sqrt(std::pow((gps_pose.position.x - point.x()), 2) + std::pow((gps_pose.position.y - point.y()), 2));
                        if(min_distance > distance)
                        {
                            checkpoint.position.x = point.x();
                            checkpoint.position.y = point.y();
                            checkpoint.position.z = 0.0;
                            checkpoint.orientation.x = 0.0;
                            checkpoint.orientation.y = 0.0;
                            checkpoint.orientation.z = 0.0;
                            checkpoint.orientation.w = 0.0;
                            
                            min_distance = distance;
                        }
                    }

                    original_minus_PA_.poses.push_back(checkpoint);
                }

                for(size_t i = 0; i < msg->item_list.size(); ++i)
                {
                    int lat = msg->item_list[i].pos_lat;
                    int lon = msg->item_list[i].pos_long;

                    double dlat = 0.0;
                    double dlon = 0.0;

                    // std::cout << "before funing : " << lat << " , " << lon << std::endl;

                    std::string lat_str = std::to_string(lat);
                    // lat digits fix 10 
                    while(1)
                    {
                        if(lat_str.size() == 9)
                        {
                            dlat = std::stod(lat_str) / 1e+7;
                            break;
                        }

                        if(lat_str.size() < 9)
                        {
                            lat_str = lat_str + "0";
                        }
                        
                        if(lat_str.size() > 9)
                        {
                            lat_str = lat_str.substr(0, 9);
                        }
                    }

                    std::string lon_str = std::to_string(lon);

                    // lat digits fix 10 
                    while(1)
                    {
                        if(lon_str.size() == 10)
                        {
                            dlon = std::stod(lon_str) / 1e+7;
                            break;
                        }
                            
                        if(lon_str.size() < 10)
                        {
                            lon_str = lon_str + "0";
                        }
                        
                        if(lon_str.size() > 10)
                        {
                            lon_str = lon_str.substr(0, 10);
                        }
                    }
                    
                    // std::cout << "after funing : " << dlat << " , " << dlon << std::endl;

                    int zone = 52;
                    bool northup = true;

                    lanelet::BasicPoint3d utm_point{0.0, 0.0, 0.0};

                    GeographicLib::UTMUPS::Forward(dlat, dlon, zone, northup, utm_point.x(), utm_point.y());

                    double point_map_x = utm_point.x() - 445815.539508;
                    double point_map_y = utm_point.y() - 3944953.128090;

                    // find nearest lane and center point 
                    lanelet::Lanelet closest_lanelet;

                    geometry_msgs::msg::Pose gps_pose;
                    gps_pose.position.x = point_map_x;
                    gps_pose.position.y = point_map_y;
                    gps_pose.position.z = 0.0;
                    gps_pose.orientation.x = 0.0;
                    gps_pose.orientation.y = 0.0;
                    gps_pose.orientation.z = 0.0;
                    gps_pose.orientation.w = 0.0;

                    temp_PA.poses.push_back(gps_pose); // temp code

                    const geometry_msgs::msg::Pose const_gps_pose(gps_pose);

                    bool ok = lanelet::utils::query::getClosestLanelet(road_lanelets_, const_gps_pose, &closest_lanelet);

                    if(ok == false)
                    {
                        std::cout << "cannot find closest lanelet! "<< std::endl;
                        continue;
                    }
                    
                    lanelet::ConstLineString2d lanelet_centerline = closest_lanelet.centerline2d();
                    // lanelet::ConstLineString2d resampled_lanelet_centerline = resample(lanelet_centerline, 0.3);

                    double min_distance = 99999.9999;

                    geometry_msgs::msg::Pose checkpoint;

                    for (const auto & point : lanelet_centerline) 
                    {
                        double distance = std::sqrt(std::pow((gps_pose.position.x - point.x()), 2) + std::pow((gps_pose.position.y - point.y()), 2));
                        if(min_distance > distance)
                        {
                            checkpoint.position.x = point.x();
                            checkpoint.position.y = point.y();
                            checkpoint.position.z = 0.0;
                            checkpoint.orientation.x = 0.0;
                            checkpoint.orientation.y = 0.0;
                            checkpoint.orientation.z = 0.0;
                            checkpoint.orientation.w = 0.0;
                            
                            min_distance = distance;
                        }
                    }

                    const double lane_yaw = lanelet::utils::getLaneletAngle(closest_lanelet, checkpoint.position);
                    checkpoint.orientation = tier4_autoware_utils::createQuaternionFromYaw(lane_yaw);

                    // intersection filtering
                    // const geometry_msgs::msg::Pose pt(checkpoint);
                    // int closest_lanelets_num = calcClosestLaneletsNum(road_lanelets_, pt);
                    // std::cout << closest_lanelets_num << std::endl;
                    // if(closest_lanelets_num > 2) continue;

                    // debug
                    std::pair<geometry_msgs::msg::Pose, int> p;
                    p.first = checkpoint;
                    p.second = msg->item_list[i].item_type;
                    item_lists_.push_back(p);

                    if(msg->item_list[i].item_type == 2) 
                    {
                        // lanelet::ConstLanelet neighbor_lanelet = getNeighborLanelet(closest_lanelet);

                        lanelet::ConstLanelets left_neighbor_lanelets = lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, closest_lanelet);
                        lanelet::ConstLanelets right_neighbor_lanelets = lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, closest_lanelet);
                        
                        std::cout << left_neighbor_lanelets.size() << ", " << right_neighbor_lanelets.size() << std::endl;
                        for(size_t k = 0; k < left_neighbor_lanelets.size(); ++k)
                        {
                            const lanelet::ConstLineString2d neighbor_lanelet_centerline = left_neighbor_lanelets[k].centerline2d();

                            double neighbor_min_distance = 99999.9999;

                            geometry_msgs::msg::Pose neighbor_checkpoint;

                            for (const auto & point : neighbor_lanelet_centerline) 
                            {
                                double distance = std::sqrt(std::pow((checkpoint.position.x - point.x()), 2) + std::pow((checkpoint.position.y - point.y()), 2));
                                if(neighbor_min_distance > distance)
                                {
                                    neighbor_checkpoint.position.x = point.x();
                                    neighbor_checkpoint.position.y = point.y();
                                    neighbor_checkpoint.position.z = 0.0;
                                    neighbor_checkpoint.orientation.x = 0.0;
                                    neighbor_checkpoint.orientation.y = 0.0;
                                    neighbor_checkpoint.orientation.z = 0.0;
                                    neighbor_checkpoint.orientation.w = 0.0;
                                    
                                    neighbor_min_distance = distance;
                                }
                            }

                            const double neighbor_lane_yaw = lanelet::utils::getLaneletAngle(left_neighbor_lanelets[k], neighbor_checkpoint.position);
                            neighbor_checkpoint.orientation = tier4_autoware_utils::createQuaternionFromYaw(neighbor_lane_yaw);
                            
                            bool isfindMinusPA = false;

                            for(size_t m = 0; m < original_minus_PA_.poses.size(); ++m)
                            {
                                double minus_pa_distance = std::sqrt(std::pow(original_minus_PA_.poses[m].position.x - neighbor_checkpoint.position.x, 2) + std::pow(original_minus_PA_.poses[m].position.y - neighbor_checkpoint.position.y, 2));

                                if(minus_pa_distance < 3)
                                    isfindMinusPA = true;
                            }

                            if(isfindMinusPA == false)
                            {
                                std::pair<geometry_msgs::msg::Pose, int> p;
                                p.first = neighbor_checkpoint;
                                p.second = 1; // plus type
                                item_lists_.push_back(p);

                                PA_.poses.push_back(neighbor_checkpoint);
                                PA_score_.push_back(99);
                            }
                            
                            std::cout << neighbor_checkpoint.position.x << " , " << neighbor_checkpoint.position.y << std::endl;

                            std::cout << " left push_back!!" << std::endl;
                        }

                        for(size_t k = 0; k < right_neighbor_lanelets.size(); ++k)
                        {
                            const lanelet::ConstLineString2d neighbor_lanelet_centerline = right_neighbor_lanelets[k].centerline2d();

                            double neighbor_min_distance = 99999.9999;

                            geometry_msgs::msg::Pose neighbor_checkpoint;

                            for (const auto & point : neighbor_lanelet_centerline) 
                            {
                                double distance = std::sqrt(std::pow((checkpoint.position.x - point.x()), 2) + std::pow((checkpoint.position.y - point.y()), 2));
                                if(neighbor_min_distance > distance)
                                {
                                    neighbor_checkpoint.position.x = point.x();
                                    neighbor_checkpoint.position.y = point.y();
                                    neighbor_checkpoint.position.z = 0.0;
                                    neighbor_checkpoint.orientation.x = 0.0;
                                    neighbor_checkpoint.orientation.y = 0.0;
                                    neighbor_checkpoint.orientation.z = 0.0;
                                    neighbor_checkpoint.orientation.w = 0.0;
                                    
                                    neighbor_min_distance = distance;
                                }
                            }

                            const double neighbor_lane_yaw = lanelet::utils::getLaneletAngle(right_neighbor_lanelets[k], neighbor_checkpoint.position);
                            neighbor_checkpoint.orientation = tier4_autoware_utils::createQuaternionFromYaw(neighbor_lane_yaw);
                            
                            bool isfindMinusPA = false;

                            for(size_t m = 0; m < original_minus_PA_.poses.size(); ++m)
                            {
                                double minus_pa_distance = std::sqrt(std::pow(original_minus_PA_.poses[m].position.x - neighbor_checkpoint.position.x, 2) + std::pow(original_minus_PA_.poses[m].position.y - neighbor_checkpoint.position.y, 2));

                                if(minus_pa_distance < 3)
                                    isfindMinusPA = true;
                            }

                            if(isfindMinusPA == false)
                            {
                                std::pair<geometry_msgs::msg::Pose, int> p;
                                p.first = neighbor_checkpoint;
                                p.second = 1; // plus type
                                item_lists_.push_back(p);

                                PA_.poses.push_back(neighbor_checkpoint);
                                PA_score_.push_back(99);
                            }

                            std::cout << neighbor_checkpoint.position.x << " , " << neighbor_checkpoint.position.y << std::endl;

                            std::cout << " right push_back!!" << std::endl;
                        }
                        
                        continue;
                    }

                    if(msg->item_list[i].item_type == 3) 
                    {
                        PA_.poses.push_back(checkpoint);
                        PA_score_.push_back(5);

                        lanelet::ConstLanelets left_neighbor_lanelets = lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, closest_lanelet);
                        lanelet::ConstLanelets right_neighbor_lanelets = lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, closest_lanelet);
                        
                        std::cout << left_neighbor_lanelets.size() << ", " << right_neighbor_lanelets.size() << std::endl;
                        for(size_t k = 0; k < left_neighbor_lanelets.size(); ++k)
                        {
                            const lanelet::ConstLineString2d neighbor_lanelet_centerline = left_neighbor_lanelets[k].centerline2d();

                            double neighbor_min_distance = 99999.9999;

                            geometry_msgs::msg::Pose neighbor_checkpoint;

                            for (const auto & point : neighbor_lanelet_centerline) 
                            {
                                double distance = std::sqrt(std::pow((checkpoint.position.x - point.x()), 2) + std::pow((checkpoint.position.y - point.y()), 2));
                                if(neighbor_min_distance > distance)
                                {
                                    neighbor_checkpoint.position.x = point.x();
                                    neighbor_checkpoint.position.y = point.y();
                                    neighbor_checkpoint.position.z = 0.0;
                                    neighbor_checkpoint.orientation.x = 0.0;
                                    neighbor_checkpoint.orientation.y = 0.0;
                                    neighbor_checkpoint.orientation.z = 0.0;
                                    neighbor_checkpoint.orientation.w = 0.0;
                                    
                                    neighbor_min_distance = distance;
                                }
                            }

                            const double neighbor_lane_yaw = lanelet::utils::getLaneletAngle(left_neighbor_lanelets[k], neighbor_checkpoint.position);
                            neighbor_checkpoint.orientation = tier4_autoware_utils::createQuaternionFromYaw(neighbor_lane_yaw);
                            
                            std::pair<geometry_msgs::msg::Pose, int> p;
                            p.first = neighbor_checkpoint;
                            p.second = msg->item_list[i].item_type; // plus type
                            item_lists_.push_back(p);

                            PA_.poses.push_back(neighbor_checkpoint);
                            PA_score_.push_back(1);
                            // std::cout << neighbor_checkpoint.position.x << " , " << neighbor_checkpoint.position.y << std::endl;

                            // std::cout << " left push_back!!" << std::endl;
                        }

                        for(size_t k = 0; k < right_neighbor_lanelets.size(); ++k)
                        {
                            const lanelet::ConstLineString2d neighbor_lanelet_centerline = right_neighbor_lanelets[k].centerline2d();

                            double neighbor_min_distance = 99999.9999;

                            geometry_msgs::msg::Pose neighbor_checkpoint;

                            for (const auto & point : neighbor_lanelet_centerline) 
                            {
                                double distance = std::sqrt(std::pow((checkpoint.position.x - point.x()), 2) + std::pow((checkpoint.position.y - point.y()), 2));
                                if(neighbor_min_distance > distance)
                                {
                                    neighbor_checkpoint.position.x = point.x();
                                    neighbor_checkpoint.position.y = point.y();
                                    neighbor_checkpoint.position.z = 0.0;
                                    neighbor_checkpoint.orientation.x = 0.0;
                                    neighbor_checkpoint.orientation.y = 0.0;
                                    neighbor_checkpoint.orientation.z = 0.0;
                                    neighbor_checkpoint.orientation.w = 0.0;
                                    
                                    neighbor_min_distance = distance;
                                }
                            }

                            const double neighbor_lane_yaw = lanelet::utils::getLaneletAngle(right_neighbor_lanelets[k], neighbor_checkpoint.position);
                            neighbor_checkpoint.orientation = tier4_autoware_utils::createQuaternionFromYaw(neighbor_lane_yaw);
                            
                            std::pair<geometry_msgs::msg::Pose, int> p;
                            p.first = neighbor_checkpoint;
                            p.second = msg->item_list[i].item_type; // plus type
                            item_lists_.push_back(p);
                            PA_.poses.push_back(neighbor_checkpoint);
                            PA_score_.push_back(1);

                            // std::cout << neighbor_checkpoint.position.x << " , " << neighbor_checkpoint.position.y << std::endl;

                            // std::cout << " right push_back!!" << std::endl;
                        }
                        continue;
                    }

                    PA_.poses.push_back(checkpoint);
                    PA_score_.push_back(msg->item_list[i].score);

                    total_checkpoints_size_ = PA_.poses.size();

                    // std::cout << "total checkpoint size : " <<  PA_.poses.size() << std::endl;
                    // std::cout <<" ( " << checkpoint.position.x << "," << checkpoint.position.y << " ) " << std::endl;
                }
                
                std::cout << "start add 3th lane checkpoint" << std::endl;
                std::cout << "PA 1 size : " << PA_.poses.size() << std::endl;
                for(size_t m = 0;  m < PA_.poses.size(); m++)
                {
                    std::cout << PA_.poses[m].position.x << ", " << PA_.poses[m].position.y << " => " << PA_score_[m] <<  std::endl;
                }

                add3thLaneCheckpoint();
                std::cout << "start Sort pa" << std::endl;
                std::cout << "PA 2 size : " << PA_.poses.size() << std::endl;

                // filtering curve entry
                geometry_msgs::msg::PoseArray filtered_PA;
                std::vector<int> new_PA_score;

                int one_score_sum = 0;
                int second_score_sum = 0;
                int third_score_sum = 0;

                for(size_t i = 0; i < PA_.poses.size(); ++i)
                {
                    geometry_msgs::msg::Pose pose = PA_.poses[i];

                    lanelet::Lanelet closest_lanelet;
                    const geometry_msgs::msg::Pose const_pose(pose);
                    lanelet::utils::query::getClosestLanelet(road_lanelets_, const_pose, &closest_lanelet);
                    
                    if(closest_lanelet.id() == 1869) // make right neighbor checkpoint
                    {
                        // second_score_sum += PA_score_[i];
                        one_score_sum += PA_score_[i];
                    }
                
                    if(closest_lanelet.id() == 1804)
                    {
                        second_score_sum += PA_score_[i];
                    }

                    if(closest_lanelet.id() == 1746)
                    {
                        third_score_sum += PA_score_[i];
                    }
                }

                bool isfiltered = false;
                for(size_t i = 0; i < PA_.poses.size(); ++i)
                {

                    geometry_msgs::msg::Pose pose = PA_.poses[i];

                    lanelet::Lanelet closest_lanelet;
                    const geometry_msgs::msg::Pose const_pose(pose);
                    lanelet::utils::query::getClosestLanelet(road_lanelets_, const_pose, &closest_lanelet);

                    // only one processed
                    if(closest_lanelet.id() == 1746 || closest_lanelet.id() == 1804 || closest_lanelet.id() == 1869)
                    {
                        if(isfiltered == true) continue;

                        isfiltered = true;

                        if(one_score_sum >= second_score_sum  &&  one_score_sum >= third_score_sum)
                        {
                            std::cout << "one lane select !! " << std::endl;
                            lanelet::ConstLanelet one_lane = route_handler_.getLaneletsFromId(1869);
                            lanelet::ConstLineString2d one_lane_centerline = one_lane.centerline2d();
                            lanelet::ConstPoint2d entry_point = one_lane_centerline.front();
                            lanelet::ConstPoint2d exit_point = one_lane_centerline.back();

                            geometry_msgs::msg::Pose temp;

                            temp.position.x = entry_point.x();
                            temp.position.y = entry_point.y();
                            temp.position.z = 0.0;
                            temp.orientation.x = 0.0;
                            temp.orientation.y = 0.0;
                            temp.orientation.z = 0.0;
                            temp.orientation.w = 0.0;

                            filtered_PA.poses.push_back(temp);
                            new_PA_score.push_back(one_score_sum);

                            temp.position.x = exit_point.x();
                            temp.position.y = exit_point.y();
                            temp.position.z = 0.0;
                            temp.orientation.x = 0.0;
                            temp.orientation.y = 0.0;
                            temp.orientation.z = 0.0;
                            temp.orientation.w = 0.0;

                            filtered_PA.poses.push_back(temp);
                            new_PA_score.push_back(one_score_sum);

                        }
                        else if(second_score_sum > 0 && second_score_sum > third_score_sum) // select second lane
                        {
                            std::cout << "second lane select !! " << std::endl;
                            lanelet::ConstLanelet second_lane = route_handler_.getLaneletsFromId(1804);
                            lanelet::ConstLineString2d second_lane_centerline = second_lane.centerline2d();
                            lanelet::ConstPoint2d entry_point = second_lane_centerline.front();
                            lanelet::ConstPoint2d exit_point = second_lane_centerline.back();

                            geometry_msgs::msg::Pose temp;

                            temp.position.x = entry_point.x();
                            temp.position.y = entry_point.y();
                            temp.position.z = 0.0;
                            temp.orientation.x = 0.0;
                            temp.orientation.y = 0.0;
                            temp.orientation.z = 0.0;
                            temp.orientation.w = 0.0;

                            filtered_PA.poses.push_back(temp);
                            new_PA_score.push_back(second_score_sum);

                            temp.position.x = exit_point.x();
                            temp.position.y = exit_point.y();
                            temp.position.z = 0.0;
                            temp.orientation.x = 0.0;
                            temp.orientation.y = 0.0;
                            temp.orientation.z = 0.0;
                            temp.orientation.w = 0.0;

                            filtered_PA.poses.push_back(temp);
                            new_PA_score.push_back(second_score_sum);

                        }
                        else if(third_score_sum > 0 && second_score_sum < third_score_sum) // select third lane
                        {
                            std::cout << "third lane select !! " << std::endl;
                            lanelet::ConstLanelet third_lane = route_handler_.getLaneletsFromId(1746);
                            lanelet::ConstLineString2d third_lane_centerline = third_lane.centerline2d();
                            lanelet::ConstPoint2d entry_point = third_lane_centerline.front();
                            lanelet::ConstPoint2d exit_point = third_lane_centerline.back();

                            geometry_msgs::msg::Pose temp;

                            temp.position.x = entry_point.x();
                            temp.position.y = entry_point.y();
                            temp.position.z = 0.0;
                            temp.orientation.x = 0.0;
                            temp.orientation.y = 0.0;
                            temp.orientation.z = 0.0;
                            temp.orientation.w = 0.0;

                            filtered_PA.poses.push_back(temp);
                            new_PA_score.push_back(third_score_sum);

                            temp.position.x = exit_point.x();
                            temp.position.y = exit_point.y();
                            temp.position.z = 0.0;
                            temp.orientation.x = 0.0;
                            temp.orientation.y = 0.0;
                            temp.orientation.z = 0.0;
                            temp.orientation.w = 0.0;

                            filtered_PA.poses.push_back(temp);
                            new_PA_score.push_back(third_score_sum);

                        }
                        else // select second lane or one lane
                        {
                            std::cout << "one lane select !! " << std::endl;
                            // lanelet::ConstLanelet second_lane = route_handler_.getLaneletsFromId(1804);
                            // lanelet::ConstLineString2d second_lane_centerline = second_lane.centerline2d();
                            // lanelet::ConstPoint2d entry_point = second_lane_centerline.front();

                            // geometry_msgs::msg::Pose temp;

                            // temp.position.x = entry_point.x();
                            // temp.position.y = entry_point.y();
                            // temp.position.z = 0.0;
                            // temp.orientation.x = 0.0;
                            // temp.orientation.y = 0.0;
                            // temp.orientation.z = 0.0;
                            // temp.orientation.w = 0.0;

                            // filtered_PA.poses.push_back(temp);
                            // new_PA_score.push_back(second_score_sum);

                            lanelet::ConstLanelet third_lane = route_handler_.getLaneletsFromId(1869);
                            lanelet::ConstLineString2d third_lane_centerline = third_lane.centerline2d();
                            lanelet::ConstPoint2d entry_point = third_lane_centerline.front();

                            geometry_msgs::msg::Pose temp;

                            temp.position.x = entry_point.x();
                            temp.position.y = entry_point.y();
                            temp.position.z = 0.0;
                            temp.orientation.x = 0.0;
                            temp.orientation.y = 0.0;
                            temp.orientation.z = 0.0;
                            temp.orientation.w = 0.0;

                            filtered_PA.poses.push_back(temp);
                            new_PA_score.push_back(second_score_sum);
                        }
                    }
                    else
                    {
                        filtered_PA.poses.push_back(PA_.poses[i]);
                        new_PA_score.push_back(PA_score_[i]);
                    }
                }
                
                PA_ = filtered_PA;
                PA_score_ = new_PA_score;

                std::cout << "PA 2-1 size : " << PA_.poses.size() << std::endl;
                sortPA();
                
                std::cout << "start Filtering pa" << std::endl;
                std::cout << "PA 3 size : " << PA_.poses.size() << std::endl;
                filteringPA();
                
                // std::cout << "start Add pa" << std::endl;
                // addFinalPA();

                std::cout << "start visualize pa" << std::endl;
                
                visualizeCheckPoint(PA_);
                visualizeCheckPointOriginal(temp_PA);
                visualizeitemList(item_lists_);
                if(msg->mission_status==4)
                {
                    is_published = true;
                    publishStage2MissionMsg();
                }
                    
                isprocessed_ = true;
            }
        }

        if((msg->mission_status == 1 && isprocessed_ == true) || (msg->mission_status == 4 && isprocessed_ == true))
        {
            // std::cout << "Mission Start Status ! Engage True Publish !" << std::endl;
            // engage publish
            
            for(size_t i = 0; i < msg->item_list.size(); ++i)
            {
                // check boost 
                if(msg->item_list[i].item_type == 3) 
                {
                    if(msg->item_list[i].item_status == 1)
                    {
                        std::cout << "get boost item !! max velocity raise up to 100 km/h !! " << std::endl;
                        tier4_planning_msgs::msg::VelocityLimit velocity_limit_msg;
                        velocity_limit_msg.stamp = now();
                        velocity_limit_msg.max_velocity = 99.0 / 3.6;
                        pub_velocity_limit_->publish(velocity_limit_msg);
                    }
                }
            }
            

            if(is_published == false)
            {
                std_msgs::msg::Bool estop;
                estop.data = false;
                pub_estop_flag_->publish(estop);

                publishStage2MissionMsg();
                is_published = true;
            }
        }

        if(msg->mission_status == 2)
        {   
            // std::cout << "Mission Pause Status" << std::endl;
            // estop publish
            
            std_msgs::msg::Bool estop;
            estop.data = true;
            pub_estop_flag_->publish(estop);
        }

        if(msg->mission_status == 3)
        {
            std::cout << "Mission Finish Status" << std::endl;
        }
    }

    void MissionManagerNode::add3thLaneCheckpoint()
    {
        geometry_msgs::msg::Pose entry_pose;

        entry_pose.position.x = 446112.30 - 445815.539508;
        entry_pose.position.y = 3944801.41 - 3944953.128090;
        entry_pose.position.z = 0.0;
        entry_pose.orientation.x = 0.0;
        entry_pose.orientation.y = 0.0;
        entry_pose.orientation.z = 0.0;
        entry_pose.orientation.w = 0.0;

        lanelet::ConstLanelet entry_closest_lanelet;
        const geometry_msgs::msg::Pose const_entry_pose(entry_pose);
        lanelet::utils::query::getClosestLanelet(road_lanelets_, const_entry_pose, &entry_closest_lanelet);

        while(1)
        {
            const auto & right_lane = routing_graph_ptr_->right(entry_closest_lanelet);
            if(!right_lane)
                break;
            else
                entry_closest_lanelet = right_lane.get();
        }

        const lanelet::ConstLineString2d entry_lanelet_centerline = entry_closest_lanelet.centerline2d();

        geometry_msgs::msg::Pose new_entry_pose;

        double entry_min_distance = 999999.999999;

        for (const auto & point : entry_lanelet_centerline) 
        {
            double distance = std::sqrt(std::pow((entry_pose.position.x - point.x()), 2) + std::pow((entry_pose.position.y - point.y()), 2));
            if(entry_min_distance > distance)
            {
                new_entry_pose.position.x = point.x();
                new_entry_pose.position.y = point.y();
                new_entry_pose.position.z = 0.0;
                new_entry_pose.orientation.x = 0.0;
                new_entry_pose.orientation.y = 0.0;
                new_entry_pose.orientation.z = 0.0;
                new_entry_pose.orientation.w = 0.0;
                                
                entry_min_distance = distance;
            }
        }

        const double entry_lane_yaw = lanelet::utils::getLaneletAngle(entry_closest_lanelet, new_entry_pose.position);
        new_entry_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(entry_lane_yaw);

        PA_.poses.push_back(new_entry_pose);
        PA_score_.push_back(100);

        geometry_msgs::msg::Pose exit_pose;
        exit_pose.position.x = 445964.57 - 445815.539508;
        exit_pose.position.y = 3944756.22 - 3944953.128090;
        exit_pose.position.z = 0.0;
        exit_pose.orientation.x = 0.0;
        exit_pose.orientation.y = 0.0;
        exit_pose.orientation.z = 0.0;
        exit_pose.orientation.w = 0.0;

        lanelet::ConstLanelet exit_closest_lanelet;
        const geometry_msgs::msg::Pose const_exit_pose(exit_pose);
        lanelet::utils::query::getClosestLanelet(road_lanelets_, const_exit_pose, &exit_closest_lanelet);

        while(1)
        {
            const auto & right_lane = routing_graph_ptr_->right(exit_closest_lanelet);
            if(!right_lane)
                break;
            else
                exit_closest_lanelet = right_lane.get();
        }

        const lanelet::ConstLineString2d exit_lanelet_centerline = exit_closest_lanelet.centerline2d();

        geometry_msgs::msg::Pose new_exit_pose;

        double exit_min_distance = 999999.999999;

        for (const auto & point : exit_lanelet_centerline) 
        {
            double distance = std::sqrt(std::pow((exit_pose.position.x - point.x()), 2) + std::pow((exit_pose.position.y - point.y()), 2));
            if(exit_min_distance > distance)
            {
                new_exit_pose.position.x = point.x();
                new_exit_pose.position.y = point.y();
                new_exit_pose.position.z = 0.0;
                new_exit_pose.orientation.x = 0.0;
                new_exit_pose.orientation.y = 0.0;
                new_exit_pose.orientation.z = 0.0;
                new_exit_pose.orientation.w = 0.0;
                                
                exit_min_distance = distance;
            }
        }

        const double exit_lane_yaw = lanelet::utils::getLaneletAngle(exit_closest_lanelet, new_exit_pose.position);
        new_exit_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(exit_lane_yaw);

        PA_.poses.push_back(new_exit_pose);
        PA_score_.push_back(100);
    }

    void MissionManagerNode::sortPA()
    {
        geometry_msgs::msg::PoseArray new_PA;
        std::vector<int> new_PA_score;
        geometry_msgs::msg::PoseArray centerlanelet_projection_poses;

        geometry_msgs::msg::Pose neighbor_start_pose;

        // center lanelet projection
        for(size_t i = 0; i < PA_.poses.size(); ++i)
        {
            geometry_msgs::msg::Pose sample_pose = PA_.poses[i];
            lanelet::Lanelet closest_lanelet;
            const geometry_msgs::msg::Pose const_sample_pose(sample_pose);
            lanelet::utils::query::getClosestLanelet(road_lanelets_, const_sample_pose, &closest_lanelet);
            
            lanelet::ConstLanelet neighbor_lanelet = getNeighborCenterLanelet(closest_lanelet);

            lanelet::ConstLineString2d lanelet_centerline = neighbor_lanelet.centerline2d();
            // lanelet::ConstLineString2d resampled_lanelet_centerline = resample(lanelet_centerline, 0.5);

            double min_distance = 99999.9999;

            geometry_msgs::msg::Pose centerlanelet_projection_pose;

            for (const auto & point : lanelet_centerline) 
            {
                double distance = std::sqrt(std::pow((sample_pose.position.x - point.x()), 2) + std::pow((sample_pose.position.y - point.y()), 2));
                if(min_distance > distance)
                {
                    centerlanelet_projection_pose.position.x = point.x();
                    centerlanelet_projection_pose.position.y = point.y();
                    centerlanelet_projection_pose.position.z = 0.0;
                    centerlanelet_projection_pose.orientation.x = 0.0;
                    centerlanelet_projection_pose.orientation.y = 0.0;
                    centerlanelet_projection_pose.orientation.z = 0.0;
                    centerlanelet_projection_pose.orientation.w = 0.0;
                            
                    min_distance = distance;
                }
            }
            if(i == 0)
            {
                neighbor_start_pose = centerlanelet_projection_pose;
            }
            centerlanelet_projection_poses.poses.push_back(centerlanelet_projection_pose);
        }

        std::cout << "PA 2-2 size : " << centerlanelet_projection_poses.poses.size() << std::endl;

        // get center lanelet my_pose
        geometry_msgs::msg::Pose start_pose = PA_.poses[0];
        // new_PA.poses.push_back(start_pose);

        lanelet::Lanelet closest_lanelet;
        const geometry_msgs::msg::Pose const_start_pose(start_pose);
        lanelet::utils::query::getClosestLanelet(road_lanelets_, const_start_pose, &closest_lanelet);
            
        lanelet::ConstLanelet neighbor_lanelet = getNeighborCenterLanelet(closest_lanelet);

        // get total center lanelet point
        std::vector<lanelet::ConstPoint2d> total_center_point_; 

        lanelet::ConstLanelet current_lanelet = neighbor_lanelet;
        int start_lanelet_id = current_lanelet.id();

        while(1)
        {
            const lanelet::ConstLineString2d lanelet_2d = current_lanelet.centerline2d();
            for (const auto & point : lanelet_2d) 
                total_center_point_.push_back(point);

            current_lanelet = routing_graph_ptr_->following(current_lanelet)[0];

            if(current_lanelet.id() == start_lanelet_id)
            {
                const lanelet::ConstLineString2d final_lane = current_lanelet.centerline2d();
                for (const auto & point : final_lane) 
                    total_center_point_.push_back(point);
                
                break;
            }
        }
        
        // using start pose index ( exist two location )
        int start_pose_count = 0;
        
        std::vector<bool> used_vec(centerlanelet_projection_poses.poses.size(), false);

        for(size_t i = 0; i < total_center_point_.size(); ++i)
        {
            int temp_index = 0;

            if(neighbor_start_pose.position.x == total_center_point_[i].x() && neighbor_start_pose.position.y == total_center_point_[i].y())
            {
                start_pose_count++;
                continue;
            }    

            if(start_pose_count == 1)
            {
                bool isfind = false;
                for(size_t k = 0; k < centerlanelet_projection_poses.poses.size(); ++k)
                {
                    if(used_vec[k] == true) continue;

                    if(centerlanelet_projection_poses.poses[k].position.x == total_center_point_[i].x() &&
                        centerlanelet_projection_poses.poses[k].position.y == total_center_point_[i].y())
                    {
                        new_PA.poses.push_back(PA_.poses[k]);
                        new_PA_score.push_back(PA_score_[k]);
                        isfind = true;
                        used_vec[k] = true;
                        break;
                    }
                }

                if(isfind == false)
                {
                    double min_dist = 999999.99999;
                    geometry_msgs::msg::Pose closest_pose;

                    for(size_t k = 0; k < centerlanelet_projection_poses.poses.size(); ++k)
                    {
                        if(used_vec[k] == true) continue;

                        double dist = std::sqrt(std::pow(centerlanelet_projection_poses.poses[k].position.x - total_center_point_[i].x(), 2) +
                            std::pow(centerlanelet_projection_poses.poses[k].position.y - total_center_point_[i].y(), 2));

                        if(dist < min_dist)
                        {
                            min_dist = dist;
                            closest_pose = PA_.poses[k];
                            temp_index = k;
                        }
                    }

                    if(min_dist < 10)
                    {
                        used_vec[temp_index] = true;
                        new_PA.poses.push_back(closest_pose);
                        new_PA_score.push_back(PA_score_[temp_index]);
                    }
                }
            }
        }

        PA_ = new_PA;
        PA_score_ = new_PA_score;
        
    }

    void MissionManagerNode::filteringPA()
    {
        geometry_msgs::msg::PoseArray first_filtered_PA, second_filtered_PA;
        std::vector<int> first_filtered_PA_score, second_filtered_PA_score;

        for(size_t i = 0; i < PA_.poses.size(); ++i)
        {
            geometry_msgs::msg::Pose pose = PA_.poses[i];

            lanelet::Lanelet closest_lanelet;
            const geometry_msgs::msg::Pose const_pose(pose);
            lanelet::utils::query::getClosestLanelet(road_lanelets_, const_pose, &closest_lanelet);

            // if(closest_lanelet.id() == 1711) continue;
            if(closest_lanelet.id() == 2724) continue;
            if(closest_lanelet.id() == 2719) continue;
            if(closest_lanelet.id() == 2710) continue;
            if(closest_lanelet.id() == 2639) continue;
            if(closest_lanelet.id() == 2640) continue;
            if(closest_lanelet.id() == 2634) continue;
            if(closest_lanelet.id() == 2631) continue;
            if(closest_lanelet.id() == 2628) continue;
            if(closest_lanelet.id() == 2896) continue;
            if(closest_lanelet.id() == 2895) continue;
            if(closest_lanelet.id() == 2894) continue;
            if(closest_lanelet.id() == 2922) continue;
            if(closest_lanelet.id() == 2910) continue;
            if(closest_lanelet.id() == 2952) continue;

            first_filtered_PA.poses.push_back(pose);
            first_filtered_PA_score.push_back(PA_score_[i]);
        }

        std::cout << "first_filtered_PA_score : " << first_filtered_PA_score.size() << std::endl;
        std::cout << "first_filtered_PA :" << first_filtered_PA.poses.size() << std::endl;

        PA_ = first_filtered_PA;
        PA_score_ = first_filtered_PA_score;

        // for(size_t i = 0; i < PA_.poses.size() - 1; ++i)
        // {
        //     double min_dist = 99999.999;
        //     bool is_same_location = false;
        //     int k_index = 0;

        //     for(size_t k = i + 1; k < PA_.poses.size(); ++k)
        //     {
        //         double dist = std::sqrt(std::pow(PA_.poses[k].position.x - PA_.poses[i].position.x, 2) +  std::pow(PA_.poses[k].position.y - PA_.poses[i].position.y, 2))
            
        //         if(dist < min_dist)
        //         {
        //             is_same_location = true;
        //             k_index = k;
        //             break;
        //         }
        //     }

        //     if(min_dist < 1) continue;

        //     if(PA_score_[i_index] < PA_score_[k_index])
        //     {
        //         second_filtered_PA.push_back(PA_score_[pose]);

        //     }


        // }

        // std::cout << "second_filtered_PA : " << second_filtered_PA.size() << std::endl;
        // std::cout << "second_filtered_PA_score :" << second_filtered_PA_score.poses.size() << std::endl;

        // PA_ = second_filtered_PA;
        // PA_score_ = second_filtered_PA_score;
    }

    void MissionManagerNode::addFinalPA()
    {
        geometry_msgs::msg::Pose temp_goal_pose;

        // get center lanelet my_pose
        geometry_msgs::msg::Pose goal_pose = PA_.poses[PA_.poses.size() - 1];
        // new_PA.poses.push_back(start_pose);

        lanelet::Lanelet closest_lanelet;
        const geometry_msgs::msg::Pose const_goal_pose(goal_pose);
        lanelet::utils::query::getClosestLanelet(road_lanelets_, const_goal_pose, &closest_lanelet);

        lanelet::ConstLanelet next_lanelet = routing_graph_ptr_->following(closest_lanelet)[0];
        lanelet::ConstLanelet next_next_lanelet = routing_graph_ptr_->following(next_lanelet)[0];

        const lanelet::ConstLineString2d lanelet_2d = next_next_lanelet.centerline2d();

        for (const auto & point : lanelet_2d) 
        {
            temp_goal_pose.position.x = point.x();
            temp_goal_pose.position.y = point.y();
            temp_goal_pose.position.z = 0.0;
            temp_goal_pose.orientation.x = 0.0;
            temp_goal_pose.orientation.y = 0.0;
            temp_goal_pose.orientation.z = 0.0;
            temp_goal_pose.orientation.w = 0.0;
        }
        
        const double lane_yaw = lanelet::utils::getLaneletAngle(next_next_lanelet, temp_goal_pose.position);
        temp_goal_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(lane_yaw);

        PA_.poses.push_back(temp_goal_pose);
    }

    void MissionManagerNode::publishStage2MissionMsg()
    {
        mission_manager_msgs::msg::CheckpointsWithLaneId stage2_mission_msg;

        for(size_t i = 1; i < PA_.poses.size(); ++i)
        {
            std::cout << "(" << PA_.poses[i].position.x << " , " << PA_.poses[i].position.y << ")" << std::endl;
            stage2_mission_msg.checkpoints.poses.push_back(PA_.poses[i]);

            lanelet::Lanelet closest_lanelet;
            const geometry_msgs::msg::Pose const_temp_pose(PA_.poses[i]);
            lanelet::utils::query::getClosestLanelet(road_lanelets_, const_temp_pose, &closest_lanelet);
            stage2_mission_msg.checkpoints_laneid.push_back(closest_lanelet.id());

            lanelet::ConstLanelets neighbor_lanelets = lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, closest_lanelet);
            mission_manager_msgs::msg::NeighborLaneletsInfo neighbor_info;
            for (const auto & llt : neighbor_lanelets)
                neighbor_info.lanelets.push_back(llt.id());
            stage2_mission_msg.neighbor_lanelets.push_back(neighbor_info);
            stage2_mission_msg.scores.push_back(PA_score_.at(i));
        }
        pub_mission_manager_msgs_->publish(stage2_mission_msg);
        pub_checkpoint_->publish(PA_);
        std::cout << "stage 2 mission checkpoints publish ! "<< std::endl;
    }

    lanelet::ConstLanelet MissionManagerNode::getNeighborLanelet(lanelet::Lanelet current_lane)
    {
        const auto & left_lane = routing_graph_ptr_->left(current_lane);
        const auto & right_lane = routing_graph_ptr_->right(current_lane);
        
        if(left_lane && right_lane) // two result
        {
            if(make_item_lane_priority_ == "left")
                return left_lane.get();
            
            if(make_item_lane_priority_ == "right")
                return right_lane.get(); 
        }

        else if(left_lane && !right_lane) {
            return left_lane.get();
        }

        else if(!left_lane && right_lane) {
            return right_lane.get();
        }

        // std::cout << "return current_lane" << std::endl;
        return current_lane;
    }

    lanelet::ConstLanelet MissionManagerNode::getNeighborCenterLanelet(lanelet::Lanelet current_lane)
    {
        const auto & left_lane = routing_graph_ptr_->left(current_lane);
        const auto & right_lane = routing_graph_ptr_->right(current_lane);
        
        if(left_lane && right_lane) // two result
        {
            return current_lane;
        }

        else if(left_lane && !right_lane) {
            return left_lane.get();
        }

        else if(!left_lane && right_lane) {
            return right_lane.get();
        }

        // std::cout << "return current_lane" << std::endl;
        return current_lane;
    }

    int MissionManagerNode::calcClosestLaneletsNum(lanelet::ConstLanelets lanelets, const geometry_msgs::msg::Pose & search_pose)
    {
        int count = 0;

        lanelet::BasicPoint2d search_point(search_pose.position.x, search_pose.position.y);

        for (const auto & llt : lanelets) {
            double distance = boost::geometry::comparable_distance(llt.polygon2d().basicPolygon(), search_point);

            if(fabs(distance) < 0.1)
            {
                count++;
            }
        }
        // std::cout << std::endl;

        return count;
    }

    void MissionManagerNode::mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg)
    {
        route_handler_.setMap(*msg);
        lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
        lanelet::utils::conversion::fromBinMsg(
            *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
        lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
        road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
        shoulder_lanelets_ = lanelet::utils::query::shoulderLanelets(all_lanelets);
        is_graph_ready_ = true;

        // for (const auto & llt : road_lanelets_)
        // {
        //     total_road_lanelets_id_.push_back(llt.id());
        //     std::cout << llt.id() << " ";
        // }            
        // std::cout << std::endl;
        
        std::cout << "success get vector map !!" << std::endl;
    }

    void MissionManagerNode::visualizeCheckPoint(geometry_msgs::msg::PoseArray PA)
    {
        if(mission_name_ == "stage2")
        {
            visualization_msgs::msg::MarkerArray MA;
        
            int id_ = 0;

            for(size_t i = 0; i < PA.poses.size(); ++i)
            {
                visualization_msgs::msg::Marker M;
                M.header.frame_id = "map";
                M.header.stamp = this->now();
                M.ns = "checkpoint";
                M.id = id_++;
                M.type = visualization_msgs::msg::Marker::ARROW;
                M.action = visualization_msgs::msg::Marker::ADD;
                M.pose = PA.poses[i];
                M.scale.x = 2.0;
                M.scale.y = 2.0;
                M.scale.z = 2.0;
                M.color.a = 1.0;
                M.color.r = 1.0;
                M.color.g = 0.0;
                M.color.b = 0.0;

                M.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                M.text = std::to_string(id_);

                MA.markers.push_back(M);
            }

            pub_checkpoint_first_debug_->publish(MA);
        }
        
        if(mission_name_ == "stage1")
        {
            visualization_msgs::msg::MarkerArray first_MA, second_MA;
        
            int first_id = 1;
            int second_id = 1;

            for(size_t i = 0; i < PA.poses.size(); ++i)
            {
                if(i <= start_pose_index_)
                {
                    visualization_msgs::msg::Marker M;
                    M.header.frame_id = "map";
                    M.header.stamp = this->now();
                    M.ns = "checkpoint";
                    M.id = first_id;
                    M.type = visualization_msgs::msg::Marker::ARROW;
                    M.action = visualization_msgs::msg::Marker::ADD;
                    M.pose = PA.poses[i];
                    M.scale.x = 2.0;
                    M.scale.y = 2.0;
                    M.scale.z = 2.0;
                    M.color.a = 1.0;
                    M.color.r = 1.0;
                    M.color.g = 0.0;
                    M.color.b = 0.0;

                    M.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                    M.text = std::to_string(first_id++);

                    first_MA.markers.push_back(M);
                }

                if(i > start_pose_index_)
                {
                    visualization_msgs::msg::Marker M;
                    M.header.frame_id = "map";
                    M.header.stamp = this->now();
                    M.ns = "checkpoint";
                    M.id = second_id;
                    M.type = visualization_msgs::msg::Marker::ARROW;
                    M.action = visualization_msgs::msg::Marker::ADD;
                    M.pose = PA.poses[i];
                    M.scale.x = 2.0;
                    M.scale.y = 2.0;
                    M.scale.z = 2.0;
                    M.color.a = 1.0;
                    M.color.r = 0.0;
                    M.color.g = 1.0;
                    M.color.b = 0.0;

                    M.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                    M.text = std::to_string(second_id++);

                    second_MA.markers.push_back(M);
                }
            }

            pub_checkpoint_first_debug_->publish(first_MA);
            pub_checkpoint_second_debug_->publish(second_MA);
        }
    }

    void MissionManagerNode::visualizeCheckPointOriginal(geometry_msgs::msg::PoseArray PA)
    {
        visualization_msgs::msg::MarkerArray MA;
        
        int id_ = 0;

        for(size_t i = 0; i < PA.poses.size(); ++i)
        {
            visualization_msgs::msg::Marker M;
            M.header.frame_id = "map";
            M.header.stamp = this->now();
            M.ns = "checkpoint";
            M.id = id_++;
            M.type = visualization_msgs::msg::Marker::CUBE;
            M.action = visualization_msgs::msg::Marker::ADD;
            M.pose = PA.poses[i];
            M.scale.x = 1.0;
            M.scale.y = 1.0;
            M.scale.z = 1.0;
            M.color.a = 1.0;
            M.color.r = 0.0;
            M.color.g = 1.0;
            M.color.b = 1.0;

            MA.markers.push_back(M);
        }

        pub_checkpoint_original_debug_->publish(MA);
    }

    void MissionManagerNode::visualizeitemList(std::vector<std::pair<geometry_msgs::msg::Pose, int>> item_list)
    {
        visualization_msgs::msg::MarkerArray MA;
        
        int id_ = 0;

        for(size_t i = 0; i < item_list.size(); ++i)
        {
            visualization_msgs::msg::Marker M;
            M.header.frame_id = "map";
            M.header.stamp = this->now();
            M.ns = "item";
            M.id = id_++;
            M.type = visualization_msgs::msg::Marker::CUBE;
            M.action =  visualization_msgs::msg::Marker::ADD;
            M.pose.position.x = item_list[i].first.position.x;
            M.pose.position.y = item_list[i].first.position.y;
            M.pose.position.z = item_list[i].first.position.z;
            M.scale.x = 1.0;
            M.scale.y = 1.0;
            M.scale.z = 1.0;
            M.color.a = 1.0;

            // M.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            // M.text = std::to_string(id_);


            if(item_list[i].second == 1)
            {
                M.color.r = 1.0;
                M.color.g = 0.0;
                M.color.b = 0.0;
            }
            else if(item_list[i].second == 2)
            {
                M.color.r = 0.0;
                M.color.g = 1.0;
                M.color.b = 0.0;
            }
            else
            {
                M.color.r = 0.0;
                M.color.g = 0.0;
                M.color.b = 1.0;
            }

            if(item_list[i].second != 1)
                MA.markers.push_back(M);
        }

        pub_item_list_debug_->publish(MA);
    }
    
    bool MissionManagerNode::getTransform(
    const std::string & target_frame, const std::string & source_frame,
    const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr)
        {
        if (target_frame == source_frame) {
            transform_stamped_ptr->header.stamp = this->get_clock()->now();
            transform_stamped_ptr->header.frame_id = target_frame;
            transform_stamped_ptr->child_frame_id = source_frame;
            transform_stamped_ptr->transform.translation.x = 0.0;
            transform_stamped_ptr->transform.translation.y = 0.0;
            transform_stamped_ptr->transform.translation.z = 0.0;
            transform_stamped_ptr->transform.rotation.x = 0.0;
            transform_stamped_ptr->transform.rotation.y = 0.0;
            transform_stamped_ptr->transform.rotation.z = 0.0;
            transform_stamped_ptr->transform.rotation.w = 1.0;
            return true;
        }

        try {
            *transform_stamped_ptr =
            tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        } catch (tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            RCLCPP_ERROR(
            this->get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

            transform_stamped_ptr->header.stamp = this->get_clock()->now();
            transform_stamped_ptr->header.frame_id = target_frame;
            transform_stamped_ptr->child_frame_id = source_frame;
            transform_stamped_ptr->transform.translation.x = 0.0;
            transform_stamped_ptr->transform.translation.y = 0.0;
            transform_stamped_ptr->transform.translation.z = 0.0;
            transform_stamped_ptr->transform.rotation.x = 0.0;
            transform_stamped_ptr->transform.rotation.y = 0.0;
            transform_stamped_ptr->transform.rotation.z = 0.0;
            transform_stamped_ptr->transform.rotation.w = 1.0;
            return false;
        }
        return true;
    }

    bool MissionManagerNode::calcObjectPolygon(const autoware_auto_perception_msgs::msg::TrackedObject & object, tier4_autoware_utils::Polygon2d * object_polygon)
    {
        const double obj_x = object.kinematics.pose_with_covariance.pose.position.x;
        const double obj_y = object.kinematics.pose_with_covariance.pose.position.y;
        if (object.shape.type == 0) {
            const double len_x = object.shape.dimensions.x + 1.0;
            const double len_y = object.shape.dimensions.y + 1.0;

            tf2::Transform tf_map2obj;
            tf2::fromMsg(object.kinematics.pose_with_covariance.pose, tf_map2obj);
    
            // set vertices at map coordinate
            tf2::Vector3 p1_map, p2_map, p3_map, p4_map;

            p1_map.setX(len_x / 2);
            p1_map.setY(len_y / 2);
            p1_map.setZ(0.0);
            p1_map.setW(1.0);

            p2_map.setX(-len_x / 2);
            p2_map.setY(len_y / 2);
            p2_map.setZ(0.0);
            p2_map.setW(1.0);

            p3_map.setX(-len_x / 2);
            p3_map.setY(-len_y / 2);
            p3_map.setZ(0.0);
            p3_map.setW(1.0);

            p4_map.setX(len_x / 2);
            p4_map.setY(-len_y / 2);
            p4_map.setZ(0.0);
            p4_map.setW(1.0);

            // transform vertices from map coordinate to object coordinate
            tf2::Vector3 p1_obj, p2_obj, p3_obj, p4_obj;

            p1_obj = tf_map2obj * p1_map;
            p2_obj = tf_map2obj * p2_map;
            p3_obj = tf_map2obj * p3_map;
            p4_obj = tf_map2obj * p4_map;

            object_polygon->outer().emplace_back(p1_obj.x(), p1_obj.y());
            object_polygon->outer().emplace_back(p2_obj.x(), p2_obj.y());
            object_polygon->outer().emplace_back(p3_obj.x(), p3_obj.y());
            object_polygon->outer().emplace_back(p4_obj.x(), p4_obj.y());

        } else if (object.shape.type == 1) {
            const size_t N = 20;
            const double r = object.shape.dimensions.x / 2;
            for (size_t i = 0; i < N; ++i) {
            object_polygon->outer().emplace_back(
                obj_x + r * std::cos(2.0 * M_PI / N * i), obj_y + r * std::sin(2.0 * M_PI / N * i));
            }
        } else if (object.shape.type == 2) {
            tf2::Transform tf_map2obj;
            tf2::fromMsg(object.kinematics.pose_with_covariance.pose, tf_map2obj);
            const auto obj_points = object.shape.footprint.points;
            for (const auto & obj_point : obj_points) {
            tf2::Vector3 obj(obj_point.x, obj_point.y, obj_point.z);
            tf2::Vector3 tf_obj = tf_map2obj * obj;
            object_polygon->outer().emplace_back(tf_obj.x(), tf_obj.y());
            }
        } else {
            std::cout << "mission manager am tracked object shape unknown !!" << std::endl;
            return false;
        }
        
        // object_polygon->outer().push_back(object_polygon->outer().front());

        return true;
    }

    pcl::PointCloud<pcl::PointXYZ> MissionManagerNode::applyVoxelGridFilter(const pcl::PointCloud<pcl::PointXYZ> & input_points)
    {
        pcl::VoxelGrid<pcl::PointXYZ> filter;
        filter.setInputCloud(pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(input_points));
        filter.setLeafSize(0.5f, 0.5f, 0.3f);

        pcl::PointCloud<pcl::PointXYZ> output_points;
        filter.filter(output_points);

        return output_points;
    }

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mission_manager::MissionManagerNode>());
  rclcpp::shutdown();

  return 0;
}