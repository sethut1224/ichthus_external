/*  ===========================================================================
    Copyright 2022. The ICHTHUS Project. All Rights Reserved.
    Jaemin Lee (xoz1206@gmail.com),
    Youngjun Han (young@ssu.ac.kr)
    Vision System Laboratory, Soongsil University.
    [Licensed under the MIT License]  
    ===========================================================================*/
    
#ifndef __MISSION_MANAGER_AM__NODE_HPP_
#define __MISSION_MANAGER_AM__NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <kiapi_msgs/msg/mission_list_stage1.hpp>
#include <kiapi_msgs/msg/mission_list_stage2.hpp>
#include <kiapi_msgs/msg/mission_route_data.hpp>
#include <kiapi_msgs/msg/item_data.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/utility/Utilities.h>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>

#include <autoware_auto_perception_msgs/msg/detected_object.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>

#include <route_handler/route_handler.hpp>

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <behavior_path_planner/path_utilities.hpp>

#include <mission_manager_msgs/msg/checkpoints_with_lane_id.hpp>
#include <mission_manager_msgs/msg/neighbor_lanelets_info.hpp>
#include <autoware_auto_system_msgs/msg/emergency_state.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <tier4_autoware_utils/ros/transform_listener.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>

#include <pcl/common/transforms.h>
// #include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl/filters/voxel_grid.h>
// #include <pcl/search/pcl_search.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <euclidean_cluster/voxel_grid_based_euclidean_cluster.hpp>

#include <pcl/common/common.h>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <scene_module/crosswalk/manager.hpp>

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>

namespace mission_manager{
    class MissionManagerNode : public rclcpp::Node
    {
        public:
            MissionManagerNode();
            ~MissionManagerNode();

            void stage1MissionListsCallback(const kiapi_msgs::msg::MissionListStage1::ConstSharedPtr msg);
            void poseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
            void visualizeCheckPoint(geometry_msgs::msg::PoseArray PA);
            void visualizeCheckPointOriginal(geometry_msgs::msg::PoseArray PA);
            void visualizeitemList(std::vector<std::pair<geometry_msgs::msg::Pose, int>> item_list);
            void checkpointPathCallback(const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg);
            void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg);
            autoware_auto_planning_msgs::msg::PathWithLaneId concatenatePaths(std::vector<autoware_auto_planning_msgs::msg::PathWithLaneId> checkpoints_paths);
            bool checkisSamePrevPath(const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg);
            double calcDistance(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);
            int calcClosestLaneletsNum(lanelet::ConstLanelets lanelets, const geometry_msgs::msg::Pose & search_pose);
            void stage2MissionListsCallback(const kiapi_msgs::msg::MissionListStage2::ConstSharedPtr msg);
            void successArriveStartPoseCallback(const std_msgs::msg::UInt8::ConstSharedPtr msg);
            lanelet::ConstLanelet getNeighborLanelet(lanelet::Lanelet current_lane);
            lanelet::ConstLanelet getNeighborCenterLanelet(lanelet::Lanelet current_lane);
            void sortPA();
            void filteringPA();
            void publishStage2MissionMsg();
            void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
            void addFinalPA();
            void add3thLaneCheckpoint();
            lanelet::ConstLineString2d resample(lanelet::ConstLineString2d lanelet_boundary, double resolution);
            geometry_msgs::msg::Pose calcClosestPose(lanelet::ConstLineString2d points, geometry_msgs::msg::Pose critic_point);
            std::vector<lanelet::ConstPoint2d> calcBoundaryPoints(lanelet::ConstLineString2d start_lanelet, lanelet::ConstLineString2d end_lanelet, geometry_msgs::msg::Pose start_pose, geometry_msgs::msg::Pose end_pose, bool lane_is_same);
            void visualizeDetectionArea(std::vector<std::vector<cv::Point2f>> convex_hull_vec);
            void trackingObjectsCallback(const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr msg);
            bool getTransform(const std::string & target_frame, const std::string & source_frame,
                const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr);
            bool calcObjectPolygon(const autoware_auto_perception_msgs::msg::TrackedObject & object, tier4_autoware_utils::Polygon2d * object_polygon);
            pcl::PointCloud<pcl::PointXYZ> applyVoxelGridFilter(const pcl::PointCloud<pcl::PointXYZ> & input_points);
            void speedBustCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr msg);
            void trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
            autoware_auto_planning_msgs::msg::Trajectory resampleTrajectory(const autoware_auto_planning_msgs::msg::Trajectory & trajectory, const double interval);

            void PathCallback(const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg);

            std::vector<lanelet::ConstLanelet> getCrosswalksOnPath(
              const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
              const lanelet::LaneletMapPtr lanelet_map,
              const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs);

            bool isInPolygon(const std::vector<geometry_msgs::msg::Point> & polygon, const geometry_msgs::msg::Point & point);
            bool isInPolygon(const std::vector<tf2::Vector3> & polygon, const tf2::Vector3 & point);

        private:
            rclcpp::Subscription<kiapi_msgs::msg::MissionListStage1>::SharedPtr sub_stage1_mission_list_;
            rclcpp::Subscription<kiapi_msgs::msg::MissionListStage2>::SharedPtr sub_stage2_mission_list_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_my_pose_;
            rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_my_pose_rviz_;
            rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_subscriber_;
            rclcpp::Subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>::SharedPtr sub_checkpoints_path_;
            rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_arrive_start_pose_success_;
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_pointcloud_;
            rclcpp::Subscription<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr sub_tracking_objects_;
            rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_speed_bust_poses_;
            rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;
            rclcpp::Subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>::SharedPtr sub_behavior_path_;

            rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_checkpoint_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_checkpoint_first_debug_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_checkpoint_second_debug_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_checkpoint_original_debug_;
            rclcpp::Publisher<autoware_auto_planning_msgs::msg::PathWithLaneId>::SharedPtr pub_total_path_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_item_list_debug_;
            rclcpp::Publisher<mission_manager_msgs::msg::CheckpointsWithLaneId>::SharedPtr pub_mission_manager_msgs_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_start_checkpoint_arrive_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_goal_checkpoint_arrive_;
            rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_mission_difficulty_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_estop_flag_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_detection_area_debug_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_detection_area_pointcloud_debug_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_euclidean_objects_debug_;
            rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr pub_fake_objects_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_speed_bust_debug_;
            rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimit>::SharedPtr pub_velocity_limit_;

            std::shared_ptr<euclidean_cluster::VoxelGridBasedEuclideanCluster> cluster_;
            
            geometry_msgs::msg::Pose my_pose_;
            bool isprocessed_;
            bool getCurrentPose_;

            bool is_graph_ready_;
            bool pose_ok_;
            
            lanelet::LaneletMapPtr lanelet_map_ptr_;
            lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
            lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
            lanelet::ConstLanelets road_lanelets_;
            lanelet::ConstLanelets shoulder_lanelets_;
            route_handler::RouteHandler route_handler_;
            
            std::vector<std::pair<double, double>> temp_gps_checkpoints_;

            std::vector<autoware_auto_planning_msgs::msg::PathWithLaneId> checkpoints_paths_;
            geometry_msgs::msg::PoseArray PA_;

            std::vector<lanelet::ConstLanelets> neighbor_lanelets_;
            
            std::vector<geometry_msgs::msg::PoseArray> total_route_lists_;
            size_t now_checkpoint_index_;
            size_t total_checkpoints_size_;
            size_t now_goal_index_;

            bool is_published;
            bool first_checkpoint_publish_;
            autoware_auto_planning_msgs::msg::PathWithLaneId total_path_;

            bool is_final_path_publish_; 
            
            autoware_auto_planning_msgs::msg::PathWithLaneId latest_path_drivable_area_;
            std::string mission_name_;
            int mission_difficulty_;

            std::vector<std::pair<geometry_msgs::msg::Pose, int>> item_lists_;

            mission_manager_msgs::msg::CheckpointsWithLaneId checkpoints_info_msg_total_;
            mission_manager_msgs::msg::CheckpointsWithLaneId checkpoints_info_msg_first_;
            mission_manager_msgs::msg::CheckpointsWithLaneId checkpoints_info_msg_second_;
            mission_manager_msgs::msg::CheckpointsWithLaneId checkpoints_info_msg_last_;

            size_t start_pose_index_;
            size_t goal_pose_index_;
            size_t last_replan_index_;

            bool start_pose_arrive_;
            bool goal_pose_arrive_;
            bool last_replan_pose_arrive_;

            bool is_find_start_index_;
            bool is_find_goal_index_;

            std::string make_item_lane_priority_;

            autoware_auto_perception_msgs::msg::TrackedObjects tracked_objects_;
            bool get_tracked_objects_;

            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf_listener_;

            std::vector<int> PA_score_;
            std::vector<std::pair<geometry_msgs::msg::Pose, int>> speed_bust_point_vec_;

            autoware_auto_planning_msgs::msg::Trajectory current_trajectory_;

            geometry_msgs::msg::PoseArray original_minus_PA_;
            
            int time_count_;

            bool is_get_pointcloud_;

            sensor_msgs::msg::PointCloud2 origin_pcd_;

            double now_velocity_limit_;

    };  
}

#endif  // __MISSION_MANAGER_AM__NODE_HPP_