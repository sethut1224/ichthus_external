/*  ===========================================================================
    Copyright 2022. The ICHTHUS Project. All Rights Reserved.
    Hyewon Bang (hwbang8150@gmail.com) and Jamin Lee (xoz1206@gmail.com), Youngjoon Han (young@ssu.ac.kr).
    Vision Systems Laboratory, Soongsil University.
    added by ICHTHUS, Hyewon Bang on 20221026
    [Licensed under the MIT License]
    ===========================================================================*/

#ifndef PROJECT_PIXEL_CLOUD_FUSION_H
#define PROJECT_PIXEL_CLOUD_FUSION_H

#define __APP_NAME__ "pixel_cloud_fusion"
#include <string>
#include <vector>
#include <map>
#include <chrono>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>


#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <std_msgs/msg/header.hpp>

#include <message_filters/subscriber.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <euclidean_cluster/voxel_grid_based_euclidean_cluster.hpp>
#include <image_projection_based_fusion/utils/geometry.hpp>

#include <shape_estimation/model/bounding_box.hpp>
// #include "object_association_merger/utils/utils.hpp"

#include <boost/geometry.hpp>
#include <multi_object_tracker/utils/utils.hpp>

namespace std{
  template<>
  class hash<cv::Point>{
    public:
      size_t operator()(const cv::Point &pixel_cloud) const
      {
        return hash<std::string>()(std::to_string(pixel_cloud.x)+"|" + std::to_string(pixel_cloud.y));
      }
  };
}//for unordered_map

namespace pcfusion
{
using sensor_msgs::msg::PointCloud2;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::DetectedObjectWithFeature;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using autoware_auto_perception_msgs::msg::DetectedObject;
using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

// template <class Msg>
class PixelCloudFusionNodelet : public rclcpp::Node
{
    public:
        explicit PixelCloudFusionNodelet(const rclcpp::NodeOptions & options);
        
        void connectCb();

        // LidarBasedcallback
        void objectFusionCallback(const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr &left_cam_obj_msg,
        const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr &right_cam_obj_msg,
        const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr &center_point_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &point_cloud_msg
        );

        // SpeedBustcallback
        void SpeedBustcallback(const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr &left_cam_speed_bust_msg,
        const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr &right_cam_speed_bust_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &point_cloud_msg
        );
        
        autoware_auto_perception_msgs::msg::DetectedObjects giveLabel(const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr &cam_obj, const autoware_auto_perception_msgs::msg::DetectedObjects &center_point_obj, std::string name);
        autoware_auto_perception_msgs::msg::DetectedObjects extractObject(const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr &cam_obj, const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pointcloud_msg, std::string name);

        autoware_auto_perception_msgs::msg::DetectedObjects mergeFusionObject(autoware_auto_perception_msgs::msg::DetectedObjects obj1,
                                                                              autoware_auto_perception_msgs::msg::DetectedObjects obj2,
                                                                              autoware_auto_perception_msgs::msg::DetectedObjects obj3,
                                                                              autoware_auto_perception_msgs::msg::DetectedObjects obj4,
                                                                              autoware_auto_perception_msgs::msg::DetectedObjects obj5);
        
        std::vector<pcl::PointCloud<pcl::PointXYZ>> calcObjectEightCord(const autoware_auto_perception_msgs::msg::DetectedObjects &center_point_obj);
        std::vector<cv::Rect> calcObject2DRect(std::vector<pcl::PointCloud<pcl::PointXYZ>> objects_eight_cord, std::string name);
        autoware_auto_perception_msgs::msg::DetectedObjects calcObjectEuclideanIOU(std::vector<cv::Rect> objects_2d_rect, const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr &cam_obj,
              const autoware_auto_perception_msgs::msg::DetectedObjects &unknown_center_point);
        autoware_auto_perception_msgs::msg::DetectedObjects calcObjectCenterPointIOU(std::vector<cv::Rect> objects_2d_rect, const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr &cam_obj, const autoware_auto_perception_msgs::msg::DetectedObjects &unknown_center_point);
        autoware_auto_perception_msgs::msg::DetectedObjects makeEuclideanClsuterAutowareObject(pcl::PointCloud<pcl::PointXYZ> input_cloud);
        void filterUnknownObj(const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr &center_point_msg,  autoware_auto_perception_msgs::msg::DetectedObjects &unknown_center_point, autoware_auto_perception_msgs::msg::DetectedObjects &label_center_point_msg);
        bool getTransform(const std::string & target_frame, const std::string & source_frame, const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr);

        bool estimate(const pcl::PointCloud<pcl::PointXYZ> & cluster,
          autoware_auto_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output);

        bool fitLShape( const pcl::PointCloud<pcl::PointXYZ> & cluster, const float min_angle, const float max_angle,
          autoware_auto_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output);

        float calcClosenessCriterion( const std::vector<float> & C_1, const std::vector<float> & C_2);
        
        // speed_bust
        std::unordered_map<cv::Point, pcl::PointXYZ> filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud , std::string name);        std::vector<pcl::PointCloud<pcl::PointXYZ>> inRoiPointCloud(const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr cam_speed_bust_obj,std::unordered_map<cv::Point, pcl::PointXYZ> &filtered_cloud);
        geometry_msgs::msg::PoseArray makeEuclideanSpeedBustAutowareObject(std::vector<pcl::PointCloud<pcl::PointXYZ>> input_cloud);
        geometry_msgs::msg::PoseArray mergeSpeedBustPose(geometry_msgs::msg::PoseArray left_speed_bust_objects, geometry_msgs::msg::PoseArray right_speed_bust_objects);
    protected:
        // void cameraInfoCallback(
        //   const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_camera_info_msg,
        //   const std::size_t camera_id);

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        // Lidar base Sync
        typedef message_filters::sync_policies::ApproximateTime<DetectedObjectsWithFeature, DetectedObjectsWithFeature, DetectedObjects, PointCloud2> LidarBaseSyncPolicy;
        typedef message_filters::Synchronizer<LidarBaseSyncPolicy> LidarBaseSync;
        typename std::shared_ptr<LidarBaseSync> lidar_base_sync_ptr_;

        // Speed bust base Sync
        typedef message_filters::sync_policies::ApproximateTime<DetectedObjectsWithFeature, DetectedObjectsWithFeature, PointCloud2> SpeedBustSyncPolicy;
        typedef message_filters::Synchronizer<SpeedBustSyncPolicy> SpeedBustSync;
        typename std::shared_ptr<SpeedBustSync> speed_bust_sync_ptr_;
    private:
        // sub
        message_filters::Subscriber<tier4_perception_msgs::msg::DetectedObjectsWithFeature> left_cam_obj_sub_;
        message_filters::Subscriber<tier4_perception_msgs::msg::DetectedObjectsWithFeature> right_cam_obj_sub_;
        message_filters::Subscriber<tier4_perception_msgs::msg::DetectedObjectsWithFeature> left_cam_speed_bust_sub_;
        message_filters::Subscriber<tier4_perception_msgs::msg::DetectedObjectsWithFeature> right_cam_speed_bust_sub_;
        message_filters::Subscriber<autoware_auto_perception_msgs::msg::DetectedObjects> centerpoint_sub_;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_cloud_sub_;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> merged_point_cloud_sub_;

        // pub
        rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr centerpoint_obj_pub_;
        rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr fusion_obj_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr speed_bust_poses_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_euclidean_objects_debug_;
        
        std::vector<std::string> labels_;
        int image_width;
        int image_height;
        double fx_, fy_, cx_, cy_;
        std::shared_ptr<euclidean_cluster::VoxelGridBasedEuclideanCluster> cluster_;
        std::shared_ptr<euclidean_cluster::VoxelGridBasedEuclideanCluster> normal_cluster_;
};
}

#endif //PROJECT_PIXEL_CLOUD_FUSION_H
