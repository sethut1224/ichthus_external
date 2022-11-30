/*  ===========================================================================
    Copyright 2022. The ICHTHUS Project. All Rights Reserved.
    Hyewon Bang (hwbang0815@naver.com) and Jamin Lee (xoz1206@gmail.com), Youngjoon Han (young@ssu.ac.kr).
    Vision Systems Laboratory, Soongsil University.
    added by ICHTHUS, Hyewon Bang on 20221026
    [Licensed under the MIT License]
    ===========================================================================*/

#include "pixel_cloud_fusion/nodelet.hpp"

namespace pcfusion
{
    PixelCloudFusionNodelet::PixelCloudFusionNodelet(const rclcpp::NodeOptions &options)
        : Node("pixel_cloud_fusion", options),
          tf_buffer_(get_clock()),
          tf_listener_(tf_buffer_)
    {
        using std::placeholders::_1;
        using std::placeholders::_2;
        using std::placeholders::_3;
        using std::placeholders::_4;


        // camera paramter
        image_width = declare_parameter("image_width", 960);
        image_height = declare_parameter("image_height", 640);
        fx_ = declare_parameter("fx", 1005.196); 
        fy_ = declare_parameter("fy", 1074.321); 
        cx_ = declare_parameter("cx", 492.249);  
        cy_ = declare_parameter("cy", 326.746);
        
        // speed_bust euclidean cluster parameter
        bool use_height = declare_parameter("use_height", true);
        int min_cluster_size = declare_parameter("min_cluster_size",0.005);
        int max_cluster_size = declare_parameter("max_cluster_size", 500);
        float tolerance = declare_parameter("tolerance", 0.5);
        float voxel_leaf_size = declare_parameter("voxel_leaf_size",0.05);
        int min_points_number_per_voxel = declare_parameter("min_points_number_per_voxel", 1);  

        // normal_object euclidean cluster parameter
        // bool normal_use_height = declare_parameter("normal_use_height", true);
        // int normal_min_cluster_size = declare_parameter("normal_min_cluster_size", 0.5);
        // int normal_max_cluster_size = declare_parameter("normal_max_cluster_size", 1000);
        // float normal_tolerance = declare_parameter("normal_tolerance", 1.0);
        // float normal_voxel_leaf_size = declare_parameter("normal_voxel_leaf_size", 0.03);
        // int normal_min_points_number_per_voxel = declare_parameter("normal_min_points_number_per_voxel", 1);  

        // sub speed_bust roi box
        left_cam_speed_bust_sub_.subscribe(this, "/left_rois1",     rmw_qos_profile_sensor_data);
        right_cam_speed_bust_sub_.subscribe(this, "/right_rois1",   rmw_qos_profile_sensor_data);

        // sub normal_object roi box
        // left_cam_obj_sub_.subscribe(this, "/left_rois0",            rmw_qos_profile_sensor_data);
        // right_cam_obj_sub_.subscribe(this, "/right_rois0",          rmw_qos_profile_sensor_data);

        // sub points cloud
        // centerpoint_sub_.subscribe(this, "/lidar_front/detected_objects", rmw_qos_profile_sensor_data);
        // merged_point_cloud_sub_.subscribe(this, "/merged_cloud", rmw_qos_profile_sensor_data);
        point_cloud_sub_.subscribe(this, "/no_ground_pointcloud", rmw_qos_profile_sensor_data);

        // pub debugging msg
        // centerpoint_obj_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>("/fusion/centerpoint_object", 1);
        // pub_euclidean_objects_debug_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/euclidean_objects_debug", 1);

        // pub normal_object fusion msg
        // fusion_obj_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>("/fusion/fusion_object", 1);

        //pub speed_bust fusion msg
        speed_bust_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/fusion/speed_bust_poses", 1);

        // lidar_base_sync_ptr_.reset(new LidarBaseSync(LidarBaseSyncPolicy(10), left_cam_obj_sub_, right_cam_obj_sub_, centerpoint_sub_, point_cloud_sub_));
        // lidar_base_sync_ptr_->registerCallback(std::bind(&PixelCloudFusionNodelet::objectFusionCallback, this, _1, _2, _3, _4));
        speed_bust_sync_ptr_.reset(new SpeedBustSync(SpeedBustSyncPolicy(10), left_cam_speed_bust_sub_, right_cam_speed_bust_sub_, point_cloud_sub_));
        speed_bust_sync_ptr_->registerCallback(std::bind(&PixelCloudFusionNodelet::SpeedBustcallback, this, _1, _2, _3));

        // normal_cluster_ = std::make_shared<euclidean_cluster::VoxelGridBasedEuclideanCluster>(normal_use_height, normal_min_cluster_size, normal_max_cluster_size, normal_tolerance, normal_voxel_leaf_size, normal_min_points_number_per_voxel);
        cluster_ = std::make_shared<euclidean_cluster::VoxelGridBasedEuclideanCluster>(use_height, min_cluster_size, max_cluster_size, tolerance, voxel_leaf_size, min_points_number_per_voxel);

         std::cout << "start pixel cloud fuison !" << std::endl;
    }
    
    // void PixelCloudFusionNodelet::objectFusionCallback(
    //     const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr &left_cam_obj_msg,
    //     const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr &right_cam_obj_msg,
    //     const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr &center_point_msg,
    //     const sensor_msgs::msg::PointCloud2::ConstSharedPtr &point_cloud_msg)
    // {
    //     std::cout << "object fusion callback ! " << std::endl;
    //     autoware_auto_perception_msgs::msg::DetectedObjects unknown_center_point;
    //     autoware_auto_perception_msgs::msg::DetectedObjects label_center_point_msg;

    //     filterUnknownObj(center_point_msg, unknown_center_point, label_center_point_msg);

    //     // 1. [ centerpoint + left yolo ] + [ centerpoint + right yolo ]
    //     // ***** give yolo label to unknown centerpoint object *****
    //     autoware_auto_perception_msgs::msg::DetectedObjects centerpoint_fusion_objects_left = giveLabel(left_cam_obj_msg, unknown_center_point, "cam1");
    //     autoware_auto_perception_msgs::msg::DetectedObjects centerpoint_fusion_objects_right = giveLabel(right_cam_obj_msg, unknown_center_point, "cam0");

    //     // // // 2. [ euclidean + left yolo ] + [ euclidean + right yolo ]
    //     // // // ***** extract yolo object *****
    //     autoware_auto_perception_msgs::msg::DetectedObjects euclidean_fusion_objects_left = extractObject(left_cam_obj_msg, point_cloud_msg, "cam1");
    //     autoware_auto_perception_msgs::msg::DetectedObjects euclidean_fusion_objects_right = extractObject(right_cam_obj_msg, point_cloud_msg, "cam0");

    //     // 3. merge objects

    //     // std::cout << "center left  :" << centerpoint_fusion_objects_left.objects.size() << std::endl;
    //     // std::cout << "center right :" << centerpoint_fusion_objects_right.objects.size() << std::endl;
    //     // std::cout << "eu left      : " << euclidean_fusion_objects_left.objects.size() << std::endl;
    //     // std::cout << "eu right     : " << euclidean_fusion_objects_right.objects.size() << std::endl;
        
    //     autoware_auto_perception_msgs::msg::DetectedObjects merged_fusion_objects = mergeFusionObject(label_center_point_msg, 
    //         centerpoint_fusion_objects_left, centerpoint_fusion_objects_right, euclidean_fusion_objects_left, euclidean_fusion_objects_right);
 
    //     merged_fusion_objects.header = center_point_msg->header;
    //     centerpoint_obj_pub_->publish(*center_point_msg);
    //     fusion_obj_pub_->publish(merged_fusion_objects);
    // }

    // void PixelCloudFusionNodelet::filterUnknownObj(const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr &center_point_msg, 
    // autoware_auto_perception_msgs::msg::DetectedObjects &unknown_center_point,
    // autoware_auto_perception_msgs::msg::DetectedObjects &label_center_point_msg
    // )
    // {
    //     for(size_t i = 0; i < center_point_msg->objects.size(); i++)
    //     {
    //         // original code
    //         if(center_point_msg->objects[i].classification.front().label == Label::UNKNOWN) unknown_center_point.objects.push_back(center_point_msg->objects[i]);
    //         else label_center_point_msg.objects.push_back(center_point_msg->objects[i]);
    //     }
    // }

    void PixelCloudFusionNodelet::SpeedBustcallback(const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr &left_cam_speed_bust_msg,
        const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr &right_cam_speed_bust_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &point_cloud_msg
        )
    {
        std::cout << "Speed Bust sync callback !!" << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);


        pcl::fromROSMsg(*point_cloud_msg, *in_cloud);
        
        std::unordered_map<cv::Point, pcl::PointXYZ> left_filtered_cloud = filterPointCloud(in_cloud, "cam1");
        std::unordered_map<cv::Point, pcl::PointXYZ> right_filtered_cloud = filterPointCloud(in_cloud, "cam0");
        
        std::vector<pcl::PointCloud<pcl::PointXYZ>> left_speed_bust_roi_points = inRoiPointCloud(left_cam_speed_bust_msg, left_filtered_cloud);
        std::vector<pcl::PointCloud<pcl::PointXYZ>> right_speed_bust_roi_points = inRoiPointCloud(right_cam_speed_bust_msg, right_filtered_cloud);
        geometry_msgs::msg::PoseArray left_speed_bust_objects = makeEuclideanSpeedBustAutowareObject(left_speed_bust_roi_points);
        geometry_msgs::msg::PoseArray right_speed_bust_objects = makeEuclideanSpeedBustAutowareObject(right_speed_bust_roi_points);

        geometry_msgs::msg::PoseArray merged_speed_bust_pose = mergeSpeedBustPose(left_speed_bust_objects, right_speed_bust_objects);

        speed_bust_poses_pub_->publish(merged_speed_bust_pose);
    }

    std::unordered_map<cv::Point, pcl::PointXYZ> PixelCloudFusionNodelet::filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud , std::string name)
    {   
        geometry_msgs::msg::TransformStamped::SharedPtr LiDAR2cam_transform = std::make_shared<geometry_msgs::msg::TransformStamped>();
        getTransform(name, "base_link", LiDAR2cam_transform);
        Eigen::Matrix4f affine_matrix = tf2::transformToEigen(LiDAR2cam_transform->transform).matrix().cast<float>();

        pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_camera_frame(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*in_cloud, *in_cloud_camera_frame, affine_matrix);

        std::vector<pcl::PointXYZ> temp_cloud(in_cloud_camera_frame->points.size());
        std::unordered_map<cv::Point, pcl::PointXYZ> filtered_cloud;
        for(size_t i = 0; i < in_cloud_camera_frame->points.size(); ++i)
        {
            temp_cloud[i] = in_cloud_camera_frame->points[i];
            int u = int(temp_cloud[i].x * fx_ / temp_cloud[i].z + cx_);
            int v = int(temp_cloud[i].y * fy_/ temp_cloud[i].z + cy_);
            if ((u >= 0) && (u < image_width) && (v >= 0) && (v < image_height) && temp_cloud[i].z > 0 && temp_cloud[i].z < 50.0)
            {   
                filtered_cloud.insert(std::pair<cv::Point, pcl::PointXYZ>(cv::Point(u, v), in_cloud->points[i]));
            }

        }
        // std::cout << "unorder size : " << filtered_cloud.size() << std::endl;
        
        return filtered_cloud;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>> PixelCloudFusionNodelet::inRoiPointCloud(const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr cam_speed_bust_obj, std::unordered_map<cv::Point, pcl::PointXYZ> &filtered_cloud)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZ>> speed_bust_objs;
        for(size_t i = 0; i < cam_speed_bust_obj->feature_objects.size(); ++i)
        {
            pcl::PointCloud<pcl::PointXYZ> speed_bust_obj;
            int roi_row = int(cam_speed_bust_obj->feature_objects[i].feature.roi.x_offset);//int(cam_speed_bust_obj->feature_objects[i].feature.roi.x_offset - cam_speed_bust_obj -> feature_objects[i].feature.roi.width / 2);
            int roi_col = int(cam_speed_bust_obj->feature_objects[i].feature.roi.y_offset);//int(cam_speed_bust_obj->feature_objects[i].feature.roi.y_offset - cam_speed_bust_obj -> feature_objects[i].feature.roi.height / 2);
            int roi_width = int(cam_speed_bust_obj->feature_objects[i].feature.roi.width);
            int roi_height = int(cam_speed_bust_obj->feature_objects[i].feature.roi.height);

            for (auto i = filtered_cloud.begin(); i != filtered_cloud.end(); ++i)
            {
                if (roi_row < i->first.x && roi_row +roi_width > i->first.x
                    && roi_col < i->first.y && roi_col + roi_height > i->first.y)
                
                {
                    speed_bust_obj.push_back(i->second);
                }
            }
            speed_bust_objs.push_back(speed_bust_obj);
        }
        return speed_bust_objs;
    }
    
    geometry_msgs::msg::PoseArray PixelCloudFusionNodelet::mergeSpeedBustPose(geometry_msgs::msg::PoseArray left_speed_bust_objects, geometry_msgs::msg::PoseArray right_speed_bust_objects)
    {    
        geometry_msgs::msg::PoseArray merge_pose_array;
        // std::cout<<"6"<<std::endl;
        for (size_t i = 0; i < left_speed_bust_objects.poses.size(); ++i)
        {
            merge_pose_array.poses.push_back(left_speed_bust_objects.poses[i]);
        }
        // std::cout<<"7"<<std::endl;
        for (size_t i = 0; i < right_speed_bust_objects.poses.size(); ++i)
        {
            merge_pose_array.poses.push_back(right_speed_bust_objects.poses[i]);
        }
        // std::cout<<"8"<<std::endl;
        return merge_pose_array;
    }

    geometry_msgs::msg::PoseArray PixelCloudFusionNodelet::makeEuclideanSpeedBustAutowareObject(std::vector<pcl::PointCloud<pcl::PointXYZ>> input_cloud)
    {
        geometry_msgs::msg::PoseArray euclidean_cluster_pose_array;
        for(size_t m = 0; m < input_cloud.size(); ++m)
        {
            geometry_msgs::msg::PoseArray tmp_poses;
            pcl::PointCloud<pcl::PointXYZ> temp_cloud = input_cloud[m];

            //jaemin
            if(temp_cloud.points.size() < 2) continue;

            pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>(temp_cloud));
            // for (size_t i =0; i < temp_cloud.points.size(); ++i)
            // {
            //     pcl::PointXYZ p;
            //     p.x = temp_cloud.points[i].x;
            //     p.y = temp_cloud.points[i].y;
            //     p.z = temp_cloud.points[i].z;
            //     input_cloud_xyz->points.push_back(p);
            // }

            // pcl::PointCloud<pcl::PointXYZ>::Ptr last_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(input_cloud));
            std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
            cluster_->cluster(input_cloud_xyz, clusters);

            for(size_t i = 0; i < clusters.size(); ++i)
            {
                geometry_msgs::msg::Pose pose_output;

                pcl::CentroidPoint<pcl::PointXYZ> centroid;
                for (const auto p : clusters[i].points) {
                    centroid.add(p);
                }
                pcl::PointXYZ centroid_point;
                centroid.get(centroid_point);

                pose_output.position.x = centroid_point.x;
                pose_output.position.y = centroid_point.y;
                pose_output.position.z = centroid_point.z;

                tmp_poses.poses.push_back(pose_output);
            }

            geometry_msgs::msg::Pose filter_pose;

            double min_pose = 10000.0;
            int min_idx = -1;
            for(size_t i = 0; i < tmp_poses.poses.size(); ++i)
            {
                double pose = std::sqrt(std::pow(tmp_poses.poses[i].position.x, 2)+std::pow(tmp_poses.poses[i].position.y, 2)+std::pow(tmp_poses.poses[i].position.z, 2));
                if(min_pose > pose)
                {
                    min_pose = pose;
                    min_idx = i;
                }
            }
            // std::cout<<"5"<<std::endl;
            if(min_idx != -1) euclidean_cluster_pose_array.poses.push_back(tmp_poses.poses[min_idx]);
        }

        return euclidean_cluster_pose_array;
    }

    bool PixelCloudFusionNodelet::getTransform(
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


    // For Normal Object Start
    autoware_auto_perception_msgs::msg::DetectedObjects PixelCloudFusionNodelet::giveLabel(const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr &cam_obj, 
                                                                                            const autoware_auto_perception_msgs::msg::DetectedObjects &center_point_obj,
                                                                                            std::string name)
    {
        autoware_auto_perception_msgs::msg::DetectedObjects centerpoint_fusion_objects;
        
        std::vector<pcl::PointCloud<pcl::PointXYZ>> objects_eight_cord = calcObjectEightCord(center_point_obj);

        std::vector<cv::Rect> objects_2d_rect = calcObject2DRect(objects_eight_cord, name);

        centerpoint_fusion_objects = calcObjectCenterPointIOU(objects_2d_rect, cam_obj, center_point_obj);
        return centerpoint_fusion_objects;
    }

    autoware_auto_perception_msgs::msg::DetectedObjects PixelCloudFusionNodelet::extractObject(const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr &cam_obj, 
                                                                                                const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pointcloud_msg,
                                                                                                std::string name)
    {
        autoware_auto_perception_msgs::msg::DetectedObjects euclidean_fusion_objects;

        pcl::PointCloud<pcl::PointXYZ> input_cloud, filtered_input_cloud;
        pcl::fromROSMsg(*pointcloud_msg, input_cloud);

        for(size_t i = 0; i < input_cloud.points.size(); ++i)
        {
            if(std::fabs(input_cloud.points[i].x) > 40) continue;
            if(std::fabs(input_cloud.points[i].y) > 40) continue;
            
            filtered_input_cloud.points.push_back(input_cloud.points[i]);
        }

        autoware_auto_perception_msgs::msg::DetectedObjects euclidean_cluster_objects;
        euclidean_cluster_objects = makeEuclideanClsuterAutowareObject(filtered_input_cloud);
        std::vector<pcl::PointCloud<pcl::PointXYZ>> objects_eight_cord = calcObjectEightCord(euclidean_cluster_objects);
        std::vector<cv::Rect> objects_2d_rect = calcObject2DRect(objects_eight_cord, name);

        euclidean_fusion_objects = calcObjectEuclideanIOU(objects_2d_rect, cam_obj, euclidean_cluster_objects);
        return euclidean_fusion_objects;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>> PixelCloudFusionNodelet::calcObjectEightCord(const autoware_auto_perception_msgs::msg::DetectedObjects &center_point_obj)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZ>> eight_cords;
        for(size_t i = 0; i < center_point_obj.objects.size(); ++i)
        {
            const double yaw = tier4_autoware_utils::normalizeRadian(tf2::getYaw(center_point_obj.objects[i].kinematics.pose_with_covariance.pose.orientation));
            Eigen::Matrix2d rotation;
            rotation << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
            Eigen::Vector2d offset0, offset1, offset2, offset3;
            offset0 = rotation * Eigen::Vector2d(center_point_obj.objects[i].shape.dimensions.x * 0.5f, center_point_obj.objects[i].shape.dimensions.y * 0.5f);
            offset1 = rotation * Eigen::Vector2d(center_point_obj.objects[i].shape.dimensions.x * 0.5f, -center_point_obj.objects[i].shape.dimensions.y * 0.5f);
            offset2 = rotation * Eigen::Vector2d(-center_point_obj.objects[i].shape.dimensions.x * 0.5f, -center_point_obj.objects[i].shape.dimensions.y * 0.5f);
            offset3 = rotation * Eigen::Vector2d(-center_point_obj.objects[i].shape.dimensions.x * 0.5f, center_point_obj.objects[i].shape.dimensions.y * 0.5f);

            double pos_x = center_point_obj.objects[i].kinematics.pose_with_covariance.pose.position.x;
            double pos_y = center_point_obj.objects[i].kinematics.pose_with_covariance.pose.position.y;
            double pos_z = center_point_obj.objects[i].kinematics.pose_with_covariance.pose.position.z;
            
            pcl::PointXYZ p1_map, p2_map, p3_map, p4_map, p5_map, p6_map, p7_map, p8_map;

            pcl::PointCloud<pcl::PointXYZ> eight_coord;

            double box_height = center_point_obj.objects[i].shape.dimensions.z;
            double half_height = box_height / 2;

            p1_map.x = (pos_x  + offset0.x());
            p1_map.y = (pos_y  + offset0.y());
            p1_map.z = (pos_z  + half_height);
            eight_coord.points.push_back(p1_map);

            p2_map.x = (pos_x  + offset0.x());
            p2_map.y = (pos_y  + offset0.y());
            p2_map.z = (pos_z  - half_height);
            eight_coord.points.push_back(p2_map);

            p3_map.x = (pos_x  + offset1.x());
            p3_map.y = (pos_y  + offset1.y());
            p3_map.z = (pos_z  + half_height);
            eight_coord.points.push_back(p3_map);

            p4_map.x = (pos_x  + offset1.x());
            p4_map.y = (pos_y  + offset1.y());
            p4_map.z = (pos_z  - half_height);
            eight_coord.points.push_back(p4_map);

            p5_map.x = (pos_x  + offset2.x());
            p5_map.y = (pos_y  + offset2.y());
            p5_map.z = (pos_z  + half_height);
            eight_coord.points.push_back(p5_map);

            p6_map.x = (pos_x  + offset2.x());
            p6_map.y = (pos_y  + offset2.y());
            p6_map.z = (pos_z  - half_height);
            eight_coord.points.push_back(p6_map);

            p7_map.x = (pos_x  + offset3.x());
            p7_map.y = (pos_y  + offset3.y());
            p7_map.z = (pos_z  + half_height);
            eight_coord.points.push_back(p7_map);

            p8_map.x = (pos_x  + offset3.x());
            p8_map.y = (pos_y  + offset3.y());
            p8_map.z = (pos_z  - half_height);
            eight_coord.points.push_back(p8_map);

            eight_cords.emplace_back(eight_coord);
        }

        return eight_cords;
    }

    std::vector<cv::Rect> PixelCloudFusionNodelet::calcObject2DRect(std::vector<pcl::PointCloud<pcl::PointXYZ>> objects_eight_cord, std::string name)
    {
        std::vector<cv::Rect> rects;
        geometry_msgs::msg::TransformStamped::SharedPtr LiDAR2cam_transform = std::make_shared<geometry_msgs::msg::TransformStamped>();
        getTransform(name, "base_link", LiDAR2cam_transform);
        Eigen::Matrix4f affine_matrix = tf2::transformToEigen(LiDAR2cam_transform->transform).matrix().cast<float>();

        // std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
        for(size_t i = 0; i < objects_eight_cord.size(); ++i)
        {
            // std::cout << "===================================" << std::endl;
            int count = 0;
            pcl::PointCloud<pcl::PointXYZ> eight_cord_camera_frame;
            pcl::transformPointCloud(objects_eight_cord[i], eight_cord_camera_frame, affine_matrix);
            std::vector<cv::Point> projection_points;
            for(size_t j = 0; j < eight_cord_camera_frame.points.size(); ++j)
            {
                //1. projection 3d point
                if(eight_cord_camera_frame[j].z < 0.0001) eight_cord_camera_frame[j].z = 0.0001;

                int u = int(eight_cord_camera_frame[j].x * fx_ / eight_cord_camera_frame[j].z + cx_);
                int v = int(eight_cord_camera_frame[j].y * fy_ / eight_cord_camera_frame[j].z + cy_);
                cv::Point pt1;
                if ((u >= 0) && (u < image_width) && (v >= 0) && (v < image_height) && eight_cord_camera_frame[j].z > 0) //&& eight_cord_camera_frame[j].z > 0)
                {
                    pt1.x = u;
                    pt1.y = v;
                    // std::cout << "(" << pt1.x << " , " << pt1.y << ")" << std::endl;
                    projection_points.push_back(pt1);
                    count++;
                }

            }
            //2. projection_point2boundingRect  (sensor_msgs::msg::RegionOfInterest)
            cv::Rect boundRect;
            if(count < 3)
            {
                boundRect.x = 9999;
                // std::cout << "pass" << std::endl;
            }
            else boundRect = cv::boundingRect(projection_points);
            rects.push_back(boundRect);
        }

        return rects;
    }

    autoware_auto_perception_msgs::msg::DetectedObjects PixelCloudFusionNodelet::calcObjectCenterPointIOU(std::vector<cv::Rect> objects_2d_rect, const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr &cam_obj,
    const autoware_auto_perception_msgs::msg::DetectedObjects &unknown_center_point)
    {
        autoware_auto_perception_msgs::msg::DetectedObjects detected_objects;
        for(size_t i = 0; i < unknown_center_point.objects.size(); i++)
        {
            double max_iou = -9999.9999;

            autoware_auto_perception_msgs::msg::DetectedObject temp_object;

            for(size_t j = 0; j < cam_obj->feature_objects.size(); ++j)
            {
                //covert center_x, center_y to left_x, top_y
                
                sensor_msgs::msg::RegionOfInterest cam_obj_roi;
                
                cam_obj_roi.x_offset = cam_obj->feature_objects[j].feature.roi.x_offset; // - cam_obj->feature_objects[j].feature.roi.width / 2;
                cam_obj_roi.y_offset = cam_obj->feature_objects[j].feature.roi.y_offset; // - cam_obj->feature_objects[j].feature.roi.height / 2;
                cam_obj_roi.width = cam_obj->feature_objects[j].feature.roi.width;
                cam_obj_roi.height = cam_obj->feature_objects[j].feature.roi.height;
                sensor_msgs::msg::RegionOfInterest roi_centerpoint;
                
                if(objects_2d_rect[i].x!=9999)
                {
                    roi_centerpoint.x_offset = objects_2d_rect[i].x;
                    roi_centerpoint.y_offset = objects_2d_rect[i].y;
                    roi_centerpoint.width = objects_2d_rect[i].width;
                    roi_centerpoint.height= objects_2d_rect[i].height;
                    double iou = image_projection_based_fusion::calcIoU(cam_obj_roi, roi_centerpoint);

                    if (max_iou < iou && iou > 0.1 )
                    {
                        max_iou = iou;
                        
                        temp_object = unknown_center_point.objects[i];
                        temp_object.classification = cam_obj->feature_objects[j].object.classification;
                    }
                }
            }

            if(max_iou != -9999.9999)
                detected_objects.objects.emplace_back(temp_object);
        }
        return detected_objects;
    }


    autoware_auto_perception_msgs::msg::DetectedObjects PixelCloudFusionNodelet::calcObjectEuclideanIOU(std::vector<cv::Rect> objects_2d_rect, const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr &cam_obj,
    const autoware_auto_perception_msgs::msg::DetectedObjects &unknown_center_point)
    {
        autoware_auto_perception_msgs::msg::DetectedObjects detected_objects;

        for(size_t i = 0; i < unknown_center_point.objects.size(); i++)
        {
            double max_iou = -9999.9999;

            autoware_auto_perception_msgs::msg::DetectedObject temp_object;

            for(size_t j = 0; j < cam_obj->feature_objects.size(); ++j)
            {
                //covert center_x, center_y to left_x, top_y
                
                sensor_msgs::msg::RegionOfInterest cam_obj_roi;
                
                cam_obj_roi.x_offset = cam_obj->feature_objects[j].feature.roi.x_offset - cam_obj->feature_objects[j].feature.roi.width / 2;
                cam_obj_roi.y_offset = cam_obj->feature_objects[j].feature.roi.y_offset - cam_obj->feature_objects[j].feature.roi.height / 2;
                cam_obj_roi.width = cam_obj->feature_objects[j].feature.roi.width;
                cam_obj_roi.height = cam_obj->feature_objects[j].feature.roi.height;
                sensor_msgs::msg::RegionOfInterest roi_centerpoint;
                
                if(objects_2d_rect[i].x!=9999)
                {
                    roi_centerpoint.x_offset = objects_2d_rect[i].x;
                    roi_centerpoint.y_offset = objects_2d_rect[i].y;
                    roi_centerpoint.width = objects_2d_rect[i].width;
                    roi_centerpoint.height= objects_2d_rect[i].height;
                    double iou = image_projection_based_fusion::calcIoU(cam_obj_roi, roi_centerpoint);

                    if (max_iou < iou && iou > 0.5 )
                    {
                        max_iou = iou;
                        
                        temp_object = unknown_center_point.objects[i];
                        temp_object.classification = cam_obj->feature_objects[j].object.classification;
                    }
                }
            }

            if(max_iou != -9999.9999)
                detected_objects.objects.emplace_back(temp_object);
        }

        return detected_objects;
    }

    autoware_auto_perception_msgs::msg::DetectedObjects PixelCloudFusionNodelet::makeEuclideanClsuterAutowareObject(pcl::PointCloud<pcl::PointXYZ> input_cloud)
    {
        autoware_auto_perception_msgs::msg::DetectedObjects euclidean_cluster_autoware_objects;

        pcl::PointCloud<pcl::PointXYZ>::Ptr last_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(input_cloud));
        std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
        normal_cluster_->cluster(last_pointcloud_ptr, clusters);

        for(size_t i = 0; i < clusters.size(); ++i)
        {
            autoware_auto_perception_msgs::msg::DetectedObject sample_object;
            
            autoware_auto_perception_msgs::msg::Shape shape_output;
            geometry_msgs::msg::Pose pose_output;

            bool estimate_ok = estimate(clusters[i], shape_output, pose_output);

            if(estimate_ok == false) continue;

            autoware_auto_perception_msgs::msg::ObjectClassification class_data;
            class_data.label = 0;
            class_data.probability = 1.0;
            sample_object.classification.push_back(class_data);
            sample_object.kinematics.pose_with_covariance.pose = pose_output;
            sample_object.kinematics.has_position_covariance = false;
            sample_object.kinematics.orientation_availability = false;
            sample_object.kinematics.twist_with_covariance.twist.linear.x = 0.0;
            sample_object.kinematics.twist_with_covariance.twist.linear.y = 0.0;
            sample_object.kinematics.twist_with_covariance.twist.linear.z = 0.0;
            sample_object.kinematics.twist_with_covariance.twist.angular.x = 0.0;
            sample_object.kinematics.twist_with_covariance.twist.angular.y = 0.0;
            sample_object.kinematics.twist_with_covariance.twist.angular.z = 0.0;
            sample_object.kinematics.has_twist = false;
            sample_object.kinematics.has_twist_covariance = false;
            sample_object.shape = shape_output;

            euclidean_cluster_autoware_objects.objects.push_back(sample_object);
        }

        return euclidean_cluster_autoware_objects;
    }


    autoware_auto_perception_msgs::msg::DetectedObjects PixelCloudFusionNodelet::mergeFusionObject(autoware_auto_perception_msgs::msg::DetectedObjects obj1,
                                                                        autoware_auto_perception_msgs::msg::DetectedObjects obj2,
                                                                        autoware_auto_perception_msgs::msg::DetectedObjects obj3,
                                                                        autoware_auto_perception_msgs::msg::DetectedObjects obj4,
                                                                        autoware_auto_perception_msgs::msg::DetectedObjects obj5)
        
    {
        autoware_auto_perception_msgs::msg::DetectedObjects merged_fusion_objects;

        for(size_t i = 0; i < obj1.objects.size(); ++i)
        {
            if(obj1.objects[i].classification.front().label == 0) // unknown
                continue;
            
            merged_fusion_objects.objects.push_back(obj1.objects[i]);
        }
        for(size_t i = 0; i < obj2.objects.size(); ++i)
        {
            if(obj2.objects[i].classification.front().label == 0) // unknown
                continue;
            
            merged_fusion_objects.objects.push_back(obj2.objects[i]);
        }
        for(size_t i = 0; i < obj3.objects.size(); ++i)
        {
            if(obj3.objects[i].classification.front().label == 0) // unknown
                continue;
            
            merged_fusion_objects.objects.push_back(obj3.objects[i]);
        }
        for(size_t i = 0; i < obj4.objects.size(); ++i)
        {
            
            if(obj4.objects[i].classification.front().label == 0) // unknown
                continue;
            
            bool pass = false;

            for(size_t k = 0;  k < merged_fusion_objects.objects.size(); ++k)
            {
                double object_IOU = utils::get2dIoU({merged_fusion_objects.objects[k].kinematics.pose_with_covariance.pose, merged_fusion_objects.objects[k].shape},
            {obj4.objects[i].kinematics.pose_with_covariance.pose, obj4.objects[i].shape});
                if(object_IOU < 0.5)
                {
                    pass = true;
                    break;
                }
            }

            if(pass == true) continue;
            
            merged_fusion_objects.objects.push_back(obj4.objects[i]);
        }

        for(size_t i = 0; i < obj5.objects.size(); ++i)
        {
            if(obj5.objects[i].classification.front().label == 0) // unknown
                continue;
            
            bool pass = false;

            for(size_t k = 0;  k < merged_fusion_objects.objects.size(); ++k)
            {
                double object_IOU = utils::get2dIoU({merged_fusion_objects.objects[k].kinematics.pose_with_covariance.pose, merged_fusion_objects.objects[k].shape},
            {obj5.objects[i].kinematics.pose_with_covariance.pose, obj5.objects[i].shape});

                if(object_IOU < 0.5)
                {
                    pass = true;
                    break;
                }
            }

            if(pass == true) continue;
            
            merged_fusion_objects.objects.push_back(obj5.objects[i]);
        }

        return merged_fusion_objects;
    }

    bool PixelCloudFusionNodelet::estimate(const pcl::PointCloud<pcl::PointXYZ> & cluster,
        autoware_auto_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output)
    {
        float min_angle, max_angle;
        min_angle = 0.0;
        max_angle = M_PI * 0.5;
        
        return fitLShape(cluster, min_angle, max_angle, shape_output, pose_output);
    }

    bool PixelCloudFusionNodelet::fitLShape( const pcl::PointCloud<pcl::PointXYZ> & cluster, const float min_angle, const float max_angle,
        autoware_auto_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output)
    {
        constexpr float epsilon = 0.001;

        // calc min and max z for height
        float min_z = cluster.empty() ? 0.0 : cluster.at(0).z;
        float max_z = cluster.empty() ? 0.0 : cluster.at(0).z;
        for (const auto & point : cluster) {
            min_z = std::min(point.z, min_z);
            max_z = std::max(point.z, max_z);
        }

        /*
        * Paper : IV2017, Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners
        * Authors : Xio Zhang, Wenda Xu, Chiyu Dong and John M. Dolan
        */

        // Paper : Algo.2 Search-Based Rectangle Fitting
        std::vector<std::pair<float /*theta*/, float /*q*/>> Q;
        constexpr float angle_resolution = M_PI / 180.0;
        for (float theta = min_angle; theta <= max_angle + epsilon; theta += angle_resolution) {
            Eigen::Vector2f e_1;
            e_1 << std::cos(theta), std::sin(theta);  // col.3, Algo.2
            Eigen::Vector2f e_2;
            e_2 << -std::sin(theta), std::cos(theta);  // col.4, Algo.2
            std::vector<float> C_1;                    // col.5, Algo.2
            std::vector<float> C_2;                    // col.6, Algo.2
            for (const auto & point : cluster) {
            C_1.push_back(point.x * e_1.x() + point.y * e_1.y());
            C_2.push_back(point.x * e_2.x() + point.y * e_2.y());
            }
            float q = calcClosenessCriterion(C_1, C_2);  // col.7, Algo.2
            Q.push_back(std::make_pair(theta, q));       // col.8, Algo.2
        }

        float theta_star{0.0};  // col.10, Algo.2
        float max_q = 0.0;
        for (size_t i = 0; i < Q.size(); ++i) {
            if (max_q < Q.at(i).second || i == 0) {
            max_q = Q.at(i).second;
            theta_star = Q.at(i).first;
            }
        }
        const float sin_theta_star = std::sin(theta_star);
        const float cos_theta_star = std::cos(theta_star);

        Eigen::Vector2f e_1_star;  // col.11, Algo.2
        Eigen::Vector2f e_2_star;
        e_1_star << cos_theta_star, sin_theta_star;
        e_2_star << -sin_theta_star, cos_theta_star;
        std::vector<float> C_1_star;  // col.11, Algo.2
        std::vector<float> C_2_star;  // col.11, Algo.2
        for (const auto & point : cluster) {
            C_1_star.push_back(point.x * e_1_star.x() + point.y * e_1_star.y());
            C_2_star.push_back(point.x * e_2_star.x() + point.y * e_2_star.y());
        }

        // col.12, Algo.2
        const float min_C_1_star = *std::min_element(C_1_star.begin(), C_1_star.end());
        const float max_C_1_star = *std::max_element(C_1_star.begin(), C_1_star.end());
        const float min_C_2_star = *std::min_element(C_2_star.begin(), C_2_star.end());
        const float max_C_2_star = *std::max_element(C_2_star.begin(), C_2_star.end());

        const float a_1 = cos_theta_star;
        const float b_1 = sin_theta_star;
        const float c_1 = min_C_1_star;
        const float a_2 = -1.0 * sin_theta_star;
        const float b_2 = cos_theta_star;
        const float c_2 = min_C_2_star;
        const float a_3 = cos_theta_star;
        const float b_3 = sin_theta_star;
        const float c_3 = max_C_1_star;
        const float a_4 = -1.0 * sin_theta_star;
        const float b_4 = cos_theta_star;
        const float c_4 = max_C_2_star;

        // calc center of bounding box
        float intersection_x_1 = (b_1 * c_2 - b_2 * c_1) / (a_2 * b_1 - a_1 * b_2);
        float intersection_y_1 = (a_1 * c_2 - a_2 * c_1) / (a_1 * b_2 - a_2 * b_1);
        float intersection_x_2 = (b_3 * c_4 - b_4 * c_3) / (a_4 * b_3 - a_3 * b_4);
        float intersection_y_2 = (a_3 * c_4 - a_4 * c_3) / (a_3 * b_4 - a_4 * b_3);

        // calc dimension of bounding box
        Eigen::Vector2f e_x;
        Eigen::Vector2f e_y;
        e_x << a_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1)), b_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1));
        e_y << a_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2)), b_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2));
        Eigen::Vector2f diagonal_vec;
        diagonal_vec << intersection_x_1 - intersection_x_2, intersection_y_1 - intersection_y_2;

        // calc yaw
        tf2::Quaternion quat;
        quat.setEuler(/* roll */ 0, /* pitch */ 0, /* yaw */ std::atan2(e_1_star.y(), e_1_star.x()));

        // output
        shape_output.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
        shape_output.dimensions.x = std::fabs(e_x.dot(diagonal_vec));
        shape_output.dimensions.y = std::fabs(e_y.dot(diagonal_vec));
        shape_output.dimensions.z = std::max((max_z - min_z), epsilon);
        pose_output.position.x = (intersection_x_1 + intersection_x_2) * 0.5;
        pose_output.position.y = (intersection_y_1 + intersection_y_2) * 0.5;
        pose_output.position.z = min_z + shape_output.dimensions.z * 0.5;
        pose_output.orientation = tf2::toMsg(quat);
        // check wrong output
        shape_output.dimensions.x = std::max(static_cast<float>(shape_output.dimensions.x), epsilon);
        shape_output.dimensions.y = std::max(static_cast<float>(shape_output.dimensions.y), epsilon);

        return true;
    }

    float PixelCloudFusionNodelet::calcClosenessCriterion( const std::vector<float> & C_1, const std::vector<float> & C_2)
    {
        // Paper : Algo.4 Closeness Criterion
        const float min_c_1 = *std::min_element(C_1.begin(), C_1.end());  // col.2, Algo.4
        const float max_c_1 = *std::max_element(C_1.begin(), C_1.end());  // col.2, Algo.4
        const float min_c_2 = *std::min_element(C_2.begin(), C_2.end());  // col.3, Algo.4
        const float max_c_2 = *std::max_element(C_2.begin(), C_2.end());  // col.3, Algo.4

        std::vector<float> D_1;  // col.4, Algo.4
        for (const auto & c_1_element : C_1) {
            const float v = std::min(max_c_1 - c_1_element, c_1_element - min_c_1);
            D_1.push_back(v * v);
        }

        std::vector<float> D_2;  // col.5, Algo.4
        for (const auto & c_2_element : C_2) {
            const float v = std::min(max_c_2 - c_2_element, c_2_element - min_c_2);
            D_2.push_back(v * v);
        }
        constexpr float d_min = 0.1 * 0.1;
        constexpr float d_max = 0.4 * 0.4;
        float beta = 0;  // col.6, Algo.4
        for (size_t i = 0; i < D_1.size(); ++i) {
            if (d_max < std::min(D_1.at(i), D_2.at(i))) {
            continue;
            }
            const float d = std::max(std::min(D_1.at(i), D_2.at(i)), d_min);
            beta += 1.0 / d;
        }
        return beta;
    }


}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcfusion::PixelCloudFusionNodelet)
