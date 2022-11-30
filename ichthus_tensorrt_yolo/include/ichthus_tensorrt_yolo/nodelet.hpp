// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*  ===========================================================================
    Copyright 2022. The ICHTHUS Project. All Rights Reserved.
    Hyewon Bang (hwbang0815@naver.com) and Youngjoon Han (young@ssu.ac.kr).
    Vision Systems Laboratory, Soongsil University.
    added by ICHTHUS, Hyewon Bang on 20221026
    [Licensed under the MIT License]
    ===========================================================================*/

#ifndef ICHTHUS_TENSORRT_YOLO__NODELET_HPP_
#define ICHTHUS_TENSORRT_YOLO__NODELET_HPP_

#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trt_yolo.h>
#include <build.h>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace object_recognition
{
class IchthusTensorrtYoloNodelet : public rclcpp::Node
{
public:
  explicit IchthusTensorrtYoloNodelet(const rclcpp::NodeOptions & options);
  void callback(sensor_msgs::msg::Image::UniquePtr in_image_msg);
  bool readLabelFile(const std::string & filepath, std::vector<std::string> * labels);

private:
  std::mutex connect_mutex_;

  image_transport::Publisher image_pub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr objects_pub_;

  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_orig_;

  yolo::Config yolo_config_;

  std::vector<std::string> labels_;
  std::vector<yolo::DetectRes> out_bboxes_;
  std::shared_ptr<yolo::Model> net_ptr_;
  std::string input_topic;
};

}  // namespace object_recognition

#endif  // ICHTHUS_TENSORRT_YOLO__NODELET_HPP_