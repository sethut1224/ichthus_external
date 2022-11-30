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

#include "ichthus_tensorrt_yolo/nodelet.hpp"
#include "build.h"
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>

#include <glob.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace object_recognition
{
IchthusTensorrtYoloNodelet::IchthusTensorrtYoloNodelet(const rclcpp::NodeOptions & options)
: Node("ichthus_tensorrt_yolo", options)
{
  using std::placeholders::_1;
  
  std::string yolo_version = declare_parameter("yolo_version", "");
  std::string engine_file = declare_parameter("engine_file", "");
  std::string label_file = declare_parameter("label_file", "");
  input_topic = declare_parameter("input_topic","");
  std::string output_topic = declare_parameter("output_topic","");
  std::cout <<"label_file : "<<label_file << std::endl;
  std::cout<<"yolo_version : " << yolo_version <<std::endl;
  std::cout<<"engine_file : " << engine_file <<std::endl;
  // std::string onnx_file = declare_parameter("onnx_file", "");
  yolo_config_.engine_file = engine_file;
  yolo_config_.yolo_version = yolo_version;

  auto anchors = declare_parameter("anchors", std::vector<long>());
  if (yolo_config_.yolo_version =="yolov4"){
    std::vector<int> anchors_int(anchors.begin(), anchors.end());
    std::vector<std::vector<int>> anchors_int_int;
    for(int i = 0; i < int(sizeof(anchors_int)); i+=2)
    {
      std::vector<int> temp = {anchors_int[i], anchors_int[i+1]};
      anchors_int_int.push_back(temp);
    }
    yolo_config_.anchors = anchors_int_int;
  }
  
  auto strides = declare_parameter("strides", std::vector<long>());
  std::vector<int> strides_int(strides.begin(), strides.end());
  yolo_config_.strides = strides_int;

  auto num_anchors= declare_parameter("num_anchors",std::vector<long>{3, 3, 3});
  std::vector<int> num_anchors_int(num_anchors.begin(), num_anchors.end());
  yolo_config_.num_anchors = num_anchors_int;

  yolo_config_.BATCH_SIZE= declare_parameter("BATCH_SIZE",1);
  yolo_config_.INPUT_CHANNEL= declare_parameter("INPUT_CHANNEL",3);
  yolo_config_.IMAGE_WIDTH= declare_parameter("IMAGE_WIDTH",640);
  yolo_config_.IMAGE_HEIGHT= declare_parameter("IMAGE_HEIGHT",640);
  yolo_config_.image_order= declare_parameter("image_order","");
  yolo_config_.channel_order= declare_parameter("channel_order","");
  auto img_mean = declare_parameter("img_mean",std::vector<double>());
  std::vector<float> img_mean_float(img_mean.begin(), img_mean.end());

  auto img_std = declare_parameter("img_std",std::vector<double>());
  std::vector<float> img_std_float(img_std.begin(), img_std.end());

  yolo_config_.img_mean= img_mean_float;
  yolo_config_.img_std= img_std_float;
  yolo_config_.alpha= declare_parameter("alpha",255.0);
  yolo_config_.resize= declare_parameter("resize","");
  yolo_config_.labels_file=label_file;//config["labels_file"}.as<std::string>();
  yolo_config_.obj_threshold=declare_parameter("obj_threshold",float());//config["obj_threshold"].as<float>();
  yolo_config_.nms_threshold=declare_parameter("nms_threshold",float());//config["nms_threshold"].as<float>();
  yolo_config_.agnostic=declare_parameter("agnostic",false);//config["agnostic"].as<bool>();

  if (!readLabelFile(label_file, &labels_)) {
    RCLCPP_ERROR(this->get_logger(), "Could not find label file");
  }
  std::ifstream fs(engine_file);

  if (fs.is_open()) {
    RCLCPP_INFO(this->get_logger(), "Found %s", engine_file.c_str());
    std::cout<<"label_file : " << label_file <<std::endl;
    net_ptr_ = build_model(yolo_config_);
    net_ptr_->LoadEngine();
    }

  RCLCPP_INFO(this->get_logger(), "Inference engine prepared.");

  std::lock_guard<std::mutex> lock(connect_mutex_);

  objects_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(output_topic, 1);
  std::string out_img = output_topic + "/debug/image";
  image_pub_ = image_transport::create_publisher(this, out_img);
  image_sub_orig_ = this->create_subscription<sensor_msgs::msg::Image>(input_topic, rclcpp::SensorDataQoS(), std::bind(&IchthusTensorrtYoloNodelet::callback, this, _1));

}

void IchthusTensorrtYoloNodelet::callback(sensor_msgs::msg::Image::UniquePtr in_image_msg_uniq)
{
  sensor_msgs::msg::Image::SharedPtr in_image_msg(new sensor_msgs::msg::Image(*in_image_msg_uniq));

  auto start = rclcpp::Clock().now();

  using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

  tier4_perception_msgs::msg::DetectedObjectsWithFeature out_objects;

  cv_bridge::CvImagePtr in_image_ptr;
  try {
    in_image_ptr = cv_bridge::toCvCopy(in_image_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  std::vector<cv::Mat> vec_img(1); 
  vec_img[0] = in_image_ptr->image.clone();
  net_ptr_->InferenceFolder(vec_img, out_bboxes_);
  std::cout <<"complete_InferenceFolder"<<std::endl;
  auto det_results = out_bboxes_;

  for(int i = 0; i < (int)vec_img.size(); i++){
    auto org_img = vec_img[i];
    auto rects = det_results[i].det_results;
    if (yolo_config_.channel_order=="BGR")
      cv::cvtColor(org_img, org_img, cv::COLOR_BGR2RGB);
    for (const auto &rect: rects){
      char t[256];
      sprintf(t, "%.2f", rect.prob);
      if ( rect.prob < yolo_config_.obj_threshold){
        break;
      }
      const auto class_id = static_cast<int>(rect.classes);
      if(labels_[class_id]=="car" || labels_[class_id]=="person"|| labels_[class_id] == "bus"||
      labels_[class_id] == "truck"||labels_[class_id] == "motorbike"||labels_[class_id] == "speed_bust")
      {
        tier4_perception_msgs::msg::DetectedObjectWithFeature object;
        object.feature.roi.x_offset = rect.x;
        object.feature.roi.y_offset = rect.y;
        object.feature.roi.width = rect.w;
        object.feature.roi.height = rect.h;
        object.object.classification.emplace_back(autoware_auto_perception_msgs::build<Label>()
                                                    .label(Label::UNKNOWN)
                                                    .probability(rect.prob));
        std::cout << "rect.prob : " << rect.prob << std::endl;
        
        if (labels_[class_id] == "car") {
        object.object.classification.front().label = Label::CAR;
        } else if (labels_[class_id] == "person") {
          object.object.classification.front().label = Label::PEDESTRIAN;
        } else if (labels_[class_id] == "bus") {
          object.object.classification.front().label = Label::BUS;
        } else if (labels_[class_id] == "truck") {
          object.object.classification.front().label = Label::TRUCK;
        } else if (labels_[class_id] == "bicycle") {
          object.object.classification.front().label = Label::BICYCLE;
        } else if (labels_[class_id] == "motorbike") {
          object.object.classification.front().label = Label::MOTORCYCLE;
        } else {
          object.object.classification.front().label = Label::UNKNOWN;//speed_bust
        }
        out_objects.feature_objects.push_back(object);
        std::string name = labels_[int(class_id)] + "-" + t;
        cv::putText(in_image_ptr->image, name, cv::Point(rect.x - rect.w / 2, rect.y - rect.h / 2 - 5),
                      cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        cv::Rect rst(rect.x - rect.w / 2, rect.y - rect.h / 2, rect.w, rect.h);
        cv::rectangle(in_image_ptr->image, rst, cv::Scalar(0, 0, 255), 2, cv::LINE_8, 0);
      }
    }
  }
  auto end = rclcpp::Clock().now();
  
  std::cout<<(end-start).nanoseconds() * 1.0e-6 << " ms"<<std::endl;
  image_pub_.publish(in_image_ptr->toImageMsg());

  out_objects.header = in_image_msg->header;
  objects_pub_->publish(out_objects);
}

bool IchthusTensorrtYoloNodelet::readLabelFile(
  const std::string & filepath, std::vector<std::string> * labels)
{
  std::ifstream labelsFile(filepath);
  if (!labelsFile.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open label file. [%s]", filepath.c_str());
    return false;
  }
  std::string label;
  while (getline(labelsFile, label)) {
    labels->push_back(label);
  }
  return true;
}

} // namespace object_recognition

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(object_recognition::IchthusTensorrtYoloNodelet)
