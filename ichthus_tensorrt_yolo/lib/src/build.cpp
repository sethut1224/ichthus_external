//
// Created by linghu8812 on 2022/8/29.
//
// Modified by Heywon on 2022/10/26
//
#include "build.h"

namespace yolo{
std::shared_ptr<Model> build_model(const Config& config) {
    std::string config_file = config.yolo_version;
    auto model = std::shared_ptr<Model>();
    if (config_file == "yolov4"){
        model = std::make_shared<YOLOv4>(config);
        std::cout <<"build : " <<config_file << std::endl;
    }
    else if (config_file == "yolov7"){
        model = std::make_shared<yolov7>(config);
    }
    else
        std::cout << "No model arch matched!" << std::endl;
    std::cout << "build_model" <<std::endl;
    return model;
}
} //namespace yolo