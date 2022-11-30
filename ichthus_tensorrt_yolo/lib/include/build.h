//
// Created by linghu8812 on 2022/8/29.
//

#ifndef TENSORRT_INFERENCE_BUILD_H
#define TENSORRT_INFERENCE_BUILD_H

// #include "ScaledYOLOv4.h"
#include "Yolov4.h"
#include "yolov7.h"

namespace yolo
{
std::shared_ptr<Model> build_model(const Config& config);
}

#endif //TENSORRT_INFERENCE_BUILD_H
