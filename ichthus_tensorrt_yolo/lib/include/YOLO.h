//
// Created by linghu8812 on 2022/7/21.
//

#ifndef TENSORRT_INFERENCE_YOLO_H
#define TENSORRT_INFERENCE_YOLO_H

#include "detection.h"
namespace yolo
{
class YOLO : public Detection {
public:
    explicit YOLO(const Config& config);// Modified by Heywon on 2022/10/26

protected:
    std::vector<DetectRes> PostProcess(const std::vector<cv::Mat> &vec_Mat, float *output) override;
};
}
#endif //TENSORRT_INFERENCE_YOLO_H
