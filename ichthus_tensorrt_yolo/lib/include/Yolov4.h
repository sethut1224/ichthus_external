#ifndef TENSORRT_INFERENCE_YOLOV4_H
#define TENSORRT_INFERENCE_YOLOV4_H

#include "detection.h"
namespace yolo
{
class YOLOv4 : public Detection {
public:
    explicit YOLOv4(const Config& config);// Modified by Heywon on 2022/10/26

private:
    std::vector<DetectRes> PostProcess(const std::vector<cv::Mat> &vec_Mat, float *output) override;
    void GenerateReferMatrix();
    static float sigmoid(float in);
    bool letter_box;
    int refer_rows;
    int refer_cols;
    cv::Mat refer_matrix;
    std::vector<std::vector<int>> grids;
    std::vector<std::vector<int>> anchors;
};
}
#endif //TENSORRT_INFERENCE_YOLOV4_H
