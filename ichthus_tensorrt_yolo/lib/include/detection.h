//
// Modified by Heywon on 2022/10/26
//
#ifndef TENSORRT_INFERENCE_DETECTION_H
#define TENSORRT_INFERENCE_DETECTION_H

#include "classification.h"
namespace yolo
{
class Detection : public Model
{
public:
    explicit Detection(const Config& config);
    std::vector<DetectRes> InferenceImages(std::vector<cv::Mat> &vec_img);
    void InferenceFolder(std::vector<cv::Mat> &vec_img,std::vector<DetectRes> &detections) override;
    void DrawResults(const std::vector<DetectRes> &detections, std::vector<cv::Mat> &vec_img,
                     std::vector<std::string> image_name);
    static float IOUCalculate(const Bbox &det_a, const Bbox &det_b);

protected:
    virtual std::vector<DetectRes> PostProcess(const std::vector<cv::Mat> &vec_Mat, float *output)=0;
    void NmsDetect(std::vector<Bbox> &detections);
    std::map<int, std::string> class_labels;
    int CATEGORY;
    float obj_threshold;
    float nms_threshold;
    bool agnostic;
    std::vector<cv::Scalar> class_colors;
    std::vector<int> strides;
    std::vector<int> num_anchors;
    int num_rows = 0;
};
}
#endif //TENSORRT_INFERENCE_DETECTION_H
