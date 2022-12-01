#include "detection.h"
namespace yolo
{
Detection::Detection(const Config& config) : Model(config) {
    labels_file = config.labels_file;
    obj_threshold = config.obj_threshold;
    nms_threshold = config.nms_threshold;
    agnostic = config.agnostic;
    strides = config.strides;
    num_anchors = config.num_anchors;
    int index = 0;
    for (const int &stride : strides)
    {
        int num_anchor = num_anchors[index] !=0 ? num_anchors[index] : 1;
        num_rows += int(IMAGE_HEIGHT / stride) * int(IMAGE_WIDTH / stride) * num_anchor;
        index+=1;
    }
    class_labels = ReadClassLabel(labels_file);
    CATEGORY = class_labels.size();
    class_colors.resize(CATEGORY);
    srand((int) time(nullptr));
    for (cv::Scalar &class_color : class_colors)
        class_color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
}

std::vector<DetectRes> Detection::InferenceImages(std::vector<cv::Mat> &vec_img) {
    std::vector<float> image_data = PreProcess(vec_img);
    auto *output = new float[outSize * BATCH_SIZE];;
    ModelInference(image_data, output);
    auto boxes = PostProcess(vec_img, output);
    delete[] output;
    return boxes;
}

// Modified by Heywon on 2022/10/26
void Detection::InferenceFolder(std::vector<cv::Mat> &vec_img,std::vector<DetectRes> &detections){
    auto det_results = InferenceImages(vec_img);
    detections = det_results;
}

void Detection::NmsDetect(std::vector<Bbox> &detections) {
    sort(detections.begin(), detections.end(), [=](const Bbox &left, const Bbox &right) {
        return left.prob > right.prob;
    });

    for (int i = 0; i < (int)detections.size(); i++)
        for (int j = i + 1; j < (int)detections.size(); j++)
        {
            if (detections[i].classes == detections[j].classes or agnostic)
            {
                float iou = IOUCalculate(detections[i], detections[j]);
                if (iou > nms_threshold)
                    detections[j].prob = 0;
            }
        }

    detections.erase(std::remove_if(detections.begin(), detections.end(), [](const Bbox &det)
    { return det.prob == 0; }), detections.end());
}

float Detection::IOUCalculate(const Bbox &det_a, const Bbox &det_b) {
    cv::Point2f center_a(det_a.x, det_a.y);
    cv::Point2f center_b(det_b.x, det_b.y);
    cv::Point2f left_up(std::min(det_a.x - det_a.w / 2, det_b.x - det_b.w / 2),
                        std::min(det_a.y - det_a.h / 2, det_b.y - det_b.h / 2));
    cv::Point2f right_down(std::max(det_a.x + det_a.w / 2, det_b.x + det_b.w / 2),
                           std::max(det_a.y + det_a.h / 2, det_b.y + det_b.h / 2));
    float distance_d = (center_a - center_b).x * (center_a - center_b).x + (center_a - center_b).y * (center_a - center_b).y;
    float distance_c = (left_up - right_down).x * (left_up - right_down).x + (left_up - right_down).y * (left_up - right_down).y;
    float inter_l = det_a.x - det_a.w / 2 > det_b.x - det_b.w / 2 ? det_a.x - det_a.w / 2 : det_b.x - det_b.w / 2;
    float inter_t = det_a.y - det_a.h / 2 > det_b.y - det_b.h / 2 ? det_a.y - det_a.h / 2 : det_b.y - det_b.h / 2;
    float inter_r = det_a.x + det_a.w / 2 < det_b.x + det_b.w / 2 ? det_a.x + det_a.w / 2 : det_b.x + det_b.w / 2;
    float inter_b = det_a.y + det_a.h / 2 < det_b.y + det_b.h / 2 ? det_a.y + det_a.h / 2 : det_b.y + det_b.h / 2;
    if (inter_b < inter_t || inter_r < inter_l)
        return 0;
    float inter_area = (inter_b - inter_t) * (inter_r - inter_l);
    float union_area = det_a.w * det_a.h + det_b.w * det_b.h - inter_area;
    if (union_area == 0)
        return 0;
    else
        return inter_area / union_area - distance_d / distance_c;
}

void Detection::DrawResults(const std::vector<DetectRes> &detections, std::vector<cv::Mat> &vec_img,
                            std::vector<std::string> image_name=std::vector<std::string>()) {
    for (int i = 0; i < (int)vec_img.size(); i++) {
        auto org_img = vec_img[i];
        if (!org_img.data)
            continue;
        auto rects = detections[i].det_results;
        if (channel_order == "BGR")
            cv::cvtColor(org_img, org_img, cv::COLOR_BGR2RGB);
        
        for(const auto &rect : rects) {
            char t[256];
            sprintf(t, "%.2f", rect.prob);
            std::string name = class_labels[rect.classes] + "-" + t;
            cv::putText(org_img, name, cv::Point(rect.x - rect.w / 2, rect.y - rect.h / 2 - 5),
                    cv::FONT_HERSHEY_COMPLEX, 0.7, class_colors[rect.classes], 2);
            cv::Rect rst(rect.x - rect.w / 2, rect.y - rect.h / 2, rect.w, rect.h);
            cv::rectangle(org_img, rst, class_colors[rect.classes], 2, cv::LINE_8, 0);
        }
        if (!image_name.empty()) {
            int pos = image_name[i].find_last_of('.');
            std::string rst_name = image_name[i].insert(pos, "_");
            std::cout << rst_name << std::endl;
            cv::imwrite(rst_name, org_img);
        }
    }
}
}