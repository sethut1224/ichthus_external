#include "classification.h"
namespace yolo
{
Classification::Classification(const Config& config) : Model(config) {
    labels_file = config.labels_file;// Modified by Heywon on 2022/10/26
    class_labels = ReadImageNetLabel(labels_file);
    CATEGORY = class_labels.size();
}

std::vector<ClassRes> Classification::InferenceImages(std::vector<cv::Mat> &vec_img) {
    auto t_start_pre = std::chrono::high_resolution_clock::now();
    std::vector<float> image_data = PreProcess(vec_img);
    auto t_end_pre = std::chrono::high_resolution_clock::now();
    float total_pre = std::chrono::duration<float, std::milli>(t_end_pre - t_start_pre).count();
    std::cout << "classification prepare image take: " << total_pre << " ms." << std::endl;
    auto *output = new float[outSize * BATCH_SIZE];;
    auto t_start = std::chrono::high_resolution_clock::now();
    ModelInference(image_data, output);
    auto t_end = std::chrono::high_resolution_clock::now();
    float total_inf = std::chrono::duration<float, std::milli>(t_end - t_start).count();
    std::cout << "classification inference take: " << total_inf << " ms." << std::endl;
    auto r_start = std::chrono::high_resolution_clock::now();
    auto results = PostProcess(vec_img, output);
    auto r_end = std::chrono::high_resolution_clock::now();
    float total_res = std::chrono::duration<float, std::milli>(r_end - r_start).count();
    std::cout << "classification postprocess take: " << total_res << " ms." << std::endl;
    delete[] output;
    return results;
}

// Modified by Heywon on 2022/10/26 : Delete InferenceFolder

std::vector<ClassRes> Classification::PostProcess(const std::vector<cv::Mat> &vec_Mat, float *output) {
    std::vector<ClassRes> vec_result;
    int index = 0;
    for (const cv::Mat &src_img : vec_Mat) {
        cv::Mat img = src_img;
        ClassRes result;
        float *out = output + index * outSize;
        auto max_pos = std::max_element(out, out + outSize);
        result.classes = max_pos - out;
        result.prob = out[result.classes];
        vec_result.push_back(result);
        index++;
    }
    return vec_result;
}

void Classification::DrawResults(const std::vector<ClassRes> &results, std::vector<cv::Mat> &vec_img,
                        std::vector<std::string> image_names=std::vector<std::string>()) {
    for (int i = 0; i < (int)vec_img.size(); i++) {
        auto org_img = vec_img[i];
        if (!org_img.data)
            continue;
        auto result = results[i];
        if (!image_names.empty()) {
            std::string rst_name = class_labels[result.classes] + ".jpg";;
            std::cout << rst_name << std::endl;
        }
    }
}
}