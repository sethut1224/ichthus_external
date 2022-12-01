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

/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef TRT_YOLO_HPP_
#define TRT_YOLO_HPP_

#include <opencv2/opencv.hpp>
#include "NvInfer.h"
#include "common.h"
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace yolo
{
struct Deleter
{
  template <typename T>
  void operator()(T * obj) const
  {
    if (obj) {
      delete obj;
    }
  }
};

#if 1//modified by ICHTHUS, Hyewon Bang on 20221026
struct ClassRes{
    int classes;
    float prob;
};
struct Bbox : ClassRes{
    float x;
    float y;
    float w;
    float h;
};
struct DetectRes {
    std::vector<Bbox> det_results;
};
#endif

struct Config
{
  std::string engine_file;
  std::string labels_file;
  std::vector<std::vector<int>> anchors;
  std::string yolo_version;
  std::vector<int> strides;
  std::vector<int> num_anchors;
  int BATCH_SIZE;
  int INPUT_CHANNEL;
  int IMAGE_WIDTH;
  bool agnostic;
  int IMAGE_HEIGHT;
  std::string image_order;
  std::string channel_order;
  std::vector<float> img_mean;
  std::vector<float> img_std;
  float alpha;
  float nms_threshold;
  float obj_threshold;
  std::string resize;
};

#if 1//modified by ICHTHUS, Hyewon Bang on 20221026
class Model
{
public:
    explicit Model(const Config &config);
    ~Model();
    void LoadEngine();
    virtual void InferenceFolder(std::vector<cv::Mat> &vec_img,std::vector<DetectRes> &detections)  = 0;

protected:
    void ReadTrtFile();
    void OnnxToTRTModel();
    std::vector<float> PreProcess(std::vector<cv::Mat> &image);
    void ModelInference(std::vector<float> image_data, float *output);
    std::string onnx_file;
    std::string engine_file;
    std::string labels_file;
    int BATCH_SIZE;
    int INPUT_CHANNEL;
    int IMAGE_WIDTH;
    int IMAGE_HEIGHT;
    nvinfer1::ICudaEngine *engine = nullptr;
    nvinfer1::IExecutionContext *context = nullptr;
    void *buffers[2];
    std::vector<int64_t> bufferSize;
    cudaStream_t stream;
    int outSize;
    std::string image_order;
    std::string channel_order;
    std::vector<float> img_mean;
    std::vector<float> img_std;
    float alpha;
    std::string resize;
};
#endif
}  // namespace yolo

#endif  // TRT_YOLO_HPP_