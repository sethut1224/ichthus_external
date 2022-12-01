#ifndef TENSORRT_INFERENCE_YOLOV7_H
#define TENSORRT_INFERENCE_YOLOV7_H

#include "YOLO.h"

namespace yolo
{
class yolov7 : public YOLO {
public:
    explicit yolov7(const Config& config);// Modified by Heywon on 2022/10/26
};
}
#endif //TENSORRT_INFERENCE_YOLOV7_H
