#ifndef TENSORRT_INFERENCE_SCALED_YOLOV4_H
#define TENSORRT_INFERENCE_SCALED_YOLOV4_H

#include "YOLO.h"
namespace yolo
{
class ScaledYOLOv4 : public YOLO {
public:
    explicit ScaledYOLOv4(const Config &config);// Modified by Heywon on 2022/10/26
};
}
#endif //TENSORRT_INFERENCE_SCALED_YOLOV4_H
