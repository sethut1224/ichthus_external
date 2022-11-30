#ifndef TENSORRT_INFERENCE_SCALED_YOLOV4_H
#define TENSORRT_INFERENCE_SCALED_YOLOV4_H

#include "YOLO.h"
namespace yolo
{
class ScaledYOLOv4 : public YOLO {
public:
    explicit ScaledYOLOv4(const Config &config);
};
}
#endif //TENSORRT_INFERENCE_SCALED_YOLOV4_H
