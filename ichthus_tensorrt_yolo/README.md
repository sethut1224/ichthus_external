# ichthus_tensorrt_yolo
This repository refactoring [tensorrt_inference](https://github.com/linghu8812/tensorrt_inference) to ROS2.

## Input
|param name | Type              |description     |
|-----------|-------------------|----------------|
|`image_raw`  | `sensor_msgs/Image` | The input image|

## Output
|param name  | Type |description |
|------------|------|------------|
|`rois` |  `tier4_perception_msgs/DetectedObjectsWithFeature` | The detected objects with 2D bounding boxes

## Environment Setting
- autoware.universe
- opencv >= 4.3.0
- tensorrt >= 8.0
- onnx

## How To Guides
1. Create trt file<br>
`Be sure to do it on your target device.`<br>
* [yolov4 Network darknet => ONNX => TensorRT](https://github.com/Bangglll/tensorrt_inference/tree/master/project/yolov7)
* [yolov7 pytorch => ONNX => TensorRT](https://github.com/Bangglll/tensorrt_inference/tree/master/project/yolov7)

Save the generated file to the data folder.<br>
Write config information in the config folder.
