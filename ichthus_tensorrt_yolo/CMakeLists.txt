#  ===========================================================================
#    Copyright 2022. The ICHTHUS Project. All Rights Reserved.
#    Hyewon Bang (hwbang0815@naver.com) and Youngjoon Han (young@ssu.ac.kr).
#    Vision Systems Laboratory, Soongsil University.
#    added by ICHTHUS, Hyewon Bang on 20221026
#    [Licensed under the MIT License]
#    ===========================================================================
cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 14)

project(ichthus_tensorrt_yolo)

find_package(autoware_cmake REQUIRED)
autoware_package()

add_compile_options(-Wno-deprecated-declarations)
add_compile_options(-fPIC)

option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)

find_package(OpenCV REQUIRED)

if(OpenCV_FOUND)
    message("Found_OpenCV")
endif()

# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if(CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
  )
  if(CUDA_VERBOSE)
    message(STATUS "CUDA is available!")
    message(STATUS "CUDA Libs: ${CUDA_LIBRARIES}")
    message(STATUS "CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif()
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER NAMES nvinfer)
find_library(NVONNXPARSER nvonnxparser)
find_library(NVINFER_PLUGIN NAMES nvinfer_plugin)
if(NVINFER AND NVONNXPARSER AND NVINFER_PLUGIN)
  if(CUDA_VERBOSE)
    message(STATUS "TensorRT is available!")
    message(STATUS "NVINFER: ${NVINFER}")
    message(STATUS "NVPARSERS: ${NVPARSERS}")
    message(STATUS "NVINFER_PLUGIN: ${NVINFER_PLUGIN}")
    message(STATUS "NVONNXPARSER: ${NVONNXPARSER}")
  endif()
  set(TRT_AVAIL ON)
else()
  message("TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

# set flags for CUDNN availability
option(CUDNN_AVAIL "CUDNN available" OFF)
# try to find the CUDNN module
find_library(CUDNN_LIBRARY
NAMES libcudnn.so${__cudnn_ver_suffix} libcudnn${__cudnn_ver_suffix}.dylib ${__cudnn_lib_win_name}
PATHS $ENV{LD_LIBRARY_PATH} ${__libpath_cudart} ${CUDNN_ROOT_DIR} ${PC_CUDNN_LIBRARY_DIRS} ${CMAKE_INSTALL_PREFIX}
PATH_SUFFIXES lib lib64 bin
DOC "CUDNN library."
)
if(CUDNN_LIBRARY)
  if(CUDA_VERBOSE)
    message(STATUS "CUDNN is available!")
    message(STATUS "CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  endif()
  set(CUDNN_AVAIL ON)
else()
  message("CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()

# create calib image directory for int8 calibration
set(CALIB_IMAGE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/calib_image")
if(NOT EXISTS "${CALIB_IMAGE_PATH}")
  execute_process(COMMAND mkdir -p ${CALIB_IMAGE_PATH})
endif()

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  include_directories(
    include
    lib/include
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
  )


  ### yolo ###
  ament_auto_add_library(yolov4 SHARED
    lib/src/trt_yolo.cpp
    lib/src/common.cpp
    lib/src/Yolov4.cpp
    lib/src/classification.cpp
    lib/src/detection.cpp
  ) 
  ament_auto_add_library(yolov7 SHARED
    lib/src/common.cpp
    lib/src/yolov7.cpp
    lib/src/YOLO.cpp
    # lib/src/classification.cpp
    lib/src/detection.cpp
  )  
  ament_auto_add_library(factory SHARED
    lib/src/build.cpp
  )
  ament_auto_add_library(yolo SHARED
    lib/src/trt_yolo.cpp
  )

  target_include_directories(yolo PUBLIC
    lib/include
  )
  
  target_link_libraries(yolov4
    ${NVINFER}
    ${NVONNXPARSER}
    ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDNN_LIBRARY}
  )
  target_link_libraries(yolo
    ${NVINFER}
    ${NVONNXPARSER}
    ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDNN_LIBRARY}
    yolov4
    yolov7
  )

  ament_auto_add_library(ichthus_tensorrt_yolo_nodelet SHARED
    src/nodelet.cpp
  )

  target_link_libraries(ichthus_tensorrt_yolo_nodelet
    ${OpenCV_LIBS}
    ${NVINFER}
    ${NVONNXPARSER}
    ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDNN_LIBRARY}
    yolo
    factory
    yolov4
    yolov7
  )

  rclcpp_components_register_node(ichthus_tensorrt_yolo_nodelet
    PLUGIN "object_recognition::IchthusTensorrtYoloNodelet"
    EXECUTABLE ichthus_tensorrt_yolo_node
  )

  ament_auto_package(INSTALL_TO_SHARE
    config
    data
    launch
  )

  install(
    TARGETS
      factory
      yolov4
      yolov7
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )

else()
  message("TensorrtYolo won't be built, CUDA and/or TensorRT were not found.")
  ament_auto_package(INSTALL_TO_SHARE
    config
    data
    launch
  )
endif()
