#  ===========================================================================
#    Copyright 2022. The ICHTHUS Project. All Rights Reserved.
#    Hyewon Bang (hwbang0815@naver.com) and Jamin Lee (xoz1206@gmail.com), Youngjoon Han (young@ssu.ac.kr).
#    Vision Systems Laboratory, Soongsil University.
#    added by ICHTHUS, Hyewon Bang on 20221026
#    ===========================================================================

cmake_minimum_required(VERSION 3.8)
project(pixel_cloud_fusion)

find_package(autoware_cmake REQUIRED)
autoware_package()
if(NOT CMAKE_CXX_STANDARD)
  # set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(OpenMP)
find_package(euclidean_cluster REQUIRED)
find_package(Eigen REQUIRED)
find_package(shape_estimation REQUIRED)
find_package(object_merger REQUIRED)

ament_auto_find_build_dependencies()

find_package(Eigen3 QUIET)

if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
    include
    # lib/include
    # ${OpenCV_INCLUDE_DIRS}
    # ${CUDA_INCLUDE_DIRS}
  )
ament_auto_add_library(pixel_cloud_fusion_nodelet SHARED
    src/nodelet.cpp
  )
rclcpp_components_register_node(pixel_cloud_fusion_nodelet
    PLUGIN "pcfusion::PixelCloudFusionNodelet"
    EXECUTABLE pixel_cloud_fusion_node
  )

target_link_libraries(pixel_cloud_fusion_nodelet
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES} # pcl

)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

ament_auto_package(INSTALL_TO_SHARE
    launch
  )
endif()

# ament_package()
