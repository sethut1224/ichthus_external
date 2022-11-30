#/*  ===========================================================================
#    Copyright 2022. The ICHTHUS Project. All Rights Reserved.
#    Jaemin Lee (xoz1206@gmail.com),
#    Youngjun Han (young@ssu.ac.kr)
#    Vision System Laboratory, Soongsil University.
#    [Licensed under the MIT License]  
#    ===========================================================================*/
cmake_minimum_required(VERSION 3.14)
project(mission_manager_am)

find_package(autoware_cmake REQUIRED)
find_package(kiapi_msgs REQUIRED)
find_package(PCL REQUIRED) # component
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)

autoware_package()

# See ndt_omp package for documentation on why PCL is special
find_package(PCL REQUIRED COMPONENTS common filters)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_package(OpenCV REQUIRED)

find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

add_definitions(${GeographicLib_DEFINITIONS})

include_directories(
  ${GeographicLib_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_DEPENDENCIES
  autoware_auto_mapping_msgs
  autoware_auto_perception_msgs
  autoware_auto_system_msgs
  tier4_autoware_utils
  visualization_msgs
  tier4_perception_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_eigen
  geometry_msgs
  kiapi_msgs
  mission_manager_msgs
  lanelet2
  lanelet2_core
  lanelet2_io
  lanelet2_extension
  route_handler
  behavior_path_planner
  behavior_velocity_planner
  euclidean_cluster

)

# ament_auto_add_library(mission_manager_am SHARED
#   src/mission_manager_am.cpp
# )

ament_auto_add_executable(mission_manager_am
  src/mission_manager_am.cpp
)

# target_link_libraries(mission_manager_am
#   mission_manager_am
# )

ament_target_dependencies(mission_manager_am ${${PROJECT_NAME}_DEPENDENCIES})

target_include_directories(mission_manager_am
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# For using message definitions from the same package
# if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
#   rosidl_target_interfaces(mission_manager_am
#     ${PROJECT_NAME} "rosidl_typesupport_cpp")
# else()
#   rosidl_get_typesupport_target(
#     cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
#   target_link_libraries(mission_manager_am "${cpp_typesupport_target}")
# endif()

# PCL dependencies - `ament_target_dependencies` doesn't respect the
# components/modules selected above and only links in `common` ,so we need
# to do this manually.
target_compile_definitions(mission_manager_am PRIVATE ${PCL_DEFINITIONS})
target_include_directories(mission_manager_am PRIVATE ${PCL_INCLUDE_DIRS})
# Unfortunately, this one can't be PRIVATE because only the plain or only the
# keyword (PRIVATE) signature of target_link_libraries can be used for one
# target, not both. The plain signature is already used inside
# `ament_target_dependencies` and possibly rosidl_target_interfaces.
target_link_libraries(mission_manager_am ${PCL_LIBRARIES} ${GeographicLib_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_directories(mission_manager_am PRIVATE ${PCL_LIBRARY_DIRS})

# ament_auto_add_executable(empty_objects_publisher
#   src/empty_objects_publisher.cpp
# )

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)