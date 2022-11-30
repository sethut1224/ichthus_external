# ===========================================================================
# Copyright 2022. The ICHTHUS Project. All Rights Reserved.
# Eunseo Choi (eunseo.choi.d@gmail.com) and Kanghee Kim (kim.kanghee@gmail.com).
# Mobility Intelligence & Computing Systems Laboratory, Soongsil University.
# [Licensed under the Apache License, Version 2.0]
# ===========================================================================

cmake_minimum_required(VERSION 3.14)
project(sim_stage_mission)

find_package(autoware_cmake REQUIRED)
find_package(kiapi_msgs REQUIRED)

autoware_package()

# See ndt_omp package for documentation on why PCL is special
find_package(PCL REQUIRED COMPONENTS common filters)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

add_definitions(${GeographicLib_DEFINITIONS})

include_directories(
  ${GeographicLib_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_DEPENDENCIES
  autoware_auto_perception_msgs
  tier4_perception_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  geometry_msgs
  kiapi_msgs
  lanelet2_core
  visualization_msgs
)

# ament_auto_add_library(mission_manager_am SHARED
#   src/mission_manager_am.cpp
# )

ament_auto_add_executable(sim_mission_gen
  src/sim_mission_gen.cpp
)

ament_auto_add_executable(sim_mission_pub
  src/sim_mission_pub.cpp
)


# target_link_libraries(mission_manager_am
#   mission_manager_am
# )

ament_target_dependencies(sim_mission_gen ${${PROJECT_NAME}_DEPENDENCIES})
ament_target_dependencies(sim_mission_pub ${${PROJECT_NAME}_DEPENDENCIES})

target_include_directories(sim_mission_gen
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(sim_mission_pub
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
target_compile_definitions(sim_mission_gen PRIVATE ${PCL_DEFINITIONS})
target_include_directories(sim_mission_gen PRIVATE ${PCL_INCLUDE_DIRS})


# Unfortunately, this one can't be PRIVATE because only the plain or only the
# keyword (PRIVATE) signature of target_link_libraries can be used for one
# target, not both. The plain signature is already used inside
# `ament_target_dependencies` and possibly rosidl_target_interfaces.
target_link_libraries(sim_mission_gen ${PCL_LIBRARIES} ${GeographicLib_LIBRARIES})
target_link_directories(sim_mission_gen PRIVATE ${PCL_LIBRARY_DIRS})

target_compile_definitions(sim_mission_pub PRIVATE ${PCL_DEFINITIONS})
target_include_directories(sim_mission_pub PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(sim_mission_pub ${PCL_LIBRARIES} ${GeographicLib_LIBRARIES})
target_link_directories(sim_mission_pub PRIVATE ${PCL_LIBRARY_DIRS})

# ament_auto_add_executable(empty_objects_publisher
#   src/empty_objects_publisher.cpp
# )


ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
