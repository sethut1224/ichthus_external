# ===========================================================================
# Copyright 2022. The ICHTHUS Project. All Rights Reserved.
# Sumin In (ism0705@naver.com),
# Youngjun Han (young@ssu.ac.kr)
# Vision System Laboratory, Soongsil University.
# added by ICHTHUS, Sumin In on 20221026
# [Licensed under the MIT License]  
# ===========================================================================

cmake_minimum_required(VERSION 3.5)
project(kiapi_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/msg/)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CurrentItem.msg"
  "msg/ItemData.msg"
  "msg/MissionListStage2.msg"
  "msg/MissionData.msg"
  "msg/MissionRouteData.msg"
  "msg/MissionListStage1.msg"
  "msg/Pvdinfo.msg"
  "msg/V2xinfo.msg"
  "msg/Mylocation.msg"
  DEPENDENCIES builtin_interfaces 
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
endif()

ament_package()