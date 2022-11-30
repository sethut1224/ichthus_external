# ===========================================================================
# Copyright 2022. The ICHTHUS Project. All Rights Reserved.
# Sumin In (ism0705@naver.com),
# Youngjun Han (young@ssu.ac.kr)
# Vision System Laboratory, Soongsil University.
# added by ICHTHUS, Sumin In on 20221026
# [Licensed under the MIT License]  
# ===========================================================================

cmake_minimum_required(VERSION 3.5)
project(lte_mission)

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
find_package(rclcpp REQUIRED)
find_package(kiapi_msgs REQUIRED)

include_directories(include
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/)

add_executable(lte_mission 
  src/main.cpp
  src/node.cpp
)
ament_target_dependencies(lte_mission
  kiapi_msgs
  std_msgs
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  lte_mission
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME})

ament_package()
