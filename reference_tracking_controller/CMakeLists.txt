cmake_minimum_required(VERSION 3.5)
project(reference_tracking_controller)

# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(reference_tracking_controller src/reference_tracking_controller.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
