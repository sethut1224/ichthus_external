################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################

cmake_minimum_required(VERSION 3.5)
project(ichthus_v2x)

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

################################################################################
# Find and load build settings from external packages
################################################################################

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kiapi_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/asn
  ${CMAKE_CURRENT_SOURCE_DIR}/include/j2735
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/)

add_executable(ichthus_v2x src/ichthus_v2x_main.cpp)
ament_target_dependencies(ichthus_v2x
  std_msgs
  sensor_msgs
  kiapi_msgs
  rclcpp
)
target_link_libraries(ichthus_v2x 
  libasncodec.a
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

install(TARGETS
  ichthus_v2x
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Find and load build settings from external packages
################################################################################
ament_package()
