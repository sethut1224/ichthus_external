cmake_minimum_required(VERSION 3.8)
project(ichthus_vehicle_interface)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
  endif()
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  endif()

  # Ignore PCL errors in Clang
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-gnu-anonymous-struct -Wno-nested-anon-types)
  endif()


# find dependencies
find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_library(ichthus_vehicle_interface SHARED
  include/ichthus_vehicle_interface/ichthus_vehicle_interface.hpp
  src/ichthus_vehicle_interface.cpp
)

target_compile_options(ichthus_vehicle_interface PRIVATE -Wno-error=old-style-cast)
rclcpp_components_register_node(ichthus_vehicle_interface
  PLUGIN "ichthus_vehicle_interface::IchthusVehicleInterfaceNode"
  EXECUTABLE ichthus_vehicle_interface_node
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

ament_auto_package(INSTALL_TO_SHARE
)
  
