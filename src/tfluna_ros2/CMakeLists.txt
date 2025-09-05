cmake_minimum_required(VERSION 3.8)
project(tfluna_ros2)

# Default to C++17 for modern features and performance
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# C++ executables
add_executable(tfluna_publisher_cpp
  src/tfluna_publisher_cpp.cpp
)

add_executable(tfluna_i2c_publisher_cpp
  src/tfluna_i2c_publisher_cpp.cpp
)

ament_target_dependencies(tfluna_publisher_cpp
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
)

ament_target_dependencies(tfluna_i2c_publisher_cpp
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
)

# I2C version uses direct system calls - no additional libraries needed

# Component library for composition
add_library(tfluna_publisher_component SHARED
  src/tfluna_publisher_cpp.cpp
)

ament_target_dependencies(tfluna_publisher_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
)

# Register component
rclcpp_components_register_nodes(tfluna_publisher_component "tfluna_ros2::TFLunaPublisher")

# Install executables
install(TARGETS
  tfluna_publisher_cpp
  tfluna_i2c_publisher_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install component library
install(TARGETS
  tfluna_publisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Python components removed - using C++ only

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
