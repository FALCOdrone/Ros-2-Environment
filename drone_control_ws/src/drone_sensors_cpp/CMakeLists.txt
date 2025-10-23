cmake_minimum_required(VERSION 3.8)
project(drone_sensors_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Magnetometer simulator node
add_executable(magnetometer_simulator
  src/magnetometer_simulator.cpp
)

ament_target_dependencies(magnetometer_simulator
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Barometer simulator node
add_executable(barometer_simulator
  src/barometer_simulator.cpp
)

ament_target_dependencies(barometer_simulator
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Install targets
install(TARGETS
  magnetometer_simulator
  barometer_simulator
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
