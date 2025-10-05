cmake_minimum_required(VERSION 3.8)
project(falco_drone_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(OGRE OGRE)
    pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


link_directories(${GAZEBO_LIBRARY_DIRS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${OGRE_LIBRARY_DIRS})
include_directories(${OpenCV_INCLUDE_DIR}
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${OGRE_INCLUDE_DIRS}
                    ${OGRE-Terrain_INCLUDE_DIRS}
                    include)



################## 2. A simple model controller for the quadrotor #############
add_library( plugin_drone SHARED
src/plugin_drone.cpp
src/plugin_drone_private.cpp
src/pid_controller.cpp
include/falco_drone_description/plugin_drone.h
include/falco_drone_description/plugin_drone_private.h
include/falco_drone_description/pid_controller.h
)


target_compile_features(plugin_drone PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(plugin_drone PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(plugin_drone
  "rclcpp"
  "gazebo_ros"
  "gazebo_plugins"
  "gazebo_dev"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(plugin_drone PRIVATE "FALCO_DRONE_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS plugin_drone
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set_target_properties(plugin_drone
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)

################## 3. Real-time drone controller executable #############
add_executable(drone_real_time_controller
    src/drone_real_time_main.cpp
    src/plugin_drone_real_time_exec.cpp
    src/plugin_drone_real_time.cpp
    src/pid_controller.cpp
)

target_include_directories(drone_real_time_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(drone_real_time_controller
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

################## 4. EKF executable #############
add_executable(ekf_node
    src/ekf_main.cpp
    src/ekf.cpp
)

target_include_directories(ekf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(ekf_node Eigen3::Eigen)

ament_target_dependencies(ekf_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
)

################## 5. Real-time EKF executable #############
add_executable(ekf_realtime
    src/ekf_realtime.cpp
    src/ekf.cpp
)

target_include_directories(ekf_realtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(ekf_realtime Eigen3::Eigen)

ament_target_dependencies(ekf_realtime
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
)

install(TARGETS
  drone_real_time_controller
  ekf_node
  ekf_realtime
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(
  include
)
ament_export_libraries(
  plugin_drone
)
ament_export_targets(
  export_${PROJECT_NAME}
)


install(
  DIRECTORY models urdf worlds
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
