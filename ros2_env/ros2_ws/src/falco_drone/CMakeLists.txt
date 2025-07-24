cmake_minimum_required(VERSION 3.8)
project(falco_drone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  PROGRAMS
    scripts/ignition_drone_controller.py
    scripts/drone_controller.py
    scripts/teleop_drone.py
    scripts/ps2cmd_vel.py
    scripts/simple_gazebo_controller.py
    scripts/teleop_quadrotor.py
    scripts/cmd_vel_to_wrench.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()