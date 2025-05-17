cmake_minimum_required(VERSION 3.8)
project(minimal_robot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_ign REQUIRED)           # Ignition integration
find_package(ros_ign_gazebo REQUIRED)    # ros_ign gazebo bridge

# Install your launch files into share/<pkg>/launch
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install your URDF (xacro) files into share/<pkg>/urdf
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install your world files into share/<pkg>/worlds
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# (Optional) If you have mesh models, you could also do:
# install(DIRECTORY models/
#   DESTINATION share/${PROJECT_NAME}/models
# )

# For testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
