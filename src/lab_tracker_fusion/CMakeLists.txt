cmake_minimum_required(VERSION 3.8)
project(lab_tracker_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python executable nodes into lib/<package> so `ros2 run` finds them.
install(PROGRAMS
  scripts/single_camera_tag_to_robot_pose_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME single_camera_tag_to_robot_pose_node
)

install(PROGRAMS
  scripts/multi_camera_tracker_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME multi_camera_tracker_node
)

# Install config files from the workspace-root configs/ directory
# (two levels up from this package: src/lab_tracker_fusion/ → workspace root).
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../configs/
  DESTINATION share/${PROJECT_NAME}/configs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
