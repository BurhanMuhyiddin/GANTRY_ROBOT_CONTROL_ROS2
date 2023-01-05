cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ros2_behavior_tree_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(ros2_behavior_tree REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
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

# C++
set(DEPENDENCIES
  behaviortree_cpp_v3
  ros2_behavior_tree_msgs
  ros2_behavior_tree
  ament_cmake
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
  control_msgs
  robot_msgs
  geometry_msgs)

# go_to_goal_pose_srv_server
add_executable(go_to_goal_pose_srv_server
  src/go_to_goal_pose_srv_server.cpp)
target_include_directories(go_to_goal_pose_srv_server
  PUBLIC include)
ament_target_dependencies(go_to_goal_pose_srv_server ${DEPENDENCIES})
install(TARGETS
go_to_goal_pose_srv_server
  DESTINATION lib/${PROJECT_NAME})

# go_to_goal_pose_test_client
add_executable(go_to_goal_pose_test_client
  src/go_to_goal_pose_test_client.cpp)
target_include_directories(go_to_goal_pose_test_client
  PUBLIC include)
ament_target_dependencies(go_to_goal_pose_test_client ${DEPENDENCIES})
install(TARGETS
go_to_goal_pose_test_client
  DESTINATION lib/${PROJECT_NAME})

# robot_control_bt
add_executable(robot_control_bt
  src/robot_control_bt.cpp)
target_include_directories(robot_control_bt
  PUBLIC include)
ament_target_dependencies(robot_control_bt ${DEPENDENCIES})
install(TARGETS
robot_control_bt
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)

# Python
# install(PROGRAMS
#   scripts/control_sm.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
