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
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_viewer REQUIRED)
find_package(simple_node REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(robot_msgs REQUIRED)

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
  ament_cmake
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
  control_msgs
  robot_msgs
  yasmin
  yasmin_ros
  yasmin_viewer)

# robot_position_control
add_executable(robot_position_control
  src/robot_position_control.cpp)
target_include_directories(robot_position_control
  PUBLIC include)
ament_target_dependencies(robot_position_control ${DEPENDENCIES})
install(TARGETS
  robot_position_control
  DESTINATION lib/${PROJECT_NAME})

# go_to_goal_pose_srv_server
add_executable(go_to_goal_pose_srv_server
  src/go_to_goal_pose_srv_server.cpp)
target_include_directories(go_to_goal_pose_srv_server
  PUBLIC include)
ament_target_dependencies(go_to_goal_pose_srv_server ${DEPENDENCIES})
install(TARGETS
go_to_goal_pose_srv_server
  DESTINATION lib/${PROJECT_NAME})

# robot_control_sm
add_executable(robot_control_sm
  src/robot_control_sm.cpp)
target_include_directories(robot_control_sm
  PUBLIC include)
ament_target_dependencies(robot_control_sm ${DEPENDENCIES})
install(TARGETS
robot_control_sm
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

# Python
install(PROGRAMS
  scripts/control_sm.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
