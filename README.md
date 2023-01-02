# GANTRY_ROBOT_CONTROL_ROS-
The purpose of this project is to create 3D gantry robot and control it via MoveIt2 in ROS2 (Humble) in C++. Overall control has been implemented over Behavior Trees.

## Before running
>1. **sudo apt install ros-humble-rmw-cyclonedds-cpp**
>2. **export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp** (run this for each terminal you use for ROS2)
>3. Source the terminals you use by doing: **source install/setup.bash**

## How to run
>1. In a terminal run: **ros2 launch robot_moveit_config demo.launch.py**
>2. In another terminal **run: ros2 run robot_control robot_control_bt**
