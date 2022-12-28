#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <stdexcept>

// #include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

double map_vals(double x, double in_min, double in_max, double out_min, double out_max) {
    if (x < in_min || x > in_max)
        throw std::invalid_argument( "Desired pose is out of joint limits...\n" );
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "crane";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    double x_min = -4.55, x_max = 4.25;
    double xj_min = 0.0, xj_max = 8.80;
    double y_min = -1.65, y_max = 1.65;
    double yj_min = -1.65, yj_max = 1.65;
    double z_min = 1.78, z_max = 4.72;
    double zj_min = 0.10, zj_max = 3.04;

    // offset: 1.52

    try
    {
        joint_group_positions[0] = map_vals(3.127, x_min, x_max, xj_max, xj_min);//x;
        joint_group_positions[1] = map_vals(1.78, z_min, z_max, zj_max, zj_min);//z;
        joint_group_positions[2] = map_vals(0.223, y_min, y_max, yj_max, yj_min);//y;
        move_group.setJointValueTarget(joint_group_positions);

        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

        move_group.execute(my_plan);
    }catch(const std::exception& e){
        RCLCPP_INFO(LOGGER, "Error: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}