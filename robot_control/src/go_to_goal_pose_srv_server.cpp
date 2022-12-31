#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/srv/go_to_goal_pose.hpp"
// #include <robot_msgs/srv/go_to_goal_pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <stdexcept>
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;

class GoToGoalPose : public rclcpp::Node
{
public:
    GoToGoalPose() : Node("go_to_goal_pose_server_node"){
        RCLCPP_INFO(LOGGER, "[GoToGoalPoseServer] Server created...\n");
        move_group_node = rclcpp::Node::make_shared("move_group_interface_node");

        service = this->create_service<robot_msgs::srv::GoToGoalPose>("go_to_goal_pose_server",
                        std::bind(&GoToGoalPose::go_to_goal_pose_clb, this, _1, _2));

        x_min = -4.55; x_max = 4.25;
        xj_min = 0.0; xj_max = 8.80;
        y_min = -1.65; y_max = 1.65;
        yj_min = -1.65; yj_max = 1.65;
        z_min = 1.78; z_max = 4.72;
        zj_min = 0.10; zj_max = 3.04;
    }
    ~GoToGoalPose() {
        RCLCPP_INFO(LOGGER, "[GoToGoalPoseServer] Server destroyed...\n");
    }

private:
    void go_to_goal_pose_clb(const std::shared_ptr<robot_msgs::srv::GoToGoalPose::Request> request,
                            std::shared_ptr<robot_msgs::srv::GoToGoalPose::Response> response);
    double map_vals(double x, double in_min, double in_max, double out_min, double out_max) const{
        if (x < in_min || x > in_max)
            throw std::invalid_argument( "Desired pose is out of joint limits...\n" );
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
private:
    rclcpp::Service<robot_msgs::srv::GoToGoalPose>::SharedPtr service;
    double x_min, x_max;
    double xj_min, xj_max;
    double y_min, y_max;
    double yj_min,  yj_max;
    double z_min, z_max;
    double zj_min, zj_max;
    const rclcpp::Logger LOGGER = rclcpp::get_logger("go_to_goal_server_logger");
public:
    std::shared_ptr<rclcpp::Node> move_group_node;
};

void GoToGoalPose::go_to_goal_pose_clb(const std::shared_ptr<robot_msgs::srv::GoToGoalPose::Request> request,
                        std::shared_ptr<robot_msgs::srv::GoToGoalPose::Response> response){
    RCLCPP_INFO(LOGGER, "[GoToGoalPoseServer] Server callback called...\n");

    double vel_scaling = 1.0, acc_scaling = 1.0;
    int planning_time = 10; // seconds
    if (request->velocity_scaling.data >= 0.0 && request->velocity_scaling.data <= 1.0)
        vel_scaling = request->velocity_scaling.data;
    if (request->acc_scaling.data >= 0.0 && request->acc_scaling.data <= 1.0)
        acc_scaling = request->acc_scaling.data;
    if (request->planning_time.data > 0)
        planning_time = request->planning_time.data;

    // executor.add_node(move_group_node);
    // std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "crane";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    try
    {
        joint_group_positions[0] = map_vals(request->goal_pose.position.x, x_min, x_max, xj_max, xj_min);//x;
        joint_group_positions[1] = map_vals(request->goal_pose.position.z, z_min, z_max, zj_max, zj_min);//z;
        joint_group_positions[2] = map_vals(request->goal_pose.position.y, y_min, y_max, yj_max, yj_min);//y;
        move_group.setJointValueTarget(joint_group_positions);

        response->success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(LOGGER, "Planning %s", response->success ? "SUCCEDED" : "FAILED");
        if (response->success){
            move_group.execute(my_plan);
        }
    }catch(const std::exception& e){
        RCLCPP_INFO(LOGGER, "Error: %s", e.what());
        response->success = false;
    }

    RCLCPP_INFO(LOGGER, "[GoToGoalPoseServer] Server callback finished...\n");
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    rclcpp::Node::SharedPtr go_to_goal_pose_ins1 = std::make_shared<GoToGoalPose>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(go_to_goal_pose_ins1);
    executor.add_node((dynamic_cast<GoToGoalPose*>(go_to_goal_pose_ins1.get()))->move_group_node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}