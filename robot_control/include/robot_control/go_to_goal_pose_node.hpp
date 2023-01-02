#ifndef GO_TO_GOAL_POSE_NODE_HPP_
#define GO_TO_GOAL_POSE_NODE_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include "ros2_behavior_tree/node_thread.hpp"

#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

class GoToGoalPoseNode : public BT::SyncActionNode
{
public:
    GoToGoalPoseNode(const std::string& action_name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(action_name, config)
    {
        x_min_ = -4.55; x_max_ = 4.25;
        xj_min_ = 0.0; xj_max_ = 8.80;
        y_min_ = -1.65; y_max_ = 1.65;
        yj_min_ = -1.65; yj_max_ = 1.65;
        z_min_ = 0.26; z_max_= 3.19;
        zj_min_ = 0.10; zj_max_ = 3.04;
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<rclcpp::Node>>("node_handle", "Node to register"),
            BT::InputPort<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("movegroup_interface", "MoveGroup interface"),
            BT::InputPort<std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>>("planning_interface", "Planning Interface"),
            BT::InputPort<double>("gx", "X Goal pose"),
            BT::InputPort<double>("gy", "Y Goal pose"),
            BT::InputPort<double>("gz", "Z Goal pose"),
        };
    }

    BT::NodeStatus tick() override
    {
        std::shared_ptr<rclcpp::Node> move_group_node;
        if (!getInput("node_handle", move_group_node)){
            throw BT::RuntimeError("Missing parameter [node_handle]");
        }
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        if (!getInput("movegroup_interface", move_group)){
            throw BT::RuntimeError("Missing parameter [movegroup_interface]");
        }
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
        if (!getInput("planning_interface", planning_scene_interface)){
            throw BT::RuntimeError("Missing parameter [planning_interface]");
        }
        double gx, gy, gz;
        if (!getInput("gx", gx) || !getInput("gy", gy) || !getInput("gz", gz)){
            throw BT::RuntimeError("Please specify goal pose.");
        }

        RCLCPP_INFO(rclcpp::get_logger("GoToGoalPoseNode"), "Callback started...\n");

        try
        {
            auto note_thread = std::make_unique<ros2_behavior_tree::NodeThread>(move_group_node);

            move_group->setMaxVelocityScalingFactor(1.0);
            move_group->setMaxAccelerationScalingFactor(1.0);

            std::vector<double> joint_group_positions(3, 0.0);
            joint_group_positions[0] = map_vals(gx, x_min_, x_max_, xj_max_, xj_min_);//x;
            joint_group_positions[1] = map_vals(gz, z_min_, z_max_, zj_max_, zj_min_);//z;
            joint_group_positions[2] = map_vals(gy, y_min_, y_max_, yj_max_, yj_min_);//y;
            move_group->setJointValueTarget(joint_group_positions);

            RCLCPP_INFO(rclcpp::get_logger("GoToGoalPoseNode"), "Planning started...\n");
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            RCLCPP_INFO(rclcpp::get_logger("GoToGoalPoseNode"), "Planning %s", success ? "SUCCEDED" : "FAILED");
            if (success){
                RCLCPP_INFO(rclcpp::get_logger("GoToGoalPoseNode"), "Execution started...\n");
                move_group->execute(my_plan);
            }else{
                return BT::NodeStatus::FAILURE;
            }
        }catch(const std::exception& e){
            RCLCPP_INFO(rclcpp::get_logger("GoToGoalPoseNode"), "Error: %s", e.what());
            return BT::NodeStatus::FAILURE;
        }

        std::this_thread::sleep_for(std::chrono::seconds(2));

        RCLCPP_INFO(rclcpp::get_logger("GoToGoalPoseNode"), "Callback finished...\n");

        return BT::NodeStatus::SUCCESS;
    }
private:
    double map_vals(double x, double in_min, double in_max, double out_min, double out_max) const{
        if (x < in_min || x > in_max)
            throw std::invalid_argument( "Desired pose is out of joint limits...\n" );
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
private:
    double x_min_, x_max_;
    double xj_min_, xj_max_;
    double y_min_, y_max_;
    double yj_min_,  yj_max_;
    double z_min_, z_max_;
    double zj_min_, zj_max_;
};

#endif