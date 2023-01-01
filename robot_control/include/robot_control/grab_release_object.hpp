#ifndef GRAB_RELEASE_OBJECT_HPP_
#define GRAB_RELEASE_OBJECT_HPP_

#include "behaviortree_cpp_v3/action_node.h"

#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

class GrabReleaseObjectNode : public BT::SyncActionNode
{
public:
    GrabReleaseObjectNode(const std::string& action_name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(action_name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<rclcpp::Node>>("node_handle", "Node to register"),
            BT::InputPort<std::string>("object_id", "ID of object to be grasped"),
            BT::InputPort<bool>("grasp", "Grasp[1] or release[0]"),
        };
    }

    BT::NodeStatus tick() override
    {
        if (!getInput("node_handle", move_group_node_)){
            throw BT::RuntimeError("Missing parameter [node_handle]");
        }
        std::string object_id;
        if (!getInput("object_id", object_id)){
            throw BT::RuntimeError("Missing parameter [object_id]");
        }
        bool is_grasp = true;
        if (!getInput("grasp", is_grasp)){
            throw BT::RuntimeError("Missing parameter [grasp]");
        }

        static const std::string PLANNING_GROUP = "crane";

        moveit::planning_interface::MoveGroupInterface move_group(move_group_node_, PLANNING_GROUP);
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        if (is_grasp){
            std::vector<std::string> touch_links;
            touch_links.push_back("virtual_gripper_link");
            touch_links.push_back("slider_ud_link");
            move_group.attachObject(object_id, "virtual_gripper_link", touch_links);
        }else{
            move_group.detachObject(object_id);
        }

        std::this_thread::sleep_for(std::chrono::seconds(2));

        return BT::NodeStatus::SUCCESS;
    }
private:
    std::shared_ptr<rclcpp::Node> move_group_node_;
};

#endif