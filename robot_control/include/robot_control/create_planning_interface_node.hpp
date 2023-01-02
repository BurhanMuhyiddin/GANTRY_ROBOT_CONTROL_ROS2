#ifndef CREATE_PLANNING_INTERFACE_NODE_HPP_
#define CREATE_PLANNING_INTERFACE_NODE_HPP_

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class CreatePlanningInterfaceNode : public BT::SyncActionNode
{
public:
    CreatePlanningInterfaceNode(const std::string& action_name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(action_name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<rclcpp::Node>>("node_handle", "Node to register"),
            BT::InputPort<std::string>("planning_group", "Planning group name"),
            BT::OutputPort<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("movegroup_interface", "Movegroup Interface"),
            BT::OutputPort<std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>>("planning_interface", "Planning Interface"),
        };
    }

    BT::NodeStatus tick() override
    {
        std::shared_ptr<rclcpp::Node> move_group_node;
        if (!getInput("node_handle", move_group_node)){
            throw BT::RuntimeError("Missing parameter [node_handle]");
        }
        std::string PLANNING_GROUP;
        if (!getInput("planning_group", PLANNING_GROUP)){
            throw BT::RuntimeError("Missing parameter [planning_group]");
        }
        
        auto movegroup_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, PLANNING_GROUP);
        auto planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        if (!setOutput("movegroup_interface", movegroup_interface)){
            throw BT::RuntimeError("Failed to set output port of [movegroup_interface].");
        }
        if (!setOutput("planning_interface", planning_scene_interface)){
            throw BT::RuntimeError("Failed to set output port of [planning_interface].");
        }

        return BT::NodeStatus::SUCCESS;
    }
};

#endif