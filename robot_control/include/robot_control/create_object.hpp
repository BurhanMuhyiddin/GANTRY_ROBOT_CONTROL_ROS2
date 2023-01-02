#ifndef CREATE_OBJECT_HPP_
#define CREATE_OBJECT_HPP_

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

class AddObjectNode : public BT::SyncActionNode
{
public:
    AddObjectNode(const std::string& action_name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(action_name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<rclcpp::Node>>("node_handle", "Node to register"),
            BT::InputPort<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("movegroup_interface", "MoveGroup interface"),
            BT::InputPort<std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>>("planning_interface", "Planning Interface"),
            BT::InputPort<double>("px", "X position"),
            BT::InputPort<double>("py", "Y position"),
            BT::InputPort<double>("pz", "Z position"),
            BT::InputPort<double>("w", "Box width"),
            BT::InputPort<double>("l", "Box length"),
            BT::InputPort<double>("h", "Box height"),
            BT::OutputPort<std::string>("object_id", "Id of created object"),
            BT::OutputPort<int>("object_count", "Count of created object"),
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
        double px, py, pz;
        if (!getInput("px", px) || !getInput("py", py) || !getInput("pz", pz)){
            throw BT::RuntimeError("Please specify box pose correctly.");
        }
        double w, l, h;
        if (!getInput("w", w) || !getInput("l", l) || !getInput("h", h)){
            throw BT::RuntimeError("Please specify box dimensions correctly.");
        }

        auto note_thread = std::make_unique<ros2_behavior_tree::NodeThread>(move_group_node);

        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = move_group->getPlanningFrame();

        collision_object.id = "box" + std::to_string(object_count_);

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = w;
        primitive.dimensions[primitive.BOX_Y] = l;
        primitive.dimensions[primitive.BOX_Z] = h;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = px;
        box_pose.position.y = py;
        box_pose.position.z = pz;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object);

        planning_scene_interface->addCollisionObjects(collision_objects);
        // planning_scene_interface->applyCollisionObject(collision_object);

        object_count_++;

        if (!setOutput("object_id", collision_object.id)){
            throw BT::RuntimeError("Failed to set output port of [object_id].");
        }
        if (!setOutput("object_count", object_count_)){
            throw BT::RuntimeError("Failed to set output port of [object_count].");
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));

        return BT::NodeStatus::SUCCESS;
    }
private:
    static int object_count_;
};

int AddObjectNode::object_count_ = 0;

#endif