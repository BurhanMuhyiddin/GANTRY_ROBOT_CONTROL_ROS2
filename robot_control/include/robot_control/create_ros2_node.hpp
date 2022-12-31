// Copyright (c) 2019 Samsung Research America
// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_BEHAVIOR_TREE__ACTION__CREATE_ROS2_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__CREATE_ROS2_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_behavior_tree/node_thread.hpp"

namespace ros2_behavior_tree
{

class CreateROS2Node : public BT::SyncActionNode
{
public:
  CreateROS2Node(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("node_name", "The name of the ROS2 node to create"),
      BT::InputPort<std::string>("namespace", "The namespace in which to create the node"),
      BT::InputPort<bool>("spin", "Whether to spin this node on a separate thread"),
      BT::OutputPort<std::shared_ptr<rclcpp::Node>>("node_handle",
        "The node handle of the created node")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string node_name;
    if (!getInput("node_name", node_name)) {
      throw BT::RuntimeError("Missing parameter [node_name] in CreateROS2Node");
    }

    std::string ns;
    if (!getInput("namespace", ns)) {
      throw BT::RuntimeError("Missing parameter [namespace] in CreateROS2Node");
    }

    bool spin_thread = false;
    if (!getInput("spin", spin_thread)) {
      throw BT::RuntimeError("Missing parameter [spin] in CreateROS2Node");
    }

    auto node = std::make_shared<rclcpp::Node>(node_name, ns);

    if (!setOutput("node_handle", node)) {
      throw BT::RuntimeError("Failed to set output port value [node_handle] in CreateROS2Node");
    }

    if (spin_thread) {
      node_thread_ = std::make_unique<ros2_behavior_tree::NodeThread>(node);
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<ros2_behavior_tree::NodeThread> node_thread_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__CREATE_ROS2_NODE_HPP_
