#ifndef GO_TO_GOAL_POSE_CLIENT_GPP_
#define GO_TO_GOAL_POSE_CLIENT_GPP_

#include <memory>
#include <string>

#include "ros2_behavior_tree/ros2_service_client_node.hpp"
#include "ros2_behavior_tree/ros2_async_service_client_node.hpp"
#include "robot_msgs/srv/go_to_goal_pose.hpp"

using GoToGoalPose = robot_msgs::srv::GoToGoalPose;

class GoToGoalposeClient : public ros2_behavior_tree::ROS2ServiceClientNode<GoToGoalPose>
{
public:
    GoToGoalposeClient(const std::string &name, const BT::NodeConfiguration &config)
    : ROS2ServiceClientNode<GoToGoalPose>(name, config)
    {
    }


    static BT::PortsList providedPorts()
    {
        return augment_basic_ports({
            BT::InputPort<double>("t_x", "x position"),
            BT::InputPort<double>("t_y", "y position"),
            BT::InputPort<double>("t_z", "z position"),
            BT::OutputPort<bool>("success", "is successful")
        });
    }

    void read_input_ports(std::shared_ptr<GoToGoalPose::Request> request) override
    {
        if (!getInput<double>("t_x", request->goal_pose.position.x))
        {
            throw BT::RuntimeError("Missing parameter [t_x] in GoToGoalPose node");
        }

        if (!getInput<double>("t_y", request->goal_pose.position.y))
        {
            throw BT::RuntimeError("Missing parameter [t_y] in GoToGoalPose node");
        }

        if (!getInput<double>("t_z", request->goal_pose.position.z))
        {
            throw BT::RuntimeError("Missing parameter [t_z] in GoToGoalPose node");
        }
    }

    void write_output_ports(std::shared_ptr<GoToGoalPose::Response> response) override
    {
        setOutput("success", response->success);
    }
};

#endif // GO_TO_GOAL_POSE_CLIENT_GPP_