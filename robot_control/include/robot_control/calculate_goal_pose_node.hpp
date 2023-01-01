#ifndef CALCULATE_GOAL_POSE_NODE_HPP_
#define CALCULATE_GOAL_POSE_NODE_HPP_

#include "behaviortree_cpp_v3/action_node.h"

#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

class CalculateGoalPoseNode : public BT::SyncActionNode
{
public:
    CalculateGoalPoseNode(const std::string& action_name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(action_name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("object_count", "Count of the object"),
            BT::OutputPort<double>("gx", "X goal pose"),
            BT::OutputPort<double>("gy", "Y goal pose"),
            BT::OutputPort<double>("gz", "Z goal pose"),
            BT::OutputPort<bool>("finish", "Should finish"),
        };
    }

    BT::NodeStatus tick() override
    {
        int object_count;
        if (!getInput("object_count", object_count)){
            throw BT::RuntimeError("Please specify number of boxes.");
        }

        double w = 0.5, l = 0.5, h = 0.25;
        int des_num_layers = 2;
        int des_col_num = 3, des_row_num = 3;

        int row_num = ceil(object_count*1.0 / des_col_num*1.0);
        int col_num = des_col_num - abs(row_num * des_col_num - object_count);

        int num_layers = floor(object_count * 1.0 / (des_col_num * des_row_num * 1.0));

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "row_num: %d\n", row_num);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "col_num: %d\n", col_num);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "num_layers: %d\n", num_layers);

        double start_x = -3.38, start_y = -1.6, start_z = 0.5;
        double place_w = w + 0.03;
        double place_l = l + 0.03;
        double place_h = h + 0.03;

        double gx = start_x + (row_num-1) * 1.0 * place_w;
        double gy = start_y + (col_num-1) * 1.0 * place_l;
        double gz = start_z + num_layers * 1.0 * place_h;

        bool should_stop = (num_layers < des_num_layers) ? false : true;

        if (!setOutput("gx", gx) || !setOutput("gy", gy) || !setOutput("gz", gz)){
            throw BT::RuntimeError("Failed to set output port of [gx, gy, gz].");
        }
        if (!setOutput("finish", should_stop)){
            throw BT::RuntimeError("Failed to set output port of [finish].");
        }

        std::this_thread::sleep_for(std::chrono::seconds(2));

        return BT::NodeStatus::SUCCESS;
    }
};

#endif