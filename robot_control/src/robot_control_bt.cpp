#include <memory>
#include <string>

#include "robot_control/go_to_goal_pose_client.hpp"
#include "ros2_behavior_tree/behavior_tree.hpp"
#include "ros2_behavior_tree/node_thread.hpp"
#include "ros2_behavior_tree/ros2_service_client_node.hpp"
#include "robot_control/create_ros2_node.hpp"
#include "rclcpp/rclcpp.hpp"

class TestServiceNode
{
public:
    TestServiceNode(const std::string &service_name, const std::string xml_text)
    {
        ros2_behavior_tree::BehaviorTree bt;
        bt.register_a_node<ros2_behavior_tree::CreateROS2Node>("CreateROS2Node");
        bt.register_a_node<GoToGoalposeClient>("GoToGoalPose");
        bt.load_xml(xml_text);

        auto bt_result = bt.execute();
    }

    ~TestServiceNode()
    {
        // ros2_node_thread_.reset();
        // ros2_node_.reset();
    }

public:
    BT::Blackboard::Ptr blackboard_;
    std::shared_ptr<rclcpp::Node> ros2_node_;
    std::shared_ptr<ros2_behavior_tree::NodeThread> ros2_node_thread_;
    std::unique_ptr<GoToGoalposeClient> go_to_goal_pose_client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    const std::string xml_text =
    R"(
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <CreateROS2Node node_name="test_bt_node" namespace="" spin="true" node_handle="{ros2_node}"/>
            <GoToGoalPose service_name="go_to_goal_pose_server" server_timeout="10000" ros2_node="{ros2_node}" t_x="3.127" t_y="0.223" t_z="1.78"/>
            <GoToGoalPose service_name="go_to_goal_pose_server" server_timeout="10000" ros2_node="{ros2_node}" t_x="0.0" t_y="0.223" t_z="1.78"/>
        </Sequence>
     </BehaviorTree>
 </root>
    )";

    auto node = std::make_shared<TestServiceNode>(std::move("go_to_goal_pose_server"), std::move(xml_text));

    // rclcpp::spin(node->ros2_node_);

    rclcpp::shutdown();

    return 0;
}