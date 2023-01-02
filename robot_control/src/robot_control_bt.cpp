#include <memory>
#include <string>

// #include "robot_control/go_to_goal_pose_client.hpp"
#include "ros2_behavior_tree/behavior_tree.hpp"
#include "ros2_behavior_tree/node_thread.hpp"
#include "ros2_behavior_tree/ros2_service_client_node.hpp"
#include "robot_control/create_ros2_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_control/create_object.hpp"
#include "robot_control/grab_release_object.hpp"
#include "robot_control/go_to_goal_pose_node.hpp"
#include "robot_control/calculate_goal_pose_node.hpp"
#include "robot_control/repeat_until_node.hpp"

class TestServiceNode
{
public:
    TestServiceNode(const std::string &service_name, const std::string xml_text)
    {
        ros2_behavior_tree::BehaviorTree bt;
        bt.register_a_node<ros2_behavior_tree::CreateROS2Node>("CreateROS2Node");
        // bt.register_a_node<GoToGoalposeClient>("GoToGoalPose");
        bt.register_a_node<AddObjectNode>("AddObject");
        bt.register_a_node<GrabReleaseObjectNode>("GrabReleaseObject");
        bt.register_a_node<GoToGoalPoseNode>("GoToGoalPose");
        bt.register_a_node<CalculateGoalPoseNode>("CalculateGoalPose");
        bt.register_a_node<ros2_behavior_tree::RepeatUntilNode>("RepeatUntil");
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
    // std::unique_ptr<GoToGoalposeClient> go_to_goal_pose_client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    const std::string xml_text =
    R"(
  <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <SetBlackboard output_key="should_stop" value="false" />
            <CreateROS2Node node_name="test_bt_node" namespace="" spin="false" node_handle="{ros2_node}"/>
            <RepeatUntil key="should_stop" value="true">
                <Sequence>
                    <AddObject node_handle="{ros2_node}" px="3.0" py="1.0" pz="0.25" w="0.5" l="0.5" h="0.5" object_id="{box_id}" object_count="{box_count}" />
                    <GoToGoalPose node_handle="{ros2_node}" gx="3.0" gy="1.0" gz="0.55" />
                    <GrabReleaseObject node_handle="{ros2_node}" object_id="{box_id}" grasp="true" />
                    <CalculateGoalPose object_count="{box_count}" gx="{gx}" gy="{gy}" gz="{gz}" finish="{should_stop}" />
                    <GoToGoalPose node_handle="{ros2_node}" gx="{gx}" gy="{gy}" gz="{gz}" />
                    <GrabReleaseObject node_handle="{ros2_node}" object_id="{box_id}" grasp="false" />
                    <GoToGoalPose node_handle="{ros2_node}" gx="4.24" gy="0.0" gz="3.18" />
                </Sequence>
            </RepeatUntil>
        </Sequence>
     </BehaviorTree>
 </root>
    )";

    auto node = std::make_shared<TestServiceNode>(std::move("go_to_goal_pose_server"), std::move(xml_text));

    rclcpp::shutdown();

    return 0;
}