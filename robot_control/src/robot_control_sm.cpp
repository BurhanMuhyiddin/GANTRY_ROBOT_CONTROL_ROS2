#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/srv/go_to_goal_pose.hpp"

#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/service_state.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

typedef robot_msgs::srv::GoToGoalPose::Request REQUEST;
typedef robot_msgs::srv::GoToGoalPose::Response RESPONSE;
typedef std::shared_ptr<yasmin::blackboard::Blackboard> BLACKBOARD;

class POSE1State : public yasmin_ros::ServiceState<robot_msgs::srv::GoToGoalPose>
{
public:
    POSE1State(simple_node::Node *node)
        : yasmin_ros::ServiceState<robot_msgs::srv::GoToGoalPose>(
            node,
            "go_to_goal_pose_server",
            std::bind(&POSE1State::pose1_request_clb, this, _1),
            {"pose2", "stop"},
            std::bind(&POSE1State::pose1_response_clb, this, _1, _2)
        ) {}
    ~POSE1State() {};
private:
    REQUEST pose1_request_clb(BLACKBOARD inp_blackboard){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request for POSE1...\n");

        robot_msgs::srv::GoToGoalPose::Request pose_request;
        pose_request.goal_pose.position.x = 3.127;
        pose_request.goal_pose.position.y = 0.223;
        pose_request.goal_pose.position.z = 1.78;

        return pose_request;
    }

    std::string pose1_response_clb(BLACKBOARD inp_blackboard, RESPONSE resp){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response for POSE1: %s\n", resp.success ? "SUCCEDED" : "FAILED");

        std::this_thread::sleep_for(std::chrono::seconds(3));

        if (resp.success){
            return "pose2";
        }else{
            return "stop";
        }
    }
};

class POSE2State : public yasmin_ros::ServiceState<robot_msgs::srv::GoToGoalPose>
{
public:
    POSE2State(simple_node::Node *node)
        : yasmin_ros::ServiceState<robot_msgs::srv::GoToGoalPose>(
            node,
            "go_to_goal_pose_server",
            std::bind(&POSE2State::pose2_request_clb, this, _1),
            {"pose1", "stop"},
            std::bind(&POSE2State::pose2_response_clb, this, _1, _2)
        ) {}
    ~POSE2State() {};
private:
    REQUEST pose2_request_clb(BLACKBOARD inp_blackboard){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request for POSE2...\n");

        robot_msgs::srv::GoToGoalPose::Request pose_request;
        pose_request.goal_pose.position.x = 0.0;
        pose_request.goal_pose.position.y = 0.223;
        pose_request.goal_pose.position.z = 1.78;

        return pose_request;
    }
    
    std::string pose2_response_clb(BLACKBOARD inp_blackboard, RESPONSE resp){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response for POSE2: %s\n", resp.success ? "SUCCEDED" : "FAILED");

        std::this_thread::sleep_for(std::chrono::seconds(3));

        if (resp.success){
            return "pose1";
        }else{
            return "stop";
        }
    }
};


class RobotControlFsm : public simple_node::Node {
public:
    RobotControlFsm() : simple_node::Node("robot_control_fsm_node"){
        auto sm = std::make_shared<yasmin::StateMachine>(yasmin::StateMachine({"sm_result"}));

        sm->add_state("POSE1", std::make_shared<POSE1State>(this), {{"pose2", "POSE2"},
                                                                    {"stop", "sm_result"},
                                                                    {yasmin_ros::basic_outcomes::CANCEL, "sm_result"}});

        sm->add_state("POSE2", std::make_shared<POSE2State>(this), {{"pose1", "POSE1"},
                                                                    {"stop", "sm_result"},
                                                                    {yasmin_ros::basic_outcomes::CANCEL, "sm_result"}});
        
        std::string outcome = (*sm.get())();
        std::cout << outcome << "\n";
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotControlFsm>();
    // node->join_spin();
    rclcpp::spin(node);
    
    rclcpp::shutdown();

    return 0;
}