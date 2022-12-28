#include <chrono>
#include <iostream>
#include <memory>
#include <string>

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

class RobotControlFsm : public simple_node::Node {
public:
    // std::bind(&Foo::doSomething, this)
    RobotControlFsm() : simple_node::Node("robot_control_fsm_node"){
        auto sm = std::make_shared<yasmin::StateMachine>(yasmin::StateMachine({"sm_result"}));

        std::vector<std::string> pose1_outcomes{"pose2", "stop"};
        sm->add_state("POSE1", std::make_shared<yasmin_ros::ServiceState<robot_msgs::srv::GoToGoalPose>>(this, "go_to_goal_pose_server",
                                                                    std::bind(&RobotControlFsm::pose1_request_clb, this, _1),
                                                                    pose1_outcomes,
                                                                    std::bind(&RobotControlFsm::pose1_response_clb, this, _1, _2)),
                                                                    {{"pose2", "POSE2"},
                                                                    {"stop", "STOP"}});
        std::vector<std::string> pose2_outcomes{"pose1", "stop"};
        sm->add_state("POSE2", std::make_shared<yasmin_ros::ServiceState<robot_msgs::srv::GoToGoalPose>>(this, "go_to_goal_pose_server",
                                                                    std::bind(&RobotControlFsm::pose2_request_clb, this, _1),
                                                                    pose2_outcomes,
                                                                    std::bind(&RobotControlFsm::pose2_response_clb, this, _1, _2)),
                                                                    {{"pose1", "POSE1"},
                                                                    {"stop", "STOP"}});
        
        std::string outcome = (*sm.get())();
        std::cout << outcome << "\n";
    }

private:
    REQUEST pose1_request_clb(BLACKBOARD inp_blackboard);
    std::string pose1_response_clb(BLACKBOARD inp_blackboard, RESPONSE resp);
    REQUEST pose2_request_clb(BLACKBOARD inp_blackboard);
    std::string pose2_response_clb(BLACKBOARD inp_blackboard, RESPONSE resp);
};

REQUEST RobotControlFsm::pose1_request_clb(BLACKBOARD inp_blackboard){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request for POSE1...\n");

    robot_msgs::srv::GoToGoalPose::Request pose_request;
    pose_request.goal_pose.position.x = 3.127;
    pose_request.goal_pose.position.y = 0.223;
    pose_request.goal_pose.position.z = 1.78;

    return pose_request;
}

std::string RobotControlFsm::pose1_response_clb(BLACKBOARD inp_blackboard, RESPONSE resp){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response for POSE1...\n");

    if (resp.success){
        return "pose2";
    }else{
        return "stop";
    }
}

REQUEST RobotControlFsm::pose2_request_clb(BLACKBOARD inp_blackboard){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request for POSE2...\n");

    robot_msgs::srv::GoToGoalPose::Request pose_request;
    pose_request.goal_pose.position.x = 0.0;
    pose_request.goal_pose.position.y = 0.223;
    pose_request.goal_pose.position.z = 1.78;

    return pose_request;
}

std::string RobotControlFsm::pose2_response_clb(BLACKBOARD inp_blackboard, RESPONSE resp){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response for POSE2...\n");

    if (resp.success){
        return "pose1";
    }else{
        return "stop";
    }
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotControlFsm>();
    node->join_spin();
    
    rclcpp::shutdown();

    return 0;
}