#!/usr/bin/env python3

import time
import rclpy

from simple_node import Node

from yasmin import State
from yasmin import StateMachine
from yasmin_ros import ServiceState
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT
from yasmin_viewer import YasminViewerPub

from robot_msgs.srv import GoToGoalPose


class DemoNode(Node):

    def __init__(self):
        super().__init__("robot_control_sm_node")

        # create a state machine
        sm = StateMachine(outcomes=["outcome4"])

        # add states
        sm.add_state("POSE1", ServiceState(self, GoToGoalPose, "go_to_goal_pose_server", self.pose1_request_cb,
                                                ["pose2", "stop"], self.pose1_response_cb),
                        transitions={"pose2": "POSE2", "stop": "STOP"})
        sm.add_state("POSE2", ServiceState(self, GoToGoalPose, "go_to_goal_pose_server", self.pose2_request_cb,
                                                ["pose1", "stop"], self.pose2_response_cb),
                        transitions={"pose1": "POSE1", "stop": "STOP"})

        # pub
        YasminViewerPub(self, "YASMIN_DEMO", sm)

        # execute
        outcome = sm()
        print(outcome)

    def pose1_request_cb(self, blackboard):
        print("request for pose1\n")
        request = GoToGoalPose.Request()
        request.pose.position.x = 3.127
        request.pose.position.y = 0.223
        request.pose.position.z = 1.78
        return request

    def pose2_request_cb(self, blackboard):
        print("request for pose2\n")
        request = GoToGoalPose.Request()
        request.pose.position.x = 0.0
        request.pose.position.y = 0.223
        request.pose.position.z = 1.78
        return request

    def pose1_response_cb(self, blackboard, response):
        print("response of pose1\n")
        if response.success:
            return "pose2"
        else:
            return "stop"
    
    def pose2_response_cb(self, blackboard, response):
        print("response of pose2\n")
        if response.success:
            return "pose1"
        else:
            return "stop"


# main
def main(args=None):

    print("yasmin_demo")
    rclpy.init(args=args)
    node = DemoNode()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
