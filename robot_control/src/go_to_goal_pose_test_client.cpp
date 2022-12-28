#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/srv/go_to_goal_pose.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
  rclcpp::Client<robot_msgs::srv::GoToGoalPose>::SharedPtr client =
    node->create_client<robot_msgs::srv::GoToGoalPose>("go_to_goal_pose_server");

  auto request = std::make_shared<robot_msgs::srv::GoToGoalPose::Request>();
  request->goal_pose.position.x = 3.127;
  request->goal_pose.position.y = 0.223;
  request->goal_pose.position.z = 1.78;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: %s", result.get()->success ? "Yeppp" : "Nooop");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}