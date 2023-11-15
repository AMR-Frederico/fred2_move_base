// cpp_node.cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  // Initialize the ROS 2 node
  rclcpp::init(argc, argv);

  // Create a ROS 2 node
  auto node = rclcpp::Node::make_shared("cpp_node");

  // Print "Hello, World!" to the console
  RCLCPP_INFO(node->get_logger(), "Hello, World!");

  // Spin the node (this keeps the node running until it's manually stopped)
  rclcpp::spin(node);

  // Shutdown the ROS 2 node
  rclcpp::shutdown();

  return 0;
}
