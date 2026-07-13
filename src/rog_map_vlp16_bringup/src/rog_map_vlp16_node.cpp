#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rog_map_ros/rog_map_ros2.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rog_map_vlp16");
  node->declare_parameter<std::string>("config_file", "");

  const auto config_file = node->get_parameter("config_file").as_string();
  if (config_file.empty()) {
    RCLCPP_FATAL(node->get_logger(), "Parameter config_file is required");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Loading ROG-Map config: %s", config_file.c_str());
  auto rog_map = std::make_shared<rog_map::ROGMapROS>(node, config_file);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rog_map.reset();
  rclcpp::shutdown();
  return 0;
}
