#include "livelybot_serial/ros2_compat.hpp"

#include <stdexcept>

namespace livelybot_serial_ros2
{
namespace
{
std::mutex g_node_mutex;
rclcpp::Node::SharedPtr g_node;
}  // namespace

void set_global_node(const rclcpp::Node::SharedPtr & node)
{
  std::lock_guard<std::mutex> lock(g_node_mutex);
  g_node = node;
}

rclcpp::Node::SharedPtr get_global_node()
{
  std::lock_guard<std::mutex> lock(g_node_mutex);
  if (!g_node) {
    throw std::runtime_error("livelybot_serial ROS2 global node is not initialized");
  }
  return g_node;
}

double now_seconds()
{
  return get_global_node()->now().seconds();
}

builtin_interfaces::msg::Time now()
{
  return get_global_node()->now();
}

void sleep_for_seconds(double seconds)
{
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

void sleep_for_ms(int64_t milliseconds)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

std::string parameter_key_from_ros1(const std::string & key)
{
  std::string converted = key;
  std::replace(converted.begin(), converted.end(), '/', '.');
  if (!converted.empty() && converted.front() == '.') {
    converted.erase(converted.begin());
  }
  return converted;
}
}  // namespace livelybot_serial_ros2
