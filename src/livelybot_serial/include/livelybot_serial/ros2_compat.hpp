#ifndef LIVELYBOT_SERIAL__ROS2_COMPAT_HPP_
#define LIVELYBOT_SERIAL__ROS2_COMPAT_HPP_

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"

namespace livelybot_serial_ros2
{
void set_global_node(const rclcpp::Node::SharedPtr & node);
rclcpp::Node::SharedPtr get_global_node();
double now_seconds();
builtin_interfaces::msg::Time now();
void sleep_for_seconds(double seconds);
void sleep_for_ms(int64_t milliseconds);

std::string parameter_key_from_ros1(const std::string & key);
}  // namespace livelybot_serial_ros2

#define ROS_INFO(...) RCLCPP_INFO(livelybot_serial_ros2::get_global_node()->get_logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(livelybot_serial_ros2::get_global_node()->get_logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(livelybot_serial_ros2::get_global_node()->get_logger(), __VA_ARGS__)
#include "rclcpp/logging.hpp"
#define ROS_INFO_STREAM(msg) RCLCPP_INFO_STREAM(livelybot_serial_ros2::get_global_node()->get_logger(), msg)
#define ROS_ERROR_STREAM(msg) RCLCPP_ERROR_STREAM(livelybot_serial_ros2::get_global_node()->get_logger(), msg)

#endif  // LIVELYBOT_SERIAL__ROS2_COMPAT_HPP_
