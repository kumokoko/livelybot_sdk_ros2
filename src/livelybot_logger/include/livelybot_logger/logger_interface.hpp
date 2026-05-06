#pragma once

#include "livelybot_logger/msg/logger_operation.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace livelybot_logger
{
class LoggerInterface
{
public:
  static void init(rclcpp::Node & node, const std::string & node_name);
  static void log_operation(
    const std::string & operation_type, const std::string & operation_data,
    const std::string & result = "success");

private:
  static rclcpp::Publisher<livelybot_logger::msg::LoggerOperation>::SharedPtr operation_pub_;
  static rclcpp::Node * node_;
  static std::string node_name_;
  static bool initialized_;
};
}  // namespace livelybot_logger
