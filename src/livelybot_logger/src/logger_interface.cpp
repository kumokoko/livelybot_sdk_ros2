#include "livelybot_logger/logger_interface.hpp"

namespace livelybot_logger
{
rclcpp::Publisher<livelybot_logger::msg::LoggerOperation>::SharedPtr LoggerInterface::operation_pub_;
rclcpp::Node * LoggerInterface::node_ = nullptr;
std::string LoggerInterface::node_name_;
bool LoggerInterface::initialized_ = false;

void LoggerInterface::init(rclcpp::Node & node, const std::string & node_name)
{
  if (initialized_) {
    return;
  }

  node_ = &node;
  node_name_ = node_name;
  operation_pub_ =
    node.create_publisher<livelybot_logger::msg::LoggerOperation>("/logger/operation", 10);
  initialized_ = true;
}

void LoggerInterface::log_operation(
  const std::string & operation_type, const std::string & operation_data, const std::string & result)
{
  if (!initialized_ || !operation_pub_ || node_ == nullptr) {
    return;
  }

  livelybot_logger::msg::LoggerOperation msg;
  msg.timestamp = node_->now();
  msg.node_name = node_name_;
  msg.operation_type = operation_type;
  msg.operation_data = operation_data;
  msg.result = result;
  operation_pub_->publish(msg);
}
}  // namespace livelybot_logger
