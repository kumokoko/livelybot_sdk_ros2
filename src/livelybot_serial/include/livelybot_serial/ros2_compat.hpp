#ifndef LIVELYBOT_SERIAL__ROS2_COMPAT_HPP_
#define LIVELYBOT_SERIAL__ROS2_COMPAT_HPP_

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "builtin_interfaces/msg/time.hpp"
#include "livelybot_interfaces/msg/motor_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace livelybot_serial_ros2
{
void set_global_node(const rclcpp::Node::SharedPtr & node);
rclcpp::Node::SharedPtr get_global_node();

std::string parameter_key_from_ros1(const std::string & key);
}  // namespace livelybot_serial_ros2

namespace livelybot_msg = livelybot_interfaces::msg;

namespace sensor_msgs
{
using Imu = sensor_msgs::msg::Imu;
using JointState = sensor_msgs::msg::JointState;
}  // namespace sensor_msgs

namespace ros
{
class Time
{
public:
  Time()
  : seconds_(0.0)
  {
  }

  explicit Time(double seconds)
  : seconds_(seconds)
  {
  }

  static Time now()
  {
    auto node = livelybot_serial_ros2::get_global_node();
    return Time(node->now().seconds());
  }

  double toSec() const
  {
    return seconds_;
  }

  operator builtin_interfaces::msg::Time() const
  {
    builtin_interfaces::msg::Time msg;
    const auto whole = static_cast<int32_t>(seconds_);
    msg.sec = whole;
    msg.nanosec = static_cast<uint32_t>((seconds_ - static_cast<double>(whole)) * 1e9);
    return msg;
  }

private:
  double seconds_;
};

class Duration
{
public:
  explicit Duration(double seconds)
  : seconds_(seconds)
  {
  }

  void sleep() const
  {
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds_));
  }

private:
  double seconds_;
};

class Rate
{
public:
  explicit Rate(double hz)
  : period_(hz > 0.0 ? 1.0 / hz : 0.0)
  {
  }

  void sleep() const
  {
    if (period_ > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(period_));
    }
  }

private:
  double period_;
};

class Publisher
{
public:
  Publisher() = default;

  template<typename MessageT>
  explicit Publisher(const std::shared_ptr<rclcpp::Publisher<MessageT>> & publisher)
  : publish_fn_([publisher](const void * msg) {
      publisher->publish(*static_cast<const MessageT *>(msg));
    })
  {
  }

  template<typename MessageT>
  void publish(const MessageT & msg) const
  {
    if (publish_fn_) {
      publish_fn_(&msg);
    }
  }

private:
  std::function<void(const void *)> publish_fn_;
};

class Subscriber
{
public:
  Subscriber() = default;

  template<typename SubscriptionT>
  explicit Subscriber(const std::shared_ptr<SubscriptionT> & subscription)
  : subscription_(subscription)
  {
  }

private:
  std::shared_ptr<void> subscription_;
};

class NodeHandle
{
public:
  NodeHandle()
  : node_(livelybot_serial_ros2::get_global_node())
  {
  }

  template<typename ParamT>
  bool getParam(const std::string & key, ParamT & value) const
  {
    return node_->get_parameter(
      livelybot_serial_ros2::parameter_key_from_ros1(key),
      value);
  }

  template<typename MessageT>
  Publisher advertise(const std::string & topic_name, size_t queue_size) const
  {
    return Publisher(node_->create_publisher<MessageT>(topic_name, queue_size));
  }

  template<typename MessageT, typename ClassT>
  Subscriber subscribe(
    const std::string & topic_name,
    size_t queue_size,
    void (ClassT::*callback)(std::shared_ptr<const MessageT>),
    ClassT * instance) const
  {
    auto subscription = node_->create_subscription<MessageT>(
      topic_name,
      queue_size,
      std::bind(callback, instance, std::placeholders::_1));
    return Subscriber(subscription);
  }

private:
  rclcpp::Node::SharedPtr node_;
};

inline bool ok()
{
  return rclcpp::ok();
}
}  // namespace ros

#define ROS_INFO(...) RCLCPP_INFO(livelybot_serial_ros2::get_global_node()->get_logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(livelybot_serial_ros2::get_global_node()->get_logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(livelybot_serial_ros2::get_global_node()->get_logger(), __VA_ARGS__)
#include "rclcpp/logging.hpp"
#define ROS_INFO_STREAM(msg) RCLCPP_INFO_STREAM(livelybot_serial_ros2::get_global_node()->get_logger(), msg)
#define ROS_ERROR_STREAM(msg) RCLCPP_ERROR_STREAM(livelybot_serial_ros2::get_global_node()->get_logger(), msg)

#endif  // LIVELYBOT_SERIAL__ROS2_COMPAT_HPP_
