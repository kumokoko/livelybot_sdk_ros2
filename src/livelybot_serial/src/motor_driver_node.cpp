#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "livelybot_interfaces/msg/low_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "hardware/robot.h"

class MotorDriverNode : public rclcpp::Node
{
public:
  MotorDriverNode()
  : rclcpp::Node(
      "motor_driver_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
  }

  void initialize()
  {
    const int period_ms = get_or_declare_parameter<int>("driver.period_ms", 5);
    if (period_ms <= 0) {
      throw std::invalid_argument("driver.period_ms must be positive");
    }
    const std::string imu_topic =
      require_non_empty_parameter("driver.imu_topic", "/imu/data");
    const std::string joint_state_topic =
      require_non_empty_parameter("driver.joint_state_topic", "error_joint_states");
    const std::string command_topic =
      require_non_empty_parameter("driver.command_topic", "low_cmd");
    const std::string broadcast_command_topic =
      require_non_empty_parameter("driver.broadcast_command_topic", "broadcast_command");

    robot_ = std::make_shared<livelybot_serial::robot>(
      livelybot_serial::robot::load_runtime_config(*this),
      [this]() { return this->now().seconds(); },
      this->get_logger());
    const auto period = std::chrono::milliseconds(period_ms);
    joint_state_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, 10);
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 100, std::bind(&MotorDriverNode::imu_callback, this, std::placeholders::_1));
    low_cmd_sub_ = this->create_subscription<livelybot_interfaces::msg::LowCmd>(
      command_topic, 10, std::bind(&MotorDriverNode::low_cmd_callback, this, std::placeholders::_1));
    broadcast_command_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      broadcast_command_topic, 10,
      std::bind(&MotorDriverNode::broadcast_command_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(period, std::bind(&MotorDriverNode::tick, this));
  }

private:
  static constexpr uint8_t kBroadcastStop = 1;
  static constexpr uint8_t kBroadcastReset = 2;
  static constexpr uint8_t kBroadcastResetZero = 3;

  template<typename ParamT>
  ParamT get_or_declare_parameter(const std::string & name, const ParamT & default_value)
  {
    ParamT value = default_value;
    if (this->get_parameter(name, value)) {
      return value;
    }
    return this->declare_parameter<ParamT>(name, default_value);
  }

  std::string require_non_empty_parameter(
    const std::string & name, const std::string & default_value)
  {
    const std::string value = get_or_declare_parameter<std::string>(name, default_value);
    if (value.empty()) {
      throw std::invalid_argument(name + " must not be empty");
    }
    return value;
  }

  static std::vector<livelybot_serial::robot::motor_command> to_motor_commands(
    const livelybot_interfaces::msg::LowCmd & msg)
  {
    std::vector<livelybot_serial::robot::motor_command> commands;
    commands.reserve(msg.motor_cmd.size());
    for (const auto & motor_cmd : msg.motor_cmd) {
      livelybot_serial::robot::motor_command command;
      command.position = motor_cmd.q;
      command.velocity = motor_cmd.dq;
      command.torque = motor_cmd.tau;
      command.kp = motor_cmd.kp;
      command.kd = motor_cmd.kd;
      commands.push_back(command);
    }
    return commands;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!robot_) {
      return;
    }
    robot_->update_imu_state(*msg);
  }

  void low_cmd_callback(const livelybot_interfaces::msg::LowCmd::SharedPtr msg)
  {
    if (!robot_) {
      return;
    }

    try {
      robot_->apply_motor_commands(to_motor_commands(*msg));
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "motor_driver_node command callback failed: %s", e.what());
      }
  }

  void broadcast_command_callback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    if (!robot_) {
      return;
    }

    try {
      switch (msg->data) {
        case kBroadcastStop:
          robot_->stop_motors();
          break;
        case kBroadcastReset:
          robot_->reset_motors();
          break;
        case kBroadcastResetZero:
          robot_->reset_zero_positions();
          break;
        default:
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Unknown broadcast command: %u", msg->data);
          break;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "motor_driver_node broadcast command failed: %s", e.what());
    }
  }

  void tick()
  {
    if (!robot_) {
      return;
    }
    try {
      const auto stamp = this->now();
      joint_state_pub_->publish(robot_->run_control_cycle(stamp, stamp.seconds()));
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "motor_driver_node control tick failed: %s", e.what());
    }
  }

  std::shared_ptr<livelybot_serial::robot> robot_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<livelybot_interfaces::msg::LowCmd>::SharedPtr low_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr broadcast_command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MotorDriverNode>();
    node->initialize();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("motor_driver_node"), "motor_driver_node failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("motor_driver_node"), "motor_driver_node failed with an unknown exception");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
