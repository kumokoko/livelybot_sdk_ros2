#include "livelybot_interfaces/msg/low_cmd.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
double triangle_position(double lower, double upper, double phase)
{
  const double span = upper - lower;
  if (phase < 0.5) {
    return lower + span * phase * 2.0;
  }
  return upper - span * (phase - 0.5) * 2.0;
}
}  // namespace

class MotorOscillatorNode : public rclcpp::Node
{
public:
  MotorOscillatorNode()
  : Node("motor_oscillator_node")
  {
    command_topic_ = declare_parameter<std::string>("command_topic", "low_cmd");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "error_joint_states");
    motor_count_ = declare_parameter<int>("motor_count", 1);
    motor_index_ = declare_parameter<int>("motor_index", 0);
    use_current_position_as_center_ = declare_parameter<bool>("use_current_position_as_center", true);
    amplitude_ = declare_parameter<double>("amplitude", 1.0);
    center_position_ = declare_parameter<double>("center_position", 0.0);
    lower_position_ = declare_parameter<double>("lower_position", -1.0);
    upper_position_ = declare_parameter<double>("upper_position", 1.0);
    hold_position_ = declare_parameter<double>("hold_position", 0.0);
    period_sec_ = declare_parameter<double>("period_sec", 4.0);
    publish_hz_ = declare_parameter<double>("publish_hz", 50.0);
    mode_ = declare_parameter<int>("mode", 0);
    kp_ = declare_parameter<double>("kp", 1.0);
    kd_ = declare_parameter<double>("kd", 0.1);
    dq_ = declare_parameter<double>("dq", 0.0);
    tau_ = declare_parameter<double>("tau", 0.0);

    validate_parameters();

    publisher_ = create_publisher<livelybot_interfaces::msg::LowCmd>(command_topic_, 10);
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 10,
      std::bind(&MotorOscillatorNode::joint_state_callback, this, std::placeholders::_1));
    start_time_ = now();

    const auto interval = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(interval),
      std::bind(&MotorOscillatorNode::publish_command, this));

    RCLCPP_INFO(
      get_logger(),
      "Motor oscillator started: topic=%s motor_index=%d motor_count=%d period=%.3fs "
      "kp=%.3f kd=%.3f",
      command_topic_.c_str(), motor_index_, motor_count_, period_sec_, kp_, kd_);
    if (use_current_position_as_center_) {
      RCLCPP_INFO(
        get_logger(), "Waiting for %s, then oscillating around current position with amplitude %.3f",
        joint_state_topic_.c_str(), amplitude_);
    } else {
      RCLCPP_INFO(
        get_logger(), "Using absolute range [%.3f, %.3f]", lower_position_, upper_position_);
    }
  }

private:
  void validate_parameters()
  {
    if (motor_count_ <= 0) {
      throw std::invalid_argument("motor_count must be greater than 0");
    }
    if (motor_index_ < 0 || motor_index_ >= motor_count_) {
      throw std::invalid_argument("motor_index must be in [0, motor_count)");
    }
    if (period_sec_ <= 0.0) {
      throw std::invalid_argument("period_sec must be greater than 0");
    }
    if (publish_hz_ <= 0.0) {
      throw std::invalid_argument("publish_hz must be greater than 0");
    }
    if (upper_position_ < lower_position_) {
      std::swap(upper_position_, lower_position_);
    }
    if (amplitude_ < 0.0) {
      amplitude_ = std::abs(amplitude_);
    }
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    latest_positions_.assign(msg->position.begin(), msg->position.end());
    if (!center_initialized_ && use_current_position_as_center_ &&
      static_cast<std::size_t>(motor_index_) < latest_positions_.size())
    {
      center_position_ = latest_positions_[static_cast<std::size_t>(motor_index_)];
      lower_position_ = center_position_ - amplitude_;
      upper_position_ = center_position_ + amplitude_;
      center_initialized_ = true;
      start_time_ = now();
      RCLCPP_INFO(
        get_logger(), "Using current motor position %.3f as center, range=[%.3f, %.3f]",
        center_position_, lower_position_, upper_position_);
    }
  }

  void publish_command()
  {
    if (use_current_position_as_center_ && !center_initialized_) {
      return;
    }

    const double elapsed = (now() - start_time_).seconds();
    const double cycle = std::fmod(elapsed, period_sec_) / period_sec_;
    const double target = triangle_position(lower_position_, upper_position_, cycle);

    livelybot_interfaces::msg::LowCmd message;
    message.motor_cmd.resize(static_cast<std::size_t>(motor_count_));

    for (std::size_t i = 0; i < message.motor_cmd.size(); ++i) {
      auto & command = message.motor_cmd[i];
      command.mode = static_cast<uint8_t>(mode_);
      const bool has_feedback = i < latest_positions_.size() &&
        std::isfinite(latest_positions_[i]) && latest_positions_[i] > -900.0;
      command.q = static_cast<float>(has_feedback ? latest_positions_[i] : hold_position_);
      command.dq = static_cast<float>(dq_);
      command.tau = static_cast<float>(tau_);
      command.kp = static_cast<float>(kp_);
      command.kd = static_cast<float>(kd_);
      command.reserve = {0, 0, 0};
    }

    message.motor_cmd[static_cast<std::size_t>(motor_index_)].q = static_cast<float>(target);
    publisher_->publish(message);
  }

  std::string command_topic_;
  std::string joint_state_topic_;
  int motor_count_{1};
  int motor_index_{0};
  bool use_current_position_as_center_{true};
  bool center_initialized_{false};
  double amplitude_{1.0};
  double center_position_{0.0};
  double lower_position_{-1.0};
  double upper_position_{1.0};
  double hold_position_{0.0};
  double period_sec_{4.0};
  double publish_hz_{50.0};
  int mode_{0};
  double kp_{1.0};
  double kd_{0.1};
  double dq_{0.0};
  double tau_{0.0};

  rclcpp::Time start_time_;
  std::vector<double> latest_positions_;
  rclcpp::Publisher<livelybot_interfaces::msg::LowCmd>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorOscillatorNode>());
  rclcpp::shutdown();
  return 0;
}
