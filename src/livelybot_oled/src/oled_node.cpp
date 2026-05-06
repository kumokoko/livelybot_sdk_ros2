#include "ip_addr.hpp"
#include "sensor_actuator_status.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class OledNode : public rclcpp::Node
{
public:
  OledNode()
  : Node("livelybot_oled")
  {
    auto motor_counts = declare_parameter<std::vector<int64_t>>("motor_counts", {6, 0, 0, 0});
    const auto oled_port = declare_parameter<std::string>("oled_port", "/dev/ttyS4");
    const auto ip_send_period_ms = declare_parameter<int>("ip_send_period_ms", 1000);
    const auto status_send_period_ms = declare_parameter<int>("status_send_period_ms", 100);

    for (size_t i = 0; i < std::min<size_t>(motor_counts.size(), motor_counts_.size()); ++i) {
      motor_counts_[i] = static_cast<int>(motor_counts[i]);
    }

    total_motor_count_ = motor_counts_[0] + motor_counts_[1] + motor_counts_[2] + motor_counts_[3];
    status_bridge_ = std::make_unique<Sensor_actuator_status>(
      motor_counts_[0], motor_counts_[1], motor_counts_[2], motor_counts_[3], oled_port);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10, std::bind(&OledNode::imu_callback, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/error_joint_states", 10,
      std::bind(&OledNode::joint_state_callback, this, std::placeholders::_1));
    battery_level_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/battery_level", 10,
      std::bind(&OledNode::battery_level_callback, this, std::placeholders::_1));
    fsm_state_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/fsm_state", 10, std::bind(&OledNode::fsm_state_callback, this, std::placeholders::_1));

    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(status_send_period_ms),
      std::bind(&OledNode::status_timer_callback, this));
    ip_timer_ = create_wall_timer(
      std::chrono::milliseconds(ip_send_period_ms), std::bind(&OledNode::ip_timer_callback, this));

    RCLCPP_INFO(
      get_logger(), "OLED node started. port=%s motors=%d", oled_port.c_str(), total_motor_count_);
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion q(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 rot_mat(q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    rot_mat.getRPY(roll, pitch, yaw);

    latest_rpy_[0] = static_cast<float>(roll);
    latest_rpy_[1] = static_cast<float>(pitch);
    latest_rpy_[2] = static_cast<float>(yaw);
    imu_received_ = true;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::fill(motor_status_.begin(), motor_status_.end(), 0);
    const auto count = std::min<size_t>(msg->position.size(), static_cast<size_t>(total_motor_count_));
    for (size_t i = 0; i < count; ++i) {
      motor_status_[i] = msg->position[i] < -900.0 ? 0U : 1U;
    }
    status_bridge_->send_motor_status(motor_status_.data());
  }

  void battery_level_callback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    status_bridge_->send_battery_level(msg->data);
  }

  void fsm_state_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    status_bridge_->send_fsm_state(msg->data);
  }

  void status_timer_callback()
  {
    if (imu_received_) {
      status_bridge_->send_imu_status(true, latest_rpy_.data());
    }
  }

  void ip_timer_callback()
  {
    update_ip_addr();
    status_bridge_->send_ip_addr(get_ip_data_u32_all(), 3);
  }

  std::array<int, 4> motor_counts_{0, 0, 0, 0};
  int total_motor_count_{0};
  bool imu_received_{false};
  std::array<float, 3> latest_rpy_{0.0F, 0.0F, 0.0F};
  std::array<unsigned char, 64> motor_status_{};
  std::unique_ptr<Sensor_actuator_status> status_bridge_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr battery_level_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fsm_state_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr ip_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OledNode>());
  rclcpp::shutdown();
  return 0;
}
