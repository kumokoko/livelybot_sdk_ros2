#include <chrono>
#include <memory>

#include "livelybot_serial/ros2_compat.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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
    livelybot_serial_ros2::set_global_node(shared_from_this());
    robot_ = std::make_shared<livelybot_serial::robot>(
      livelybot_serial::robot::load_runtime_config(*this));
    const auto period =
      std::chrono::milliseconds(this->declare_parameter<int>("driver.period_ms", 5));
    joint_state_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("error_joint_states", 10);
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 100, std::bind(&MotorDriverNode::imu_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(period, std::bind(&MotorDriverNode::tick, this));
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    robot_->update_imu_state(*msg);
  }

  void tick()
  {
    robot_->detect_motor_limit();
    robot_->motor_send_2();
    joint_state_pub_->publish(robot_->build_joint_state_message());
  }

  std::shared_ptr<livelybot_serial::robot> robot_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorDriverNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
