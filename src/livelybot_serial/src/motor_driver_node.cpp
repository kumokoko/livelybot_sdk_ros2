#include <chrono>
#include <memory>

#include "livelybot_serial/ros2_compat.hpp"
#include "rclcpp/rclcpp.hpp"

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
    robot_ = std::make_shared<livelybot_serial::robot>();
    const auto period =
      std::chrono::milliseconds(this->declare_parameter<int>("driver.period_ms", 5));
    timer_ = this->create_wall_timer(period, std::bind(&MotorDriverNode::tick, this));
  }

private:
  void tick()
  {
    robot_->detect_motor_limit();
    robot_->motor_send_2();
  }

  std::shared_ptr<livelybot_serial::robot> robot_;
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
