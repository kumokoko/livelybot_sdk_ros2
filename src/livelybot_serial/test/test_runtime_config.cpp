#include "hardware/robot.h"

#include <algorithm>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

namespace
{
class RclcppTestEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

std::shared_ptr<rclcpp::Node> make_node()
{
  return std::make_shared<rclcpp::Node>("runtime_config_test");
}

template<typename ValueT>
void declare_param(rclcpp::Node & node, const std::string & name, const ValueT & value)
{
  node.declare_parameter<ValueT>(name, value);
}

void declare_valid_runtime_config(rclcpp::Node & node)
{
  declare_param(node, "robot.Seial_baudrate", 921600);
  declare_param(node, "robot.robot_name", std::string("test_robot"));
  declare_param(node, "robot.CANboard_num", 1);
  declare_param(node, "robot.Serial_Type", std::string("/dev/ttyACM"));
  declare_param(node, "robot.control_type", 9);
  declare_param(node, "robot.imu_limt_flag", false);
  declare_param(node, "robot.imu_dir", false);
  declare_param(node, "robot.imu_limt_num", 1.0);
  declare_param(node, "robot.motor_timeout_ms", 1000);

  declare_param(node, "robot.CANboard.No_1_CANboard.CANport_num", 1);
  declare_param(node, "robot.CANboard.No_1_CANboard.CANport.CANport_1.motor_num", 1);
  declare_param(node, "robot.CANboard.No_1_CANboard.CANport.CANport_1.serial_id", 1);

  const std::string motor_base =
    "robot.CANboard.No_1_CANboard.CANport.CANport_1.motor.motor1";
  declare_param(node, motor_base + ".name", std::string("joint1"));
  declare_param(node, motor_base + ".id", 1);
  declare_param(node, motor_base + ".type", std::string("5046_20"));
  declare_param(node, motor_base + ".num", 1);
  declare_param(node, motor_base + ".pos_limit_enable", true);
  declare_param(node, motor_base + ".pos_upper", 1.0);
  declare_param(node, motor_base + ".pos_lower", -1.0);
  declare_param(node, motor_base + ".tor_limit_enable", true);
  declare_param(node, motor_base + ".tor_upper", 2.0);
  declare_param(node, motor_base + ".tor_lower", -2.0);
}

void declare_motor(
  rclcpp::Node & node, int port_index, int motor_index, int motor_id, int motor_num)
{
  const std::string motor_base =
    "robot.CANboard.No_1_CANboard.CANport.CANport_" + std::to_string(port_index) +
    ".motor.motor" + std::to_string(motor_index);
  declare_param(node, motor_base + ".name", std::string("joint") + std::to_string(motor_num));
  declare_param(node, motor_base + ".id", motor_id);
  declare_param(node, motor_base + ".type", std::string("5046_20"));
  declare_param(node, motor_base + ".num", motor_index);
  declare_param(node, motor_base + ".pos_limit_enable", false);
  declare_param(node, motor_base + ".pos_upper", 0.0);
  declare_param(node, motor_base + ".pos_lower", 0.0);
  declare_param(node, motor_base + ".tor_limit_enable", false);
  declare_param(node, motor_base + ".tor_upper", 0.0);
  declare_param(node, motor_base + ".tor_lower", 0.0);
}

void declare_runtime_config_with_ports(rclcpp::Node & node, int canport_num, int total_motors)
{
  declare_param(node, "robot.Seial_baudrate", 921600);
  declare_param(node, "robot.robot_name", std::string("test_robot"));
  declare_param(node, "robot.CANboard_num", 1);
  declare_param(node, "robot.Serial_Type", std::string("/dev/ttyACM"));
  declare_param(node, "robot.control_type", 9);
  declare_param(node, "robot.imu_limt_flag", false);
  declare_param(node, "robot.imu_dir", false);
  declare_param(node, "robot.imu_limt_num", 1.0);
  declare_param(node, "robot.motor_timeout_ms", 1000);
  declare_param(node, "robot.CANboard.No_1_CANboard.CANport_num", canport_num);

  int remaining_motors = total_motors;
  int global_motor_num = 1;
  for (int port_index = 1; port_index <= canport_num; ++port_index) {
    const int ports_left = canport_num - port_index + 1;
    const int motor_count = std::max(1, remaining_motors / ports_left);
    remaining_motors -= motor_count;

    const std::string port_base =
      "robot.CANboard.No_1_CANboard.CANport.CANport_" + std::to_string(port_index);
    declare_param(node, port_base + ".motor_num", motor_count);
    declare_param(node, port_base + ".serial_id", port_index);
    for (int motor_index = 1; motor_index <= motor_count; ++motor_index) {
      declare_motor(node, port_index, motor_index, motor_index, global_motor_num++);
    }
  }
}
}  // namespace

TEST(RuntimeConfig, LoadsLegacyParameterTree)
{
  auto node = make_node();
  declare_valid_runtime_config(*node);

  const auto config = livelybot_serial::robot::load_runtime_config(*node);

  EXPECT_EQ(config.robot_name, "test_robot");
  EXPECT_EQ(config.serial_baudrate, 921600);
  EXPECT_EQ(config.boards.size(), 1u);
  ASSERT_EQ(config.boards.front().ports.size(), 1u);
  ASSERT_EQ(config.boards.front().ports.front().motors.size(), 1u);
  EXPECT_EQ(config.boards.front().ports.front().motors.front().motor_name, "joint1");
}

TEST(RuntimeConfig, RejectsInvalidControlType)
{
  auto node = make_node();
  declare_valid_runtime_config(*node);
  node->set_parameter(rclcpp::Parameter("robot.control_type", 8));

  EXPECT_THROW(
    livelybot_serial::robot::load_runtime_config(*node),
    std::invalid_argument);
}

TEST(RuntimeConfig, AcceptsRos2StyleTopLevelAliases)
{
  auto node = make_node();
  declare_param(*node, "robot.serial_baudrate", 115200);
  declare_param(*node, "robot.robot_name", std::string("test_robot"));
  declare_param(*node, "robot.canboard_num", 1);
  declare_param(*node, "robot.serial_type", std::string("/dev/ttyUSB"));
  declare_param(*node, "robot.control_type", 9);
  declare_param(*node, "robot.imu_limit_flag", true);
  declare_param(*node, "robot.imu_dir", false);
  declare_param(*node, "robot.imu_limit_num", 0.5);
  declare_param(*node, "robot.motor_timeout_ms", 1000);

  declare_param(*node, "robot.CANboard.No_1_CANboard.CANport_num", 1);
  declare_param(*node, "robot.CANboard.No_1_CANboard.CANport.CANport_1.motor_num", 1);
  declare_param(*node, "robot.CANboard.No_1_CANboard.CANport.CANport_1.serial_id", 1);

  const std::string motor_base =
    "robot.CANboard.No_1_CANboard.CANport.CANport_1.motor.motor1";
  declare_param(*node, motor_base + ".name", std::string("joint1"));
  declare_param(*node, motor_base + ".id", 1);
  declare_param(*node, motor_base + ".type", std::string("5046_20"));
  declare_param(*node, motor_base + ".num", 1);
  declare_param(*node, motor_base + ".pos_limit_enable", true);
  declare_param(*node, motor_base + ".pos_upper", 1.0);
  declare_param(*node, motor_base + ".pos_lower", -1.0);
  declare_param(*node, motor_base + ".tor_limit_enable", true);
  declare_param(*node, motor_base + ".tor_upper", 2.0);
  declare_param(*node, motor_base + ".tor_lower", -2.0);

  const auto config = livelybot_serial::robot::load_runtime_config(*node);

  EXPECT_EQ(config.serial_baudrate, 115200);
  EXPECT_EQ(config.serial_type, "/dev/ttyUSB");
  EXPECT_TRUE(config.imu_limit_flag);
  EXPECT_FLOAT_EQ(config.imu_limit_num, 0.5f);
}

TEST(RuntimeConfig, RejectsDuplicateMotorIdsWithinPort)
{
  auto node = make_node();
  declare_valid_runtime_config(*node);
  node->set_parameter(
    rclcpp::Parameter("robot.CANboard.No_1_CANboard.CANport.CANport_1.motor_num", 2));

  const std::string motor_base =
    "robot.CANboard.No_1_CANboard.CANport.CANport_1.motor.motor2";
  declare_param(*node, motor_base + ".name", std::string("joint2"));
  declare_param(*node, motor_base + ".id", 1);
  declare_param(*node, motor_base + ".type", std::string("5046_20"));
  declare_param(*node, motor_base + ".num", 2);
  declare_param(*node, motor_base + ".pos_limit_enable", false);
  declare_param(*node, motor_base + ".pos_upper", 0.0);
  declare_param(*node, motor_base + ".pos_lower", 0.0);
  declare_param(*node, motor_base + ".tor_limit_enable", false);
  declare_param(*node, motor_base + ".tor_upper", 0.0);
  declare_param(*node, motor_base + ".tor_lower", 0.0);

  EXPECT_THROW(
    livelybot_serial::robot::load_runtime_config(*node),
    std::invalid_argument);
}

TEST(RuntimeConfig, RejectsInvalidPositionLimitRange)
{
  auto node = make_node();
  declare_valid_runtime_config(*node);
  const std::string motor_base =
    "robot.CANboard.No_1_CANboard.CANport.CANport_1.motor.motor1";
  node->set_parameter(rclcpp::Parameter(motor_base + ".pos_upper", -2.0));
  node->set_parameter(rclcpp::Parameter(motor_base + ".pos_lower", 2.0));

  EXPECT_THROW(
    livelybot_serial::robot::load_runtime_config(*node),
    std::invalid_argument);
}

TEST(RuntimeConfig, RejectsInvalidTorqueLimitRange)
{
  auto node = make_node();
  declare_valid_runtime_config(*node);
  const std::string motor_base =
    "robot.CANboard.No_1_CANboard.CANport.CANport_1.motor.motor1";
  node->set_parameter(rclcpp::Parameter(motor_base + ".tor_upper", -2.0));
  node->set_parameter(rclcpp::Parameter(motor_base + ".tor_lower", 2.0));

  EXPECT_THROW(
    livelybot_serial::robot::load_runtime_config(*node),
    std::invalid_argument);
}

TEST(RuntimeConfig, AcceptsSevenCanPorts)
{
  auto node = make_node();
  declare_runtime_config_with_ports(*node, 7, 30);

  const auto config = livelybot_serial::robot::load_runtime_config(*node);

  ASSERT_EQ(config.boards.size(), 1u);
  EXPECT_EQ(config.boards.front().ports.size(), 7u);
}

TEST(RuntimeConfig, RejectsMoreThanSevenCanPorts)
{
  auto node = make_node();
  declare_runtime_config_with_ports(*node, 8, 30);

  EXPECT_THROW(
    livelybot_serial::robot::load_runtime_config(*node),
    std::invalid_argument);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(new RclcppTestEnvironment);
  return RUN_ALL_TESTS();
}
