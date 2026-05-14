#include "hardware/motor.h"

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

namespace
{
motor::config make_motor_config(int control_type)
{
  motor::config config;
  config.motor_name = "joint1";
  config.id = 1;
  config.num = 1;
  config.canport_num = 1;
  config.canboard_num = 1;
  config.type_name = "5046_20";
  config.control_type = control_type;
  return config;
}
}  // namespace

TEST(MotorCommands, EncodesPositionVelocityTorqueKpKdCommand)
{
  cdc_tr_message_s tx_message{};
  motor m(make_motor_config(9), &tx_message, 1, rclcpp::get_logger("test_motor_commands"));

  m.fresh_cmd_int16(1.0f, 2.0f, 0.3f, 10.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);

  EXPECT_EQ(tx_message.head.s.head, 0xF7);
  EXPECT_EQ(tx_message.head.s.cmd, MODE_POS_VEL_TQE_KP_KD);
  EXPECT_EQ(tx_message.head.s.len, sizeof(motor_pos_val_tqe_rpd_s));
  EXPECT_NE(tx_message.data.pos_val_tqe_rpd[0].pos, static_cast<int16_t>(-32768));
  EXPECT_NE(tx_message.data.pos_val_tqe_rpd[0].val, 0);
  EXPECT_NE(tx_message.data.pos_val_tqe_rpd[0].tqe, 0);
  EXPECT_NE(tx_message.data.pos_val_tqe_rpd[0].rkp, 0);
  EXPECT_NE(tx_message.data.pos_val_tqe_rpd[0].rkd, 0);
}

TEST(MotorCommands, EncodesSimpleControlModes)
{
  {
    cdc_tr_message_s tx_message{};
    motor m(make_motor_config(1), &tx_message, 1, rclcpp::get_logger("test_motor_commands"));
    m.fresh_cmd_int16(1.0f, 2.0f, 0.3f, 10.0f, 0.0f, 1.0f, 0.0f, 3.0f, 4.0f);
    EXPECT_EQ(tx_message.head.s.cmd, MODE_POSITION);
    EXPECT_EQ(tx_message.head.s.len, sizeof(int16_t));
    EXPECT_NE(tx_message.data.position[0], static_cast<int16_t>(-32768));
  }

  {
    cdc_tr_message_s tx_message{};
    motor m(make_motor_config(2), &tx_message, 1, rclcpp::get_logger("test_motor_commands"));
    m.fresh_cmd_int16(1.0f, 2.0f, 0.3f, 10.0f, 0.0f, 1.0f, 0.0f, 3.0f, 4.0f);
    EXPECT_EQ(tx_message.head.s.cmd, MODE_VELOCITY);
    EXPECT_EQ(tx_message.head.s.len, sizeof(int16_t));
    EXPECT_NE(tx_message.data.velocity[0], 0);
  }

  {
    cdc_tr_message_s tx_message{};
    motor m(make_motor_config(3), &tx_message, 1, rclcpp::get_logger("test_motor_commands"));
    m.fresh_cmd_int16(1.0f, 2.0f, 0.3f, 10.0f, 0.0f, 1.0f, 0.0f, 3.0f, 4.0f);
    EXPECT_EQ(tx_message.head.s.cmd, MODE_TORQUE);
    EXPECT_EQ(tx_message.head.s.len, sizeof(int16_t));
    EXPECT_NE(tx_message.data.torque[0], 0);
  }

  {
    cdc_tr_message_s tx_message{};
    motor m(make_motor_config(4), &tx_message, 1, rclcpp::get_logger("test_motor_commands"));
    m.fresh_cmd_int16(1.0f, 2.0f, 0.3f, 10.0f, 0.0f, 1.0f, 0.0f, 3.0f, 4.0f);
    EXPECT_EQ(tx_message.head.s.cmd, MODE_VOLTAGE);
    EXPECT_EQ(tx_message.head.s.len, sizeof(int16_t));
    EXPECT_EQ(tx_message.data.voltage[0], 30);
  }

  {
    cdc_tr_message_s tx_message{};
    motor m(make_motor_config(5), &tx_message, 1, rclcpp::get_logger("test_motor_commands"));
    m.fresh_cmd_int16(1.0f, 2.0f, 0.3f, 10.0f, 0.0f, 1.0f, 0.0f, 3.0f, 4.0f);
    EXPECT_EQ(tx_message.head.s.cmd, MODE_CURRENT);
    EXPECT_EQ(tx_message.head.s.len, sizeof(int16_t));
    EXPECT_EQ(tx_message.data.current[0], 40);
  }
}

TEST(MotorCommands, EncodesCompositeControlModes)
{
  {
    cdc_tr_message_s tx_message{};
    motor m(make_motor_config(6), &tx_message, 1, rclcpp::get_logger("test_motor_commands"));
    m.fresh_cmd_int16(1.0f, 2.0f, 0.3f, 10.0f, 0.0f, 1.0f, 0.4f, 0.0f, 0.0f);
    EXPECT_EQ(tx_message.head.s.cmd, MODE_POS_VEL_TQE);
    EXPECT_EQ(tx_message.head.s.len, sizeof(motor_pos_val_tqe_s));
    EXPECT_NE(tx_message.data.pos_val_tqe[0].pos, static_cast<int16_t>(-32768));
  }

  {
    cdc_tr_message_s tx_message{};
    motor m(make_motor_config(10), &tx_message, 1, rclcpp::get_logger("test_motor_commands"));
    m.fresh_cmd_int16(1.0f, 2.0f, 0.3f, 10.0f, 0.0f, 1.0f, 0.4f, 0.0f, 0.0f);
    EXPECT_EQ(tx_message.head.s.cmd, MODE_POS_VEL_KP_KD);
    EXPECT_EQ(tx_message.head.s.len, sizeof(motor_pos_val_rpd_s));
    EXPECT_NE(tx_message.data.pos_val_rpd[0].rkp, 0);
  }

  {
    cdc_tr_message_s tx_message{};
    motor m(make_motor_config(11), &tx_message, 1, rclcpp::get_logger("test_motor_commands"));
    m.fresh_cmd_int16(1.0f, 2.0f, 0.3f, 10.0f, 0.0f, 1.0f, 0.4f, 0.0f, 0.0f);
    EXPECT_EQ(tx_message.head.s.cmd, MODE_POS_VEL_ACC);
    EXPECT_EQ(tx_message.head.s.len, sizeof(motor_pos_val_acc_s));
    EXPECT_EQ(tx_message.data.pos_val_acc[0].acc, 400);
  }

  {
    cdc_tr_message_s tx_message{};
    motor m(make_motor_config(12), &tx_message, 1, rclcpp::get_logger("test_motor_commands"));
    m.fresh_cmd_int16(1.0f, 2.0f, 0.3f, 10.0f, 0.0f, 1.0f, 0.4f, 0.0f, 0.0f);
    EXPECT_EQ(tx_message.head.s.cmd, MODE_POS_VEL_TQE_KP_KD2);
    EXPECT_EQ(tx_message.head.s.len, sizeof(motor_pos_val_tqe_rpd_s));
    EXPECT_NE(tx_message.data.pos_val_tqe_rpd[0].rkp, 0);
  }
}

TEST(MotorCommands, RejectsDeprecatedControlMode)
{
  cdc_tr_message_s tx_message{};
  motor m(make_motor_config(7), &tx_message, 1, rclcpp::get_logger("test_motor_commands"));

  EXPECT_THROW(
    m.fresh_cmd_int16(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
    std::runtime_error);
}
