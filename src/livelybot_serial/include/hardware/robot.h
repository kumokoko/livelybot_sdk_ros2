#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <algorithm>
#include <cmath>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <thread>

#include <dirent.h>
#include <libserialport.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "canboard.h"
#include "livelybot_serial/ros2_compat.hpp"

namespace livelybot_serial
{
class robot
{
public:
  struct runtime_config
  {
    std::string robot_name;
    std::string serial_type;
    int canboard_num = 0;
    int serial_baudrate = 0;
    int control_type = 0;
    bool imu_limit_flag = false;
    bool imu_dir = false;
    float imu_limit_num = 0.0f;
    int motor_timeout_ms = 0;
    std::vector<canboard::config> boards;
  };

private:
  std::string robot_name, Serial_Type;
  int CANboard_num, Seial_baudrate;
  std::vector<canboard> CANboards;
  std::vector<std::string> str;
  std::string SDK_version2 = "4.4.6";
  std::thread error_check_thread_;
  fun_version fun_v = fun_v1;
  float slave_v = 3.0f;
  int control_type;

  bool imu_limt_flag = false;
  bool imu_dir = false;
  float imu_limt_num = 0.0f;
  float roll = 0.0f;
  float pitch = 0.0f;

  int motor_timeout_ms = 0;
  runtime_config config_;

public:
  std::vector<lively_serial *> ser;
  std::vector<motor *> Motors;
  std::vector<canport *> CANPorts;
  std::vector<std::thread> ser_recv_threads;
  int motor_position_limit_flag = 0;
  int motor_torque_limit_flag = 0;

  static runtime_config load_runtime_config(ros::NodeHandle &node_handle);

  explicit robot(const runtime_config &_config);
  ~robot();

  sensor_msgs::msg::JointState build_joint_state_message() const;
  void update_imu_state(const sensor_msgs::msg::Imu &msg);
  void detect_motor_limit();
  void motor_send_2();
  int serial_pid_vid(const char *name, int *pid, int *vid);
  int serial_pid_vid(const char *name);
  std::vector<std::string> list_serial_ports(const std::string &full_prefix);
  void init_ser();
  void check_error();
  int check_serial_dev_exist(int);
  void set_port_motor_num();
  void send_get_motor_state_cmd();
  void send_get_motor_version_cmd();
  void chevk_motor_connection_position();
  void chevk_motor_connection_version();
  void set_stop();
  void set_reset();
  void set_reset_zero();
  void set_reset_zero(std::initializer_list<int> motors);
  void set_motor_runzero();
  void set_timeout(int16_t t_ms);
  void set_timeout(uint8_t portx, int16_t t_ms);
  void motor_version_detection();
  void set_data_reset();
  void canboard_bootloader();
  void canboard_fdcan_reset();
  bool imu_limt();
};
}  // namespace livelybot_serial

#endif
