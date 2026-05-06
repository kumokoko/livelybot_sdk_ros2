#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "serial/serial.h"

struct imu_status_s
{
  bool imu_con_sta;
  float rpy[3];
};

struct motor_status_s
{
  int can1_num;
  int can2_num;
  int can3_num;
  int can4_num;
  unsigned char motor_status[40];
};

class Sensor_actuator_status
{
public:
  Sensor_actuator_status(
    int can1_num, int can2_num, int can3_num, int can4_num, const std::string & device_name);
  ~Sensor_actuator_status();

  void send_imu_status(bool imu_exist, float * rpy);
  void send_motor_status(unsigned char * motor_status);
  void send_ip_addr(unsigned int * ip_data, unsigned char len);
  void send_battery_level(uint8_t level);
  void send_fsm_state(int32_t fsm_state);

private:
  int serial_pid_vid(const char * name);
  std::vector<std::string> list_serial_ports(const std::string & full_prefix);

  imu_status_s imu_status_{};
  motor_status_s motor_status_{};
  serial::Serial serial_;
  int success_flag_{0};
  unsigned char send_buffer_[64]{};
};
