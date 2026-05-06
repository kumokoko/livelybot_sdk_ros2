#include "sensor_actuator_status.hpp"

#include <dirent.h>
#include <libserialport.h>

#include <algorithm>
#include <cstring>
#include <iostream>

Sensor_actuator_status::Sensor_actuator_status(
  int can1_num, int can2_num, int can3_num, int can4_num, const std::string & device_name)
{
  motor_status_.can1_num = can1_num;
  motor_status_.can2_num = can2_num;
  motor_status_.can3_num = can3_num;
  motor_status_.can4_num = can4_num;

  if (device_name == "/dev/ttyACM") {
    const auto ports = list_serial_ports(device_name);
    for (const auto & port : ports) {
      if (serial_pid_vid(port.c_str()) > 0) {
        serial_.setPort(port);
        break;
      }
    }
  } else {
    serial_.setPort(device_name);
  }

  serial_.setBaudrate(115200);
  auto timeout = serial::Timeout::simpleTimeout(1000);
  serial_.setTimeout(timeout);

  try {
    serial_.open();
  } catch (const std::exception &) {
  }

  if (serial_.isOpen()) {
    success_flag_ = 1;
  }
}

Sensor_actuator_status::~Sensor_actuator_status()
{
  if (serial_.isOpen()) {
    serial_.close();
  }
}

void Sensor_actuator_status::send_imu_status(bool imu_exist, float * rpy)
{
  imu_status_.imu_con_sta = imu_exist;

  send_buffer_[0] = 0xA5;
  send_buffer_[1] = 0x5A;
  send_buffer_[2] = 14;
  send_buffer_[3] = 0x10;
  send_buffer_[4] = imu_status_.imu_con_sta;
  std::memcpy(&send_buffer_[5], rpy, 12);
  send_buffer_[17] = 0x66;
  send_buffer_[18] = 0x47;
  send_buffer_[19] = 0x74;

  serial_.write(send_buffer_, send_buffer_[2] + 6);
}

void Sensor_actuator_status::send_motor_status(unsigned char * motor_status)
{
  send_buffer_[0] = 0xA5;
  send_buffer_[1] = 0x5A;
  send_buffer_[2] = 3 + motor_status_.can1_num + motor_status_.can2_num + motor_status_.can3_num +
                    motor_status_.can4_num;
  send_buffer_[3] = 0x11;
  send_buffer_[4] =
    (motor_status_.can1_num & 0x0F) | ((motor_status_.can2_num & 0x0F) << 4);
  send_buffer_[5] =
    (motor_status_.can3_num & 0x0F) | ((motor_status_.can4_num & 0x0F) << 4);
  std::memcpy(
    &send_buffer_[6], motor_status,
    motor_status_.can1_num + motor_status_.can2_num + motor_status_.can3_num +
      motor_status_.can4_num);
  send_buffer_[send_buffer_[2] + 3] = 0x66;
  send_buffer_[send_buffer_[2] + 4] = 0x47;
  send_buffer_[send_buffer_[2] + 5] = 0x74;
  serial_.write(send_buffer_, send_buffer_[2] + 6);
}

void Sensor_actuator_status::send_ip_addr(unsigned int * ip_data, unsigned char len)
{
  send_buffer_[0] = 0xA5;
  send_buffer_[1] = 0x5A;
  send_buffer_[2] = len * 4 + 1;
  send_buffer_[3] = 0x12;
  std::memcpy(&send_buffer_[4], ip_data, len * 4);
  send_buffer_[send_buffer_[2] + 3] = 0x66;
  send_buffer_[send_buffer_[2] + 4] = 0x47;
  send_buffer_[send_buffer_[2] + 5] = 0x74;
  serial_.write(send_buffer_, send_buffer_[2] + 6);
}

void Sensor_actuator_status::send_battery_level(uint8_t level)
{
  send_buffer_[0] = 0xA5;
  send_buffer_[1] = 0x5A;
  send_buffer_[2] = 2;
  send_buffer_[3] = 0x13;
  std::memcpy(&send_buffer_[4], &level, 1);
  send_buffer_[send_buffer_[2] + 3] = 0x66;
  send_buffer_[send_buffer_[2] + 4] = 0x47;
  send_buffer_[send_buffer_[2] + 5] = 0x74;
  serial_.write(send_buffer_, send_buffer_[2] + 6);
}

void Sensor_actuator_status::send_fsm_state(int32_t fsm_state)
{
  send_buffer_[0] = 0xA5;
  send_buffer_[1] = 0x5A;
  send_buffer_[2] = 5;
  send_buffer_[3] = 0x14;
  std::memcpy(&send_buffer_[4], &fsm_state, 4);
  send_buffer_[send_buffer_[2] + 3] = 0x66;
  send_buffer_[send_buffer_[2] + 4] = 0x47;
  send_buffer_[send_buffer_[2] + 5] = 0x74;
  serial_.write(send_buffer_, send_buffer_[2] + 6);
}

int Sensor_actuator_status::serial_pid_vid(const char * name)
{
  int pid = 0;
  int vid = 0;
  int result = 0;
  struct sp_port * port = nullptr;

  sp_get_port_by_name(name, &port);
  if (!port) {
    return -1;
  }

  sp_open(port, SP_MODE_READ);
  if (sp_get_port_usb_vid_pid(port, &vid, &pid) != SP_OK) {
    result = -1;
  } else if (vid == 1155 && pid == 22339) {
    result = 1;
  } else {
    result = -2;
  }

  sp_close(port);
  sp_free_port(port);
  return result;
}

std::vector<std::string> Sensor_actuator_status::list_serial_ports(const std::string & full_prefix)
{
  const std::string base_path = full_prefix.substr(0, full_prefix.rfind('/') + 1);
  const std::string prefix = full_prefix.substr(full_prefix.rfind('/') + 1);
  std::vector<std::string> serial_ports;

  DIR * directory = opendir(base_path.c_str());
  if (!directory) {
    std::cerr << "Could not open directory " << base_path << std::endl;
    return serial_ports;
  }

  struct dirent * entry = nullptr;
  while ((entry = readdir(directory)) != nullptr) {
    std::string entry_name = entry->d_name;
    if (entry_name.find(prefix) == 0) {
      serial_ports.push_back(base_path + entry_name);
    }
  }

  closedir(directory);
  std::sort(serial_ports.begin(), serial_ports.end());
  return serial_ports;
}
