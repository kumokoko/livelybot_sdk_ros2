#ifndef LIVELYBOT_SERIAL_HARDWARE_ROBOT_H
#define LIVELYBOT_SERIAL_HARDWARE_ROBOT_H

#include <atomic>
#include <functional>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "canboard.h"
#include "rclcpp/node.hpp"
#include "rclcpp/logger.hpp"

namespace livelybot_serial
{
class robot
{
public:
  enum error_run_state
  {
    error_check = 0,
    error_clear,
    error_wait_dev,
    error_reconnect,
  };
  using error_run_state_e = error_run_state;

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

  struct motor_command
  {
    float position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
  };

private:
  std::vector<canboard> can_boards_;
  std::thread error_check_thread_;
  std::atomic<bool> error_check_running_{false};
  std::vector<std::unique_ptr<lively_serial>> serial_devices_;
  std::vector<lively_serial *> serials_;
  std::vector<motor *> motors_;
  std::vector<canport *> can_ports_;
  std::vector<std::thread> serial_receive_threads_;
  mutable std::mutex runtime_mutex_;
  fun_version function_version_ = fun_v1;
  float slave_version_ = 3.0f;
  float roll_ = 0.0f;
  float pitch_ = 0.0f;

  int motor_position_limit_state_ = 0;
  int motor_torque_limit_state_ = 0;
  runtime_config config_;
  std::function<double()> now_seconds_;
  rclcpp::Logger logger_;

  static void validate_runtime_config(const runtime_config &config);
  std::vector<lively_serial *> collect_board_serials(const canboard::config &board_config, size_t &serial_offset);
  std::vector<std::string> discover_matching_serial_ports();
  void create_serial_devices_for_config(const std::vector<std::string> &ports);
  void start_serial_receivers();
  void build_runtime_topology();
  void validate_runtime_topology() const;
  void clear_runtime_topology();
  void stop_serial_receivers();
  void destroy_serial_devices();
  void start_error_check_thread();
  void stop_error_check_thread();
  void initialize_hardware();
  void cleanup_runtime();
  void reset_all_boards(int repeat_count);
  void repeat_for_all_boards(int repeat_count, double interval_seconds, void (canboard::*operation)());
  void for_each_board(void (canboard::*operation)());
  void repeat_timeout_for_all_boards(int repeat_count, double interval_seconds, int16_t t_ms);
  void repeat_timeout_for_port(int repeat_count, double interval_seconds, uint8_t portx, int16_t t_ms);
  void validate_timeout_ms(int16_t t_ms) const;
  void stop_motors_unlocked();
  void reset_motors_unlocked();
  void reset_zero_positions_unlocked();
  void reset_zero_positions_unlocked(std::initializer_list<int> motors);
  void set_motor_timeout_unlocked(int16_t t_ms);
  void set_motor_timeout_unlocked(uint8_t portx, int16_t t_ms);
  void reset_data_unlocked();
  void enter_canboard_bootloader_unlocked();
  void reset_canboard_fdcan_unlocked();
  int expected_serial_device_count() const;
  void detect_motor_limit();
  void send_motor_command_frame();
  bool is_supported_serial_port(const std::string &name);
  std::vector<std::string> list_serial_ports(const std::string &full_prefix);
  void initialize_serial_devices();
  void check_error();
  int count_existing_serial_devices(int scan_limit);
  void configure_port_motor_counts();
  void request_motor_state();
  void request_motor_version();
  void check_motor_connection_for_firmware();
  void check_motor_connection_position();
  void check_motor_connection_version();
  void motor_version_detection();
  bool imu_limit_ok();
  motor &require_motor(size_t motor_index);
  canport &require_port(size_t port_index);
  canboard &require_board(size_t board_index);
  canport &require_motor_port(const motor &selected_motor);
  void append_motor_state(
    sensor_msgs::msg::JointState &joint_state_msg, motor *m, double now_seconds) const;
  sensor_msgs::msg::JointState build_joint_state_message(
    const builtin_interfaces::msg::Time &stamp, double now_seconds) const;

public:
  static runtime_config load_runtime_config(rclcpp::Node &node);
  static canboard::config load_board_config(rclcpp::Node &node, int cb_id, int control_type);
  static canport::config load_port_config(rclcpp::Node &node, int cb_id, int cp_id, int control_type);
  static motor::config load_motor_config(
    rclcpp::Node &node, int cb_id, int cp_id, int motor_index, int control_type);

  robot(
    const runtime_config &_config, std::function<double()> now_seconds,
    const rclcpp::Logger &logger);
  ~robot();

  void update_imu_state(const sensor_msgs::msg::Imu &msg);
  void apply_motor_commands(const std::vector<motor_command> &commands);
  sensor_msgs::msg::JointState run_control_cycle(
    const builtin_interfaces::msg::Time &stamp, double now_seconds);
  void stop_motors();
  void reset_motors();
  void reset_zero_positions();
  void reset_zero_positions(std::initializer_list<int> motors);
  void enable_motor_runzero();
  void set_motor_timeout(int16_t t_ms);
  void set_motor_timeout(uint8_t portx, int16_t t_ms);
  void reset_data();
  void enter_canboard_bootloader();
  void reset_canboard_fdcan();
};
}  // namespace livelybot_serial

#endif
