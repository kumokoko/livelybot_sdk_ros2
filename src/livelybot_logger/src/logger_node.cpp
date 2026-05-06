#include "livelybot_logger/logger_interface.hpp"

#include "livelybot_logger/msg/logger_operation.hpp"
#include "livelybot_power/msg/power_switch.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{
constexpr const char * kSocketPath = "/tmp/logger_service.sock";
constexpr int kMotorCount = 12;

std::string exec_system_command(const char * cmd)
{
  std::array<char, 256> buffer{};
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe) {
    return result;
  }

  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
    result += buffer.data();
  }

  return result;
}
}  // namespace

struct MotorStatus
{
  double position{0.0};
  double velocity{0.0};
  double torque{0.0};
  bool is_valid{false};
};

struct ImuStatus
{
  struct
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  } acceleration;

  struct
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  } angular_velocity;

  struct
  {
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
  } euler_angles;

  bool is_valid{false};
};

struct BatteryStatus
{
  double voltage{0.0};
  double current{0.0};
  double temperature{0.0};
  double remaining_percentage{0.0};
  bool is_valid{false};
};

struct PowerStatus
{
  enum class SwitchState
  {
    UNKNOWN = -1,
    OFF = 0,
    ON = 1
  };

  SwitchState system_power{SwitchState::UNKNOWN};
  SwitchState motor_power{SwitchState::UNKNOWN};
  bool is_valid{false};
};

class LoggerNode : public rclcpp::Node
{
public:
  LoggerNode()
  : Node("livelybot_logger")
  {
    heartbeat_interval_ = declare_parameter<double>("heartbeat_interval", 1.0);
    power_timeout_ = declare_parameter<double>("power_timeout", 1.0);
    status_send_interval_ = declare_parameter<double>("status_send_interval", 5.0);
    monitored_nodes_ = declare_parameter<std::vector<std::string>>(
      "monitored_nodes",
      std::vector<std::string>{
        "/power_node", "/yesense_imu", "/motor_driver_node", "/livelybot_logger",
        "/livelybot_oled", "/robot_emergency_stop", "/robot_falldown_protect"});

    initialize_socket();
    livelybot_logger::LoggerInterface::init(*this, get_name());

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/error_joint_states", 10,
      std::bind(&LoggerNode::joint_state_callback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10, std::bind(&LoggerNode::imu_callback, this, std::placeholders::_1));
    power_switch_sub_ = create_subscription<livelybot_power::msg::PowerSwitch>(
      "/power_switch_state", 10,
      std::bind(&LoggerNode::power_switch_callback, this, std::placeholders::_1));
    battery_volt_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/battery_voltage", 10,
      std::bind(&LoggerNode::battery_volt_callback, this, std::placeholders::_1));
    battery_curr_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/battery_current", 10,
      std::bind(&LoggerNode::battery_curr_callback, this, std::placeholders::_1));
    battery_temp_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/battery_temperature", 10,
      std::bind(&LoggerNode::battery_temp_callback, this, std::placeholders::_1));
    bms_error_sub_ = create_subscription<std_msgs::msg::Int8>(
      "/bms_error", 10, std::bind(&LoggerNode::bms_error_callback, this, std::placeholders::_1));
    operation_sub_ = create_subscription<livelybot_logger::msg::LoggerOperation>(
      "/logger/operation", 10,
      std::bind(&LoggerNode::operation_callback, this, std::placeholders::_1));

    heartbeat_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(heartbeat_interval_)),
      std::bind(&LoggerNode::heartbeat_callback, this));
    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(status_send_interval_)),
      std::bind(&LoggerNode::status_callback, this));
    power_timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&LoggerNode::power_timeout_callback, this));

    send_message("OPERATION:livelybot_logger_start");
    RCLCPP_INFO(get_logger(), "Logger node started.");
  }

  ~LoggerNode() override
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
  }

private:
  void initialize_socket()
  {
    socket_fd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (socket_fd_ == -1) {
      RCLCPP_ERROR(get_logger(), "Failed to create logger socket");
    }
  }

  bool send_message(const std::string & message)
  {
    if (socket_fd_ == -1 || message.empty()) {
      return false;
    }

    struct sockaddr_un addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, kSocketPath, sizeof(addr.sun_path) - 1);

    const auto sent = sendto(
      socket_fd_, message.c_str(), message.size(), 0, reinterpret_cast<struct sockaddr *>(&addr),
      sizeof(addr));
    return sent == static_cast<ssize_t>(message.size());
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (auto & motor : motors_) {
      motor.is_valid = false;
    }

    const auto count = std::min(msg->position.size(), motors_.size());
    for (size_t i = 0; i < count; ++i) {
      motors_[i].position = msg->position[i];
      motors_[i].velocity = i < msg->velocity.size() ? msg->velocity[i] : 0.0;
      motors_[i].torque = i < msg->effort.size() ? msg->effort[i] : 0.0;
      motors_[i].is_valid = true;
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    imu_.acceleration.x = msg->linear_acceleration.x;
    imu_.acceleration.y = msg->linear_acceleration.y;
    imu_.acceleration.z = msg->linear_acceleration.z;
    imu_.angular_velocity.x = msg->angular_velocity.x;
    imu_.angular_velocity.y = msg->angular_velocity.y;
    imu_.angular_velocity.z = msg->angular_velocity.z;

    tf2::Quaternion q(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 rot_mat(q);
    rot_mat.getRPY(imu_.euler_angles.roll, imu_.euler_angles.pitch, imu_.euler_angles.yaw);
    imu_.is_valid = true;
  }

  void power_switch_callback(const livelybot_power::msg::PowerSwitch::SharedPtr msg)
  {
    last_power_update_ = now();
    power_.system_power =
      msg->control_switch == 1 ? PowerStatus::SwitchState::ON : PowerStatus::SwitchState::OFF;
    power_.motor_power =
      msg->power_switch == 1 ? PowerStatus::SwitchState::ON : PowerStatus::SwitchState::OFF;
    power_.is_valid = true;
  }

  void battery_volt_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    battery_.voltage = msg->data;
    battery_.is_valid = true;
  }

  void battery_curr_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    battery_.current = msg->data;
  }

  void battery_temp_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    battery_.temperature = msg->data;
  }

  void bms_error_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    std::ostringstream stream;
    stream << "BMS_ERROR_CODE:" << static_cast<int>(msg->data);
    livelybot_logger::LoggerInterface::log_operation("ERROR", stream.str());
  }

  void operation_callback(const livelybot_logger::msg::LoggerOperation::SharedPtr msg)
  {
    send_message("OPERATION:" + msg->operation_type + " | " + msg->operation_data);
  }

  void power_timeout_callback()
  {
    if ((now() - last_power_update_).seconds() > power_timeout_) {
      power_.is_valid = false;
      power_.system_power = PowerStatus::SwitchState::UNKNOWN;
      power_.motor_power = PowerStatus::SwitchState::UNKNOWN;
    }
  }

  void heartbeat_callback()
  {
    send_message("HEARTBEAT");
  }

  void status_callback()
  {
    update_system_resource_info();
    update_node_status_info();
    check_system_status();

    std::ostringstream status_msg;
    status_msg << "STATUS:MOTOR;";
    for (size_t i = 0; i < motors_.size(); ++i) {
      status_msg << "M" << (i + 1) << ",";
      if (motors_[i].is_valid) {
        status_msg << "pos:" << std::fixed << std::setprecision(6) << motors_[i].position << ","
                   << "vel:" << motors_[i].velocity << ",tor:" << motors_[i].torque;
      } else {
        status_msg << "pos:NULL,vel:NULL,tor:NULL";
      }
      if (i + 1 < motors_.size()) {
        status_msg << ";";
      }
    }

    status_msg << ";IMU,";
    if (imu_.is_valid) {
      status_msg << "acc:" << imu_.acceleration.x << "/" << imu_.acceleration.y << "/"
                 << imu_.acceleration.z << ",gyro:" << imu_.angular_velocity.x << "/"
                 << imu_.angular_velocity.y << "/" << imu_.angular_velocity.z << ",euler:"
                 << imu_.euler_angles.roll << "/" << imu_.euler_angles.pitch << "/"
                 << imu_.euler_angles.yaw;
    } else {
      status_msg << "acc:NULL/NULL/NULL,gyro:NULL/NULL/NULL,euler:NULL/NULL/NULL";
    }

    status_msg << ";POWER,";
    if (power_.is_valid) {
      status_msg << "sys:"
                 << (power_.system_power == PowerStatus::SwitchState::ON   ? "1"
                     : power_.system_power == PowerStatus::SwitchState::OFF ? "0"
                                                                            : "NULL")
                 << ",motor:"
                 << (power_.motor_power == PowerStatus::SwitchState::ON   ? "1"
                     : power_.motor_power == PowerStatus::SwitchState::OFF ? "0"
                                                                           : "NULL");
    } else {
      status_msg << "sys:NULL,motor:NULL";
    }

    status_msg << ";BATTERY,volt:" << (battery_.is_valid ? std::to_string(battery_.voltage) : "NULL")
               << ",curr:" << (battery_.is_valid ? std::to_string(battery_.current) : "NULL")
               << ",temp:" << (battery_.is_valid ? std::to_string(battery_.temperature) : "NULL")
               << ",remain:"
               << (battery_.is_valid ? std::to_string(battery_.remaining_percentage) : "NULL");

    status_msg << ";SYSTEM_RESOURCES,cpu:" << system_cpu_usage_ << "%,mem:" << system_memory_usage_
               << "%,disk:" << system_disk_usage_ << "%,cpu_temp:" << cpu_temperature_ << "C";

    for (const auto & entry : node_statuses_) {
      status_msg << ";ROS_NODE,node:" << entry.first << ",status:" << entry.second.first
                 << ",cpu:" << entry.second.second.first
                 << ",memory:" << entry.second.second.second;
    }

    send_message(status_msg.str());
  }

  void update_system_resource_info()
  {
    const auto cpu_output = exec_system_command("top -bn1 | grep 'Cpu(s)' | awk '{print $2 + $4}'");
    if (!cpu_output.empty()) {
      system_cpu_usage_ = std::stof(cpu_output);
    }

    const auto mem_output = exec_system_command("free | grep Mem | awk '{print $3/$2 * 100.0}'");
    if (!mem_output.empty()) {
      system_memory_usage_ = std::stof(mem_output);
    }

    const auto disk_output =
      exec_system_command("df -h / | grep / | awk '{print $5}' | sed 's/%//'");
    if (!disk_output.empty()) {
      system_disk_usage_ = std::stof(disk_output);
    }

    const auto temp_output = exec_system_command(
      "cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -nr | head -n1");
    if (!temp_output.empty()) {
      cpu_temperature_ = std::stof(temp_output) / 1000.0;
    }
  }

  void update_node_status_info()
  {
    node_statuses_.clear();
    const auto node_list_output = exec_system_command("ros2 node list 2>/dev/null");
    const auto process_output = exec_system_command("ps aux");

    for (const auto & node_name : monitored_nodes_) {
      std::string status = node_list_output.find(node_name) != std::string::npos ? "run" : "stopped";
      std::string cpu_usage = "NULL";
      std::string memory_usage = "NULL";

      std::istringstream stream(process_output);
      std::string line;
      const auto basename = node_name.rfind('/') == 0 ? node_name.substr(1) : node_name;
      while (std::getline(stream, line)) {
        if (line.find(basename) == std::string::npos) {
          continue;
        }
        std::istringstream line_stream(line);
        std::string user;
        std::string pid;
        line_stream >> user >> pid >> cpu_usage >> memory_usage;
        break;
      }

      node_statuses_[node_name] = {status, {cpu_usage, memory_usage}};
    }
  }

  void check_system_status()
  {
    if (battery_.is_valid && battery_.voltage < 10.0) {
      livelybot_logger::LoggerInterface::log_operation(
        "ERROR", "LOW_BATTERY:" + std::to_string(battery_.voltage) + "V");
    }

    if (cpu_temperature_ > 80.0) {
      livelybot_logger::LoggerInterface::log_operation(
        "ERROR", "HIGH_CPU_TEMP:" + std::to_string(cpu_temperature_) + "C");
    }
  }

  std::array<MotorStatus, kMotorCount> motors_{};
  ImuStatus imu_{};
  BatteryStatus battery_{};
  PowerStatus power_{};

  int socket_fd_{-1};
  double heartbeat_interval_{1.0};
  double power_timeout_{1.0};
  double status_send_interval_{5.0};
  double system_cpu_usage_{0.0};
  double system_memory_usage_{0.0};
  double system_disk_usage_{0.0};
  double cpu_temperature_{0.0};
  rclcpp::Time last_power_update_{0, 0, RCL_ROS_TIME};
  std::vector<std::string> monitored_nodes_;
  std::map<std::string, std::pair<std::string, std::pair<std::string, std::string>>> node_statuses_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<livelybot_power::msg::PowerSwitch>::SharedPtr power_switch_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_volt_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_curr_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_temp_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr bms_error_sub_;
  rclcpp::Subscription<livelybot_logger::msg::LoggerOperation>::SharedPtr operation_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr power_timeout_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoggerNode>());
  rclcpp::shutdown();
  return 0;
}
