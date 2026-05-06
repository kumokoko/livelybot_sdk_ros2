#include <cmath>
#include <memory>
#include <string>

#include "livelybot_power/msg/power_switch.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"

namespace
{
struct AngleRange
{
  double min{0.0};
  double max{0.0};
  bool rotable{false};
  bool enabled{false};
};

struct WarningLevel
{
  double minor{0.0};
  double major{0.0};
};

struct AttitudeConfig
{
  bool enabled{false};
  AngleRange roll;
  AngleRange pitch;
  AngleRange yaw;
  WarningLevel warning;
};

double radian_to_degree(double radian)
{
  return radian * 180.0 / M_PI;
}
}  // namespace

class RobotFalldownProtectNode : public rclcpp::Node
{
public:
  RobotFalldownProtectNode()
  : rclcpp::Node(
      "robot_falldown_protect",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    const auto config_path = declare_parameter<std::string>(
      "config_path", "cfg/falldown_condition_pi.yaml");
    load_config(config_path);

    power_pub_ = create_publisher<livelybot_power::msg::PowerSwitch>("power_switch_control", 10);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 20, std::bind(&RobotFalldownProtectNode::imu_callback, this, std::placeholders::_1));
  }

private:
  void load_config(const std::string & config_path)
  {
    const auto config = YAML::LoadFile(config_path);
    config_.enabled = config["enable"].as<bool>();

    const auto warning = config["warning_level"];
    config_.warning.minor = warning["minor"].as<double>();
    config_.warning.major = warning["major"].as<double>();

    const auto thresholds = config["thresholds"];
    load_range(thresholds["roll"], config_.roll);
    load_range(thresholds["pitch"], config_.pitch);
    load_range(thresholds["yaw"], config_.yaw);

    RCLCPP_INFO(
      get_logger(),
      "Falldown protection config loaded from %s (enabled=%s)",
      config_path.c_str(),
      config_.enabled ? "true" : "false");
  }

  static void load_range(const YAML::Node & node, AngleRange & range)
  {
    range.min = node["min"].as<double>();
    range.max = node["max"].as<double>();
    range.rotable = node["rotable"].as<bool>();
    range.enabled = node["enable"].as<bool>();
  }

  static double normalize_angle(double value, const AngleRange & range)
  {
    if (range.rotable && value < 0.0) {
      return value + 360.0;
    }
    return value;
  }

  static bool range_triggered(double value, const AngleRange & range)
  {
    if (!range.enabled) {
      return false;
    }
    return value < range.min || value > range.max;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!config_.enabled || already_cut_power_) {
      return;
    }

    tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);
    tf2::Matrix3x3 rot_mat(q);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    rot_mat.getRPY(roll, pitch, yaw);

    roll = normalize_angle(radian_to_degree(roll), config_.roll);
    pitch = normalize_angle(radian_to_degree(pitch), config_.pitch);
    yaw = normalize_angle(radian_to_degree(yaw), config_.yaw);

    if (!range_triggered(roll, config_.roll) &&
      !range_triggered(pitch, config_.pitch) &&
      !range_triggered(yaw, config_.yaw))
    {
      return;
    }

    livelybot_power::msg::PowerSwitch power_switch;
    power_switch.control_switch = 1;
    power_switch.power_switch = 0;
    power_pub_->publish(power_switch);
    already_cut_power_ = true;

    RCLCPP_ERROR(
      get_logger(),
      "Falldown protection triggered. roll=%.2f pitch=%.2f yaw=%.2f",
      roll, pitch, yaw);
  }

  AttitudeConfig config_;
  bool already_cut_power_{false};
  rclcpp::Publisher<livelybot_power::msg::PowerSwitch>::SharedPtr power_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotFalldownProtectNode>());
  rclcpp::shutdown();
  return 0;
}
