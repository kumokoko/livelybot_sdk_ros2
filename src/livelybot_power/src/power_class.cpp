#include <memory>
#include <string>

#include "livelybot_can_driver.hpp"
#include "livelybot_power/msg/power_switch.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int8.hpp"

class PowerNode;

namespace
{
using livelybot_power::msg::PowerSwitch;

PowerNode * g_power_node = nullptr;
constexpr uint8_t kBmsAddr = 0x06;
constexpr uint8_t kPowerSwitchAddr = 0x07;
constexpr uint8_t kOrangePiAddr = 0x01;
}  // namespace

class PowerNode : public rclcpp::Node
{
public:
  PowerNode()
  : rclcpp::Node(
      "power_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    can_device_(this->get_parameter("can_device").as_string()),
    can_handler_(can_device_.c_str())
  {
    battery_volt_pub_ = create_publisher<std_msgs::msg::Float32>("battery_voltage", 10);
    battery_curr_pub_ = create_publisher<std_msgs::msg::Float32>("battery_current", 10);
    battery_temp_pub_ = create_publisher<std_msgs::msg::Float32>("battery_temperature", 10);
    battery_level_pub_ = create_publisher<std_msgs::msg::UInt8>("battery_level", 10);
    power_switch_pub_ = create_publisher<PowerSwitch>("power_switch_state", 10);
    bms_error_pub_ = create_publisher<std_msgs::msg::Int8>("bms_error", 10);

    power_switch_sub_ = create_subscription<PowerSwitch>(
      "power_switch_control",
      10,
      std::bind(&PowerNode::power_switch_callback, this, std::placeholders::_1));

    g_power_node = this;
    can_handler_.start_callback(&PowerNode::custom_can_recv_parse);
  }

  static void custom_can_recv_parse(int can_id, unsigned char * data, unsigned char)
  {
    auto * node = g_power_node;
    if (!node) {
      return;
    }

    const auto dev_addr = static_cast<uint8_t>(can_id >> 7);
    const auto data_type = static_cast<uint8_t>((can_id >> 1) & 0x3F);
    const auto append_flag = static_cast<uint8_t>(can_id & 0x01);

    if (!append_flag && dev_addr == kBmsAddr && data_type == 0x01) {
      std_msgs::msg::Float32 volt_msg;
      volt_msg.data = (*reinterpret_cast<int16_t *>(&data[0])) / 100.0f;
      node->battery_volt_pub_->publish(volt_msg);

      std_msgs::msg::Float32 curr_msg;
      curr_msg.data = (*reinterpret_cast<int16_t *>(&data[2])) / 100.0f;
      node->battery_curr_pub_->publish(curr_msg);

      std_msgs::msg::Float32 temp_msg;
      temp_msg.data = (*reinterpret_cast<int16_t *>(&data[4])) / 100.0f;
      node->battery_temp_pub_->publish(temp_msg);

      std_msgs::msg::UInt8 level_msg;
      level_msg.data = data[6];
      node->battery_level_pub_->publish(level_msg);
      return;
    }

    if (!append_flag && dev_addr == kBmsAddr && data_type == 0x02) {
      std_msgs::msg::Int8 error_msg;
      error_msg.data = static_cast<int8_t>(data[0]);
      node->bms_error_pub_->publish(error_msg);
      return;
    }

    if (!append_flag && dev_addr == kPowerSwitchAddr && data_type == 0x02) {
      PowerSwitch power_switch_msg;
      power_switch_msg.control_switch = data[0];
      power_switch_msg.power_switch = data[1];
      node->power_switch_pub_->publish(power_switch_msg);
      RCLCPP_INFO(
        node->get_logger(),
        "Power Switch: %u, %u",
        power_switch_msg.control_switch,
        power_switch_msg.power_switch);
    }
  }

private:
  void power_switch_callback(const PowerSwitch::SharedPtr msg)
  {
    const uint32_t can_id = (kOrangePiAddr << 7) | (1 << 1);
    uint8_t data[2];
    data[0] = static_cast<uint8_t>(msg->control_switch);
    data[1] = static_cast<uint8_t>(msg->power_switch);
    can_handler_.send(can_id, data, 2);
  }

  std::string can_device_;
  livelybot_can::CAN_Driver can_handler_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_volt_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_curr_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_temp_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr battery_level_pub_;
  rclcpp::Publisher<PowerSwitch>::SharedPtr power_switch_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr bms_error_pub_;
  rclcpp::Subscription<PowerSwitch>::SharedPtr power_switch_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
