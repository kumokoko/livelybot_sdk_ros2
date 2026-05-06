#include <memory>

#include "livelybot_power/msg/power_switch.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class RobotEmergencyStopNode : public rclcpp::Node
{
public:
  RobotEmergencyStopNode()
  : rclcpp::Node(
      "robot_emergency_stop",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    lt_axis_(declare_parameter<int>("lt_axis", 2)),
    rt_axis_(declare_parameter<int>("rt_axis", 5)),
    home_button_(declare_parameter<int>("home_button", 8)),
    trigger_threshold_(declare_parameter<double>("trigger_threshold", -0.5))
  {
    power_pub_ = create_publisher<livelybot_power::msg::PowerSwitch>("power_switch_control", 10);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&RobotEmergencyStopNode::joy_callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(),
      "Emergency stop ready. Trigger combo: LT axis %d, RT axis %d, HOME button %d",
      lt_axis_, rt_axis_, home_button_);
  }

private:
  static bool axis_pressed(const sensor_msgs::msg::Joy & msg, int axis_index, double threshold)
  {
    if (axis_index < 0 || static_cast<size_t>(axis_index) >= msg.axes.size()) {
      return false;
    }
    return msg.axes[axis_index] < threshold;
  }

  static bool button_pressed(const sensor_msgs::msg::Joy & msg, int button_index)
  {
    if (button_index < 0 || static_cast<size_t>(button_index) >= msg.buttons.size()) {
      return false;
    }
    return msg.buttons[button_index] > 0;
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const bool lt_pressed = axis_pressed(*msg, lt_axis_, trigger_threshold_);
    const bool rt_pressed = axis_pressed(*msg, rt_axis_, trigger_threshold_);
    const bool home_pressed = button_pressed(*msg, home_button_);

    if (!(lt_pressed && rt_pressed && home_pressed)) {
      combo_active_ = false;
      return;
    }

    if (combo_active_) {
      return;
    }
    combo_active_ = true;

    livelybot_power::msg::PowerSwitch power_switch;
    power_switch.control_switch = 1;
    power_switch.power_switch = 0;
    power_pub_->publish(power_switch);
    RCLCPP_WARN(get_logger(), "Emergency power off triggered by controller combo");
  }

  int lt_axis_;
  int rt_axis_;
  int home_button_;
  double trigger_threshold_;
  bool combo_active_{false};
  rclcpp::Publisher<livelybot_power::msg::PowerSwitch>::SharedPtr power_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotEmergencyStopNode>());
  rclcpp::shutdown();
  return 0;
}
