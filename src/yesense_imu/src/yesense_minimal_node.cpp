#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "analysis_data.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "serial/serial.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "yesense_imu/msg/yesense_imu_all_data.hpp"

namespace
{
constexpr uint8_t MODE_HEADER1 = 0;
constexpr uint8_t MODE_HEADER2 = 1;
constexpr uint8_t MODE_TID_L = 2;
constexpr uint8_t MODE_TID_H = 3;
constexpr uint8_t MODE_LENGTH = 4;
constexpr uint8_t MODE_MESSAGE = 5;
constexpr uint8_t MODE_CHECKSUM_L = 6;
constexpr uint8_t MODE_CHECKSUM_H = 7;
constexpr size_t DATA_BUF_SIZE = 1024;
}  // namespace

class YesenseMinimalNode : public rclcpp::Node
{
public:
  YesenseMinimalNode()
  : rclcpp::Node(
      "yesense_imu",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    port_(declare_parameter<std::string>("yesense_port", "/dev/ttyS7")),
    baudrate_(declare_parameter<int>("yesense_baudrate", 460800)),
    frame_id_(declare_parameter<std::string>("frame_id", "imu_link")),
    linear_acceleration_stddev_(declare_parameter<double>("linear_acceleration_stddev", 0.0)),
    angular_velocity_stddev_(declare_parameter<double>("angular_velocity_stddev", 0.0)),
    orientation_stddev_(declare_parameter<double>("orientation_stddev", 0.0)),
    mode_(0),
    bytes_(0),
    index_(0),
    ck1_(0),
    ck2_(0)
  {
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 20);
    raw_pub_ = create_publisher<yesense_imu::msg::YesenseImuAllData>("yesense/all_data", 20);

    imu_msg_.header.frame_id = frame_id_;
    imu_msg_.linear_acceleration_covariance[0] = linear_acceleration_stddev_;
    imu_msg_.linear_acceleration_covariance[4] = linear_acceleration_stddev_;
    imu_msg_.linear_acceleration_covariance[8] = linear_acceleration_stddev_;
    imu_msg_.angular_velocity_covariance[0] = angular_velocity_stddev_;
    imu_msg_.angular_velocity_covariance[4] = angular_velocity_stddev_;
    imu_msg_.angular_velocity_covariance[8] = angular_velocity_stddev_;
    imu_msg_.orientation_covariance[0] = orientation_stddev_;
    imu_msg_.orientation_covariance[4] = orientation_stddev_;
    imu_msg_.orientation_covariance[8] = orientation_stddev_;

    serial_.setPort(port_);
    serial_.setBaudrate(static_cast<uint32_t>(baudrate_));
    auto timeout = serial::Timeout::simpleTimeout(1000);
    serial_.setTimeout(timeout);
    serial_.open();

    timer_ = create_wall_timer(std::chrono::milliseconds(2), std::bind(&YesenseMinimalNode::poll, this));
  }

  ~YesenseMinimalNode() override
  {
    if (serial_.isOpen()) {
      serial_.close();
    }
  }

private:
  void poll()
  {
    if (serial_.available() > 0) {
      const auto data = serial_.read(serial_.available());
      for (const auto ch : data) {
        buffer_.push_back(static_cast<uint8_t>(ch));
      }
    }

    while (!buffer_.empty()) {
      const uint8_t data = buffer_.front();
      buffer_.pop_front();
      parse_byte(data);
    }
  }

  void parse_byte(uint8_t data)
  {
    switch (mode_) {
      case MODE_HEADER1:
        if (data == 0x59) {
          mode_ = MODE_HEADER2;
        }
        break;
      case MODE_HEADER2:
        if (data == 0x53) {
          mode_ = MODE_TID_L;
          ck1_ = 0;
          ck2_ = 0;
          index_ = 0;
        } else {
          mode_ = MODE_HEADER1;
        }
        break;
      case MODE_TID_L:
        ck1_ += data;
        ck2_ += ck1_;
        mode_ = MODE_TID_H;
        break;
      case MODE_TID_H:
        ck1_ += data;
        ck2_ += ck1_;
        mode_ = MODE_LENGTH;
        break;
      case MODE_LENGTH:
        ck1_ += data;
        ck2_ += ck1_;
        bytes_ = data;
        index_ = 0;
        mode_ = bytes_ > 0 ? MODE_MESSAGE : MODE_CHECKSUM_L;
        break;
      case MODE_MESSAGE:
        ck1_ += data;
        ck2_ += ck1_;
        if (index_ < static_cast<int>(DATA_BUF_SIZE)) {
          message_in_[index_++] = data;
        }
        --bytes_;
        if (bytes_ == 0) {
          mode_ = MODE_CHECKSUM_L;
        }
        break;
      case MODE_CHECKSUM_L:
        if (ck1_ != data) {
          reset_parser();
          return;
        }
        mode_ = MODE_CHECKSUM_H;
        break;
      case MODE_CHECKSUM_H:
        if (ck2_ == data) {
          decode_message();
        }
        reset_parser();
        break;
      default:
        reset_parser();
        break;
    }
  }

  void decode_message()
  {
    unsigned short pos = 0;
    int payload_len = index_;
    while (payload_len > 0) {
      auto * payload = reinterpret_cast<payload_data_t *>(message_in_.data() + pos);
      if (pos >= DATA_BUF_SIZE) {
        break;
      }
      const auto ret = parse_data_by_id(
        payload->data_id,
        payload->data_len,
        reinterpret_cast<unsigned char *>(payload) + 2);
      if (ret == 0x01) {
        pos += payload->data_len + sizeof(payload_data_t);
        payload_len -= payload->data_len + sizeof(payload_data_t);
      } else {
        ++pos;
        --payload_len;
      }
    }
    publish_imu(g_output_info);
  }

  void publish_imu(const protocol_info_t & imu_data)
  {
    const auto stamp = now();
    imu_msg_.header.stamp = stamp;

    tf2::Quaternion q;
    q.setRPY(
      imu_data.roll / 180.0 * M_PI,
      imu_data.pitch / 180.0 * M_PI,
      imu_data.yaw / 180.0 * M_PI);
    imu_msg_.orientation = tf2::toMsg(q);
    imu_msg_.angular_velocity.x = imu_data.angle_x / 180.0 * M_PI;
    imu_msg_.angular_velocity.y = imu_data.angle_y / 180.0 * M_PI;
    imu_msg_.angular_velocity.z = imu_data.angle_z / 180.0 * M_PI;
    imu_msg_.linear_acceleration.x = imu_data.accel_x;
    imu_msg_.linear_acceleration.y = imu_data.accel_y;
    imu_msg_.linear_acceleration.z = imu_data.accel_z;
    imu_pub_->publish(imu_msg_);

    yesense_imu::msg::YesenseImuAllData raw_data;
    raw_data.temperature = imu_data.imu_temp;
    raw_data.sample_timestamp = imu_data.sample_timestamp;
    raw_data.sync_timestamp = imu_data.out_sync_timestamp;
    raw_data.accel.linear.x = imu_data.accel_x;
    raw_data.accel.linear.y = imu_data.accel_y;
    raw_data.accel.linear.z = imu_data.accel_z;
    raw_data.accel.angular.x = imu_data.angle_x;
    raw_data.accel.angular.y = imu_data.angle_y;
    raw_data.accel.angular.z = imu_data.angle_z;
    raw_data.euler_angle.roll = imu_data.roll;
    raw_data.euler_angle.pitch = imu_data.pitch;
    raw_data.euler_angle.yaw = imu_data.yaw;
    raw_data.quaternion.q0 = imu_data.quaternion_data0;
    raw_data.quaternion.q1 = imu_data.quaternion_data1;
    raw_data.quaternion.q2 = imu_data.quaternion_data2;
    raw_data.quaternion.q3 = imu_data.quaternion_data3;
    raw_data.location.longtidue = imu_data.longtidue;
    raw_data.location.latitude = imu_data.latitude;
    raw_data.location.altidue = imu_data.altidue;
    raw_data.status.fusion_status = imu_data.status & 0x0F;
    raw_data.status.gnss_status = (imu_data.status >> 4) & 0x0F;
    raw_pub_->publish(raw_data);
  }

  void reset_parser()
  {
    mode_ = MODE_HEADER1;
    bytes_ = 0;
    index_ = 0;
    ck1_ = 0;
    ck2_ = 0;
  }

  std::string port_;
  int baudrate_;
  std::string frame_id_;
  double linear_acceleration_stddev_;
  double angular_velocity_stddev_;
  double orientation_stddev_;
  serial::Serial serial_;
  std::deque<uint8_t> buffer_;
  std::array<uint8_t, DATA_BUF_SIZE> message_in_{};
  int mode_;
  int bytes_;
  int index_;
  uint8_t ck1_;
  uint8_t ck2_;
  sensor_msgs::msg::Imu imu_msg_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<yesense_imu::msg::YesenseImuAllData>::SharedPtr raw_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<YesenseMinimalNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
