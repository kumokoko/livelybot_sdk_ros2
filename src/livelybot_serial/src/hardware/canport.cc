#include "hardware/canport.h"

#include "rclcpp/logging.hpp"

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <unordered_set>

namespace
{
constexpr int kPortMotorNumMax = 30;
constexpr int kMotorIdMax = kCdcTrMessageDataLen / sizeof(int16_t);
constexpr uint8_t kAllMotorsPayload = 0x7f;

void sleep_for_seconds(double seconds)
{
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}
}  // namespace


canport::canport(
    const config &_config, lively_serial *serial, const rclcpp::Logger &logger)
: serial_(serial),
  logger_(logger)
{
    if (serial_ == nullptr)
    {
        throw std::invalid_argument("CAN port serial pointer must not be null");
    }

    canboard_id_ = _config.canboard_id;
    canport_id_ = _config.canport_id;
    motor_num_ = _config.motor_num;

    if (kPortMotorNumMax < motor_num_)
    {
        throw std::invalid_argument("Too many motors configured for CAN port");
    }
    if (motor_num_ != static_cast<int>(_config.motors.size()))
    {
        throw std::invalid_argument("CAN port motor_num does not match configured motors");
    }

    std::unordered_set<int> motor_ids;
    for (const auto &motor_config : _config.motors)
    {
        if (motor_config.id < 1 || motor_config.id > kMotorIdMax)
        {
            throw std::invalid_argument("Motor id is outside the serial command buffer range");
        }
        if (!motor_ids.insert(motor_config.id).second)
        {
            throw std::invalid_argument("Duplicate motor id configured for CAN port");
        }
        configured_motor_ids_.push_back(motor_config.id);
        if (max_motor_id_ < motor_config.id)
        {
            max_motor_id_ = motor_config.id;
        }
    }
    for (const auto &motor_config : _config.motors)
    {
        auto motor_instance = std::make_unique<motor>(
            motor_config, &tx_message_, max_motor_id_, logger_);
        motors_.push_back(motor_instance.get());
        motor_storage_.push_back(std::move(motor_instance));
    }
    for (motor *m : motors_)
    {
        motor_map_.emplace(m->motor_id(), m);
    }
    serial_->set_motor_map(motor_map_);
    serial_->set_port_version_target(&port_version_);
    serial_->set_motor_ack_targets(&acknowledged_motor_ids_, &acknowledged_mode_);
    serial_->set_function_version_target(&function_version_);
}

canport::~canport()
{
    motors_.clear();
    motor_storage_.clear();
    motor_map_.clear();
}

void canport::prepare_single_byte_command(uint8_t command)
{
    if (tx_message_.head.s.cmd != command)
    {
        tx_message_.head.s.head = 0XF7;
        tx_message_.head.s.cmd = command;
        tx_message_.head.s.len = 1;
        std::memset(&tx_message_.data, 0, tx_message_.head.s.len);
    }
}

void canport::set_single_byte_command_payload(uint8_t command, uint8_t value)
{
    prepare_single_byte_command(command);
    tx_message_.data.data[0] = value;
}

void canport::send_current_command(int repeat_count)
{
    for (int i = 0; i < repeat_count; ++i)
    {
        send_motor_command_frame();
    }
}

int canport::acknowledged_motor_count(uint8_t command) const
{
    if (acknowledged_mode_ != command)
    {
        return 0;
    }

    int count = 0;
    for (int id : configured_motor_ids_)
    {
        if (acknowledged_motor_ids_.count(id) == 1)
        {
            ++count;
        }
    }
    return count;
}

bool canport::wait_for_all_motor_ack(uint8_t command)
{
    return wait_for_ack_condition(
        [this, command]() {
            return acknowledged_motor_count(command) == motor_num_;
        });
}

bool canport::wait_for_ack_condition(const std::function<bool()> &acknowledged)
{
    constexpr int kMaxDelay = 10000;
    acknowledged_motor_ids_.clear();
    acknowledged_mode_ = 0;
    for (int t = 0; t < kMaxDelay; ++t)
    {
        send_motor_command_frame();
        sleep_for_seconds(0.02);
        if (acknowledged())
        {
            return true;
        }
    }
    return false;
}

bool canport::wait_for_motor_ack(uint8_t command, int id)
{
    return wait_for_ack_condition(
        [this, command, id]() {
            return acknowledged_mode_ == command && acknowledged_motor_ids_.count(id) == 1;
        });
}


float canport::configure_motor_count()
{
    set_single_byte_command_payload(MODE_SET_NUM, static_cast<uint8_t>(motor_num_));

    int t = 0;
    constexpr int kMaxDelay = 1000;  // 单位ms
    while (t++ < kMaxDelay)
    {
        send_motor_command_frame();
        sleep_for_seconds(0.02);
        if (port_version_ >= 2)
        {
            break;
        }
    }

    if (t < kMaxDelay)
    {
        RCLCPP_INFO(logger_, "\033[1;32mCANboard(%d) version is: v%.1f\033[0m", canboard_id_, port_version_);
    }
    else
    {
        RCLCPP_ERROR(logger_, "CANboard(%d) CANport(%d) Connection disconnected!!!", canboard_id_, canport_id_);
    }

    return port_version_;
}


int canport::reset_zero_positions()
{
    set_single_byte_command_payload(MODE_RESET_ZERO, kAllMotorsPayload);

    if (wait_for_all_motor_ack(MODE_RESET_ZERO))
    {
        RCLCPP_INFO(logger_, "\033[1;32mMotor zero position reset successfully, waiting for the motor to save the settings.\033[0m");
        return 0;
    }
    else
    {
        RCLCPP_ERROR(logger_, "Motor reset to zero position failed.");
        return 1;
    }
}


int canport::reset_zero_positions(int id)
{
    set_single_byte_command_payload(MODE_RESET_ZERO, static_cast<uint8_t>(id));

    return wait_for_motor_ack(MODE_RESET_ZERO, id) ? 0 : 1;
}


void canport::stop_motors()
{
    set_single_byte_command_payload(MODE_STOP, kAllMotorsPayload);
    send_current_command(1);
}


void canport::enable_motor_runzero()
{
    set_single_byte_command_payload(MODE_RUNZERO, kAllMotorsPayload);

    send_current_command(1);
}


void canport::reset_motors()
{
    set_single_byte_command_payload(MODE_RESET, kAllMotorsPayload);
    send_current_command(3);
}


void canport::save_configuration()
{
    set_single_byte_command_payload(MODE_CONF_WRITE, kAllMotorsPayload);

    if (wait_for_all_motor_ack(MODE_CONF_WRITE))
    {
        RCLCPP_INFO(logger_, "\033[1;32mSettings saved successfully.\033[0m");
    }
    else
    {
        throw std::runtime_error("Failed to save CAN port settings");
    }
}


int canport::save_configuration(int id)
{
    set_single_byte_command_payload(MODE_CONF_WRITE, static_cast<uint8_t>(id));

    return wait_for_motor_ack(MODE_CONF_WRITE, id) ? 0 : 1;
}


void canport::request_motor_state()
{
    set_single_byte_command_payload(MODE_MOTOR_STATE, kAllMotorsPayload);
    send_current_command(1);
}


void canport::request_motor_state_with_mode()
{
    set_single_byte_command_payload(MODE_MOTOR_STATE2, kAllMotorsPayload);
    send_current_command(1);
}


void canport::request_motor_version()
{
    set_single_byte_command_payload(MODE_MOTOR_VERSION, kAllMotorsPayload);
    send_current_command(1);
}


void canport::set_function_version(fun_version v)
{
    set_single_byte_command_payload(MODE_FUN_V, static_cast<uint8_t>(v));

    int t = 0;
    constexpr int kMaxDelay = 1000;  // 单位ms
    while (t++ < kMaxDelay)
    {
        send_motor_command_frame();
        sleep_for_seconds(0.02);
        if (v == function_version_)
        {
            break;
        }
    }

    if (t == kMaxDelay)
    {
        RCLCPP_ERROR(logger_, "CANboard(%d) CANport(%d) function version error!!!", canboard_id_, canport_id_);
    }
}


void canport::reset_data()
{
    std::memset(tx_message_.data.data, 0xFF, kCdcTrMessageDataLen);
}


void canport::set_motor_timeout(int16_t t_ms)
{
    if (tx_message_.head.s.cmd != MODE_TIME_OUT)
    {
        tx_message_.head.s.head = 0XF7;
        tx_message_.head.s.cmd = MODE_TIME_OUT;
        tx_message_.head.s.len = motor_num_ * 2;
        std::memset(&tx_message_.data, 0, tx_message_.head.s.len);
    }

    for (int i = 0; i < motor_num_; i++)
    {
        tx_message_.data.timeout[i] = t_ms;
    }

    send_motor_command_frame();
}


void canport::append_motors_to(std::vector<motor *> &motors)
{
    for (motor *m : motors_)
    {
        motors.push_back(m);
    }
}


void canport::send_motor_command_frame()
{
    serial_->send_frame(tx_message_);
}


void canport::enter_canboard_bootloader()
{
    prepare_single_byte_command(MODE_BOOTLOADER);
    send_current_command(3);
}


void canport::reset_canboard_fdcan()
{
    prepare_single_byte_command(MODE_FDCAN_RESET);
    send_current_command(3);
}
