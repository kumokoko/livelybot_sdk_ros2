#include "hardware/canboard.h"

#include <chrono>
#include <stdexcept>
#include <thread>

namespace
{
void sleep_for_seconds(double seconds)
{
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}
}  // namespace

canboard::canboard(
    const config &_config, const std::vector<lively_serial *> &serials_for_board,
    const rclcpp::Logger &logger)
{
    if (serials_for_board.size() != static_cast<size_t>(_config.canport_num))
    {
        throw std::runtime_error("CANboard serial mapping does not match configured CAN ports");
    }
    if (_config.ports.size() != static_cast<size_t>(_config.canport_num))
    {
        throw std::invalid_argument("CANboard port config count does not match CANport_num");
    }
    for (lively_serial *serial : serials_for_board)
    {
        if (serial == nullptr)
        {
            throw std::invalid_argument("CANboard serial mapping contains a null serial pointer");
        }
    }

    for (size_t j = 0; j < _config.ports.size(); ++j)
    {
        auto port = std::make_unique<canport>(_config.ports[j], serials_for_board[j], logger);
        can_ports_.push_back(port.get());
        can_port_storage_.push_back(std::move(port));
    }
}

canboard::~canboard()
{
    can_ports_.clear();
    can_port_storage_.clear();
}

canport &canboard::require_port(size_t port_index)
{
    if (port_index >= can_ports_.size())
    {
        throw std::out_of_range("CAN port index is outside the configured range");
    }
    return *can_ports_[port_index];
}

void canboard::append_can_ports_to(std::vector<canport *> &can_ports)
{
    for (canport *c : can_ports_)
    {
        can_ports.push_back(c);
    }
}

void canboard::send_motor_command_frame()
{
    for (canport *c : can_ports_)
    {
        c->send_motor_command_frame();
    }
}

void canboard::stop_motors()
{
    for (canport *c : can_ports_)
    {
        c->stop_motors();
    }
}

void canboard::reset_motors()
{
    for (canport *c : can_ports_)
    {
        c->reset_motors();
    }
}

float canboard::configure_port_motor_counts()
{
    float v = 0;
    for (canport *c : can_ports_)
    {
        v = c->configure_motor_count();
    }

    return v;
}

void canboard::request_motor_state()
{
    for (canport *c : can_ports_)
    {
        c->request_motor_state();
    }
}

void canboard::request_motor_state_with_mode()
{
    for (canport *c : can_ports_)
    {
        c->request_motor_state_with_mode();
    }
}

void canboard::request_motor_version()
{
    for (canport *c : can_ports_)
    {
        c->request_motor_version();
    }
}

void canboard::set_function_version(fun_version v)
{
    for (canport *c : can_ports_)
    {
        c->set_function_version(v);
    }
}

void canboard::reset_data()
{
    for (canport *c : can_ports_)
    {
        c->reset_data();
    }
}

void canboard::send_reset_pulses(canport &port, int repeat_count, double interval_seconds)
{
    for (int i = 0; i < repeat_count; ++i)
    {
        port.reset_motors();
        port.send_motor_command_frame();
        sleep_for_seconds(interval_seconds);
    }
}

void canboard::reset_zero_positions()
{
    for (canport *c : can_ports_)
    {
        send_reset_pulses(*c, 5, 0.1);
        sleep_for_seconds(1.0);
        if (c->reset_zero_positions() == 0)
        {
            c->save_configuration();
        }
        send_reset_pulses(*c, 1, 0.0);
        sleep_for_seconds(1.0);
        c->send_motor_command_frame();
        sleep_for_seconds(1.0);
    }
}

void canboard::enable_motor_runzero()
{
    for (canport *c : can_ports_)
    {
        c->enable_motor_runzero();
    }
}

void canboard::set_motor_timeout(int16_t t_ms)
{
    for (canport *c : can_ports_)
    {
        c->set_motor_timeout(t_ms);
    }
}

void canboard::set_motor_timeout(uint8_t portx, int16_t t_ms)
{
    require_port(portx).set_motor_timeout(t_ms);
}

void canboard::enter_canboard_bootloader()
{
    require_port(0).enter_canboard_bootloader();
}

void canboard::reset_canboard_fdcan()
{
    require_port(0).reset_canboard_fdcan();
}
