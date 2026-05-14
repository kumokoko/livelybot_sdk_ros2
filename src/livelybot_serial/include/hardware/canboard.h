#ifndef LIVELYBOT_SERIAL_HARDWARE_CANBOARD_H
#define LIVELYBOT_SERIAL_HARDWARE_CANBOARD_H

#include <memory>
#include <vector>

#include "canport.h"
#include "rclcpp/logger.hpp"

class canboard
{
public:
    struct config
    {
        int canboard_id = 0;
        int canport_num = 0;
        std::vector<canport::config> ports;
    };

private:
    canport &require_port(size_t port_index);
    void send_reset_pulses(canport &port, int repeat_count, double interval_seconds);

    std::vector<std::unique_ptr<canport>> can_port_storage_;
    std::vector<canport*> can_ports_;

public:
    canboard(
        const config &_config, const std::vector<lively_serial *> &serials_for_board,
        const rclcpp::Logger &logger);
    ~canboard();
    canboard(const canboard &) = delete;
    canboard &operator=(const canboard &) = delete;
    canboard(canboard &&) noexcept = default;
    canboard &operator=(canboard &&) noexcept = default;

    void append_can_ports_to(std::vector<canport *> &can_ports);
    void send_motor_command_frame();
    void stop_motors();
    void reset_motors();
    float configure_port_motor_counts();
    void request_motor_state();
    void request_motor_state_with_mode();
    void request_motor_version();
    void set_function_version(fun_version v);
    void reset_data();
    void reset_zero_positions();
    void enable_motor_runzero();
    void set_motor_timeout(int16_t t_ms);
    void set_motor_timeout(uint8_t portx, int16_t t_ms);
    void enter_canboard_bootloader();
    void reset_canboard_fdcan();
};
#endif
