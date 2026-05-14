#ifndef LIVELYBOT_SERIAL_HARDWARE_CANPORT_H
#define LIVELYBOT_SERIAL_HARDWARE_CANPORT_H

#include "lively_serial.h"
#include "motor.h"

#include <functional>
#include <map>
#include <memory>
#include <unordered_set>
#include <vector>

#include "rclcpp/logger.hpp"

class canport
{
public:
    struct config
    {
        int motor_num = 0;
        int serial_id = 0;
        int canboard_id = 0;
        int canport_id = 0;
        std::vector<motor::config> motors;
    };

private:
    void prepare_single_byte_command(uint8_t command);
    void set_single_byte_command_payload(uint8_t command, uint8_t value);
    void send_current_command(int repeat_count);
    int acknowledged_motor_count(uint8_t command) const;
    bool wait_for_ack_condition(const std::function<bool()> &acknowledged);
    bool wait_for_all_motor_ack(uint8_t command);
    bool wait_for_motor_ack(uint8_t command, int id);

    int motor_num_ = 0;
    std::vector<std::unique_ptr<motor>> motor_storage_;
    std::vector<motor *> motors_;
    std::map<int, motor *> motor_map_;
    int canboard_id_ = 0;
    int canport_id_ = 0;
    lively_serial *serial_ = nullptr;
    cdc_tr_message_s tx_message_{};
    int max_motor_id_ = 0;
    float port_version_ = 0.0f;
    fun_version function_version_ = fun_v1;
    std::unordered_set<int> acknowledged_motor_ids_;
    int acknowledged_mode_ = 0;
    std::vector<int> configured_motor_ids_;
    rclcpp::Logger logger_;

public:
    canport(const config &_config, lively_serial *serial, const rclcpp::Logger &logger);
    ~canport();
    canport(const canport &) = delete;
    canport &operator=(const canport &) = delete;
    canport(canport &&) noexcept = default;
    canport &operator=(canport &&) noexcept = default;

    float configure_motor_count();
    int reset_zero_positions();
    int reset_zero_positions(int id);
    void stop_motors();
    void enable_motor_runzero();
    void reset_motors();
    void save_configuration();
    int save_configuration(int id);
    void request_motor_state();
    void request_motor_state_with_mode();
    void request_motor_version();
    void set_function_version(fun_version v);
    void reset_data();
    void set_motor_timeout(int16_t t_ms);
    void append_motors_to(std::vector<motor *> &motors);
    void send_motor_command_frame();
    void enter_canboard_bootloader();
    void reset_canboard_fdcan();
};

#endif
