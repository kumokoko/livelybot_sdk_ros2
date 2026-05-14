#ifndef LIVELYBOT_SERIAL_LIVELY_SERIAL_H
#define LIVELYBOT_SERIAL_LIVELY_SERIAL_H

#include "serial_struct.h"
#include "serial/serial.h"
#include "hardware/motor.h"

#include <atomic>
#include <cstddef>
#include <functional>
#include <map>
#include <unordered_set>

#include "rclcpp/logger.hpp"
#include "rclcpp/clock.hpp"

class lively_serial
{
private:
    void mark_error();
    bool read_exact(uint8_t *buffer, std::size_t length, const char *description);
    void handle_ack_message(uint8_t command, const cdc_tr_message_data_s &message_data, uint16_t length);
    void handle_port_version_message(const cdc_tr_message_data_s &message_data, uint16_t length);
    void handle_fun_version_message(const cdc_tr_message_data_s &message_data, uint16_t length);
    void handle_motor_version_message(const cdc_tr_message_data_s &message_data, uint16_t length);
    void handle_motor_state_message(const cdc_tr_message_data_s &message_data, uint16_t length);
    void handle_motor_state2_message(const cdc_tr_message_data_s &message_data, uint16_t length);
    void dispatch_frame(uint8_t command, const cdc_tr_message_data_s &message_data, uint16_t length);

    serial::Serial serial_port_;
    std::atomic_bool init_flag{false};
    std::map<int, motor *> motor_map_;

    float *port_version_target_ = nullptr;
    std::unordered_set<int> *motor_ack_ids_ = nullptr;
    int *motor_ack_mode_ = nullptr;
    fun_version *function_version_target_ = nullptr;
    std::function<double()> now_seconds_;
    rclcpp::Logger logger_;
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    std::atomic_bool error_flag{false};

public:
    lively_serial(
        const std::string &port, uint32_t baudrate, std::function<double()> now_seconds,
        const rclcpp::Logger &logger);
    ~lively_serial();

    void send_frame(cdc_tr_message_s &cdc_tr_message);
    void receive_loop();
    void set_port_version_target(float *port_version);
    void set_motor_ack_targets(std::unordered_set<int> *motor_ids, int *mode);
    void set_motor_map(const std::map<int, motor *> &motors);
    void set_function_version_target(fun_version *function_version);
    lively_serial(const lively_serial &) = delete;
    lively_serial &operator=(const lively_serial &) = delete;
    bool is_serial_error();
    void request_stop();
    void close();
};

#endif
