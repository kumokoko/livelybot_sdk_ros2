#ifndef LIVELYBOT_SERIAL_HARDWARE_MOTOR_H
#define LIVELYBOT_SERIAL_HARDWARE_MOTOR_H

#include "serial_struct.h"

#include <cstddef>
#include <cstdint>

#include <string>

#include "rclcpp/logger.hpp"

enum motor_type
{
    null = 0,
    m3536_32,
    m4538_19,
    m5046_20,
    m5047_09,
    m5047_36,
    m4438_30,
    m4438_32,
    m6056_36,
    m5043_20,
    m7256_35,
    m60sg_35,
    m60bm_35,
    m5047_36_2,
    mGeneral,
};

enum pos_vel_convert_type
{
    radian_2pi = 0,
    angle_360,
    turns,
};

class motor
{
public:
    struct config
    {
        std::string motor_name;
        int id = 0;
        int num = 0;
        int canport_num = 0;
        int canboard_num = 0;
        std::string type_name;
        int control_type = 0;
        bool pos_limit_enable = false;
        float pos_upper = 0.0f;
        float pos_lower = 0.0f;
        bool tor_limit_enable = false;
        float tor_upper = 0.0f;
        float tor_lower = 0.0f;
    };

private:
    inline int16_t pos_float2int(float in_data, uint8_t type);
    inline int16_t vel_float2int(float in_data, uint8_t type);
    inline int16_t tqe_float2int(float in_data, motor_type motor_type);
    inline float pos_int2float(int16_t in_data, uint8_t type);
    inline float vel_int2float(int16_t in_data, uint8_t type);
    inline float tqe_int2float(int16_t in_data, motor_type type);
    inline float pid_scale(float in_data, motor_type motor_type);
    inline int16_t pid_gain_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t kp_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t ki_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t kd_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t int16_limit(int32_t data);
    std::size_t payload_index(std::size_t capacity) const;
    void require_payload_capacity(std::size_t capacity) const;
    bool prepare_command(uint8_t command, uint16_t length);
    void prepare_position_command();
    void prepare_int16_array_command(uint8_t command);
    void prepare_byte_array_command(uint8_t command);
    void prepare_pos_val_tqe_command(uint8_t command);
    void prepare_pos_val_acc_command(uint8_t command);
    void prepare_pos_val_tqe_rpd_command(uint8_t command);
    void prepare_pos_val_rpd_command(uint8_t command);
    void pos_vel_tqe_kp_kd_command(
        uint8_t command, float position, float velocity, float torque, float kp, float kd);

    int id = 0;
    int canport_num_ = 0;
    int canboard_num_ = 0;
    motor_back_t data{};
    std::string motor_name;
    motor_type type_ = motor_type::null;
    cdc_tr_message_s *tx_message_ = nullptr;
    int max_motor_id_ = 0;
    int control_type = 0;
    pos_vel_convert_type position_velocity_type_ = radian_2pi;
    bool pos_limit_enable = false;
    float pos_upper = 0.0f;
    float pos_lower = 0.0f;
    bool tor_limit_enable = false;
    float tor_upper = 0.0f;
    float tor_lower = 0.0f;
    int position_limit_state_ = 0;
    int torque_limit_state_ = 0;
    cdc_rx_motor_version_s version{};
    rclcpp::Logger logger_;

public:
    motor(
        const config &_config, cdc_tr_message_s *_p_cdc_tx_message, int _id_max,
        const rclcpp::Logger &logger);
    ~motor() = default;

    void fresh_cmd_int16(float position, float velocity, float torque, float kp, float ki, float kd, float acc, float voltage, float current);

    void position(float position);
    void velocity(float velocity);
    void torque(float torque);
    void voltage(float voltage);
    void current(float current);
    void set_motor_timeout(int16_t t_ms);
    void pos_vel_MAXtqe(float position, float velocity, float torque_max);
    void pos_vel_tqe_kp_kd(float position, float velocity, float torque, float Kp, float Kd);
    void pos_vel_tqe_kp_kd2(float position, float velocity, float torque, float kp, float kd);
    void pos_vel_kp_kd(float position, float velocity, float Kp, float Kd);
    void pos_vel_acc(float position, float velocity, float acc);

    void stop();
    void brake();
    void reset();
    void send_state_cmd();

    void fresh_data(
        uint8_t mode, uint8_t fault, int16_t position, int16_t velocity, int16_t torque,
        double receive_time);

    int motor_id() const;
    int canport_id() const;
    int canboard_id() const;
    const motor_back_t &state() const;
    const std::string &name() const;
    int position_limit_state() const;
    int torque_limit_state() const;
    const cdc_rx_motor_version_s &firmware_version() const;
    void update_version(const cdc_rx_motor_version_s &v);
    void set_motor_type(motor_type t);
};

#endif
