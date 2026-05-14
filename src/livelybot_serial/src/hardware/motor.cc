#include "hardware/motor.h"

#include "rclcpp/logging.hpp"

#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <unordered_map>

namespace
{
constexpr float kTwoPi = 6.28318530717f;
constexpr std::size_t kInt16PayloadCapacity = kCdcTrMessageDataLen / sizeof(int16_t);
constexpr std::size_t kBytePayloadCapacity = kCdcTrMessageDataLen / sizeof(uint8_t);
constexpr std::size_t kPosValTqePayloadCapacity =
    kCdcTrMessageDataLen / sizeof(motor_pos_val_tqe_s);
constexpr std::size_t kPosValAccPayloadCapacity =
    kCdcTrMessageDataLen / sizeof(motor_pos_val_acc_s);
constexpr std::size_t kPosValTqeRpdPayloadCapacity =
    kCdcTrMessageDataLen / sizeof(motor_pos_val_tqe_rpd_s);
constexpr std::size_t kPosValRpdPayloadCapacity =
    kCdcTrMessageDataLen / sizeof(motor_pos_val_rpd_s);
constexpr int kModePosition = 1;
constexpr int kModeVelocity = 2;
constexpr int kModeTorque = 3;
constexpr int kModeVoltage = 4;
constexpr int kModeCurrent = 5;
constexpr int kModePosVelMaxTorque = 6;
constexpr int kModeDeprecated7 = 7;
constexpr int kModeDeprecated8 = 8;
constexpr int kModePosVelTorqueKpKd = 9;
constexpr int kModePosVelKpKd = 10;
constexpr int kModePosVelAcc = 11;
constexpr int kModePosVelTorqueKpKd2 = 12;
constexpr int16_t kPositionNoCommand = -32768;

const std::unordered_map<std::string, motor_type> kMotorTypes =
{
    {"NULL", motor_type::null},
    {"3536_32", motor_type::m3536_32},
    {"4538_19", motor_type::m4538_19},
    {"5046_20", motor_type::m5046_20},
    {"5047_9", motor_type::m5047_09},
    {"5047_36", motor_type::m5047_36},
    {"4438_30", motor_type::m4438_30},
    {"4438_32", motor_type::m4438_32},
    {"6056_36", motor_type::m6056_36},
    {"5043_20", motor_type::m5043_20},
    {"7256_35", motor_type::m7256_35},
    {"60SG_35", motor_type::m60sg_35},
    {"60BM_35", motor_type::m60bm_35},
    {"5047_36_2", motor_type::m5047_36_2},
    {"General", motor_type::mGeneral},
};

const std::unordered_map<motor_type, float> kMotorTorqueAdjustments =
{
    {motor_type::m3536_32,   0.4581f},
    {motor_type::m5046_20,   0.5280f},
    {motor_type::m4538_19,   0.4450f},
    {motor_type::m5047_09,   0.5330f},
    {motor_type::m5047_36,   0.4938f},
    {motor_type::m5047_36_2, 0.8030f},
    {motor_type::m4438_30,   0.5256f},
    {motor_type::m4438_32,   0.5584f},
    {motor_type::m6056_36,   0.6770f},
    {motor_type::m7256_35,   0.6770f},
    {motor_type::m60sg_35,   0.7942f},
    {motor_type::m60bm_35,   0.7942f},
    {motor_type::m5043_20,   0.9660f},
    {motor_type::mGeneral,   0.5000f}
};

}  // namespace

motor::motor(
    const config &_config, cdc_tr_message_s *_p_cdc_tx_message, int _id_max,
    const rclcpp::Logger &logger)
: canport_num_(_config.canport_num),
  canboard_num_(_config.canboard_num),
  tx_message_(_p_cdc_tx_message),
  max_motor_id_(_id_max),
  logger_(logger)
{
    if (tx_message_ == nullptr)
    {
        throw std::invalid_argument("Motor command buffer pointer must not be null");
    }

    motor_name = _config.motor_name;
    id = _config.id;
    pos_limit_enable = _config.pos_limit_enable;
    pos_upper = _config.pos_upper;
    pos_lower = _config.pos_lower;
    tor_limit_enable = _config.tor_limit_enable;
    tor_upper = _config.tor_upper;
    tor_lower = _config.tor_lower;
    control_type = _config.control_type;

    try
    {
        RCLCPP_INFO(logger_, "Got params type: %s", _config.type_name.c_str());
        type_ = kMotorTypes.at(_config.type_name);
    }
    catch (const std::out_of_range &)
    {
        throw std::invalid_argument("Motor model error: " + _config.type_name);
    }
    data.time = 0;
    data.ID = id;
    data.mode = 0;
    data.fault = 0;
    data.position = 999.0f;
    data.velocity = 0;
    data.torque = 0;
}


inline int16_t motor::int16_limit(int32_t data)
{
    if (data >= 32700)
    {
        RCLCPP_INFO(logger_, "\033[1;32mPID output has reached the saturation limit.\033[0m");
        return static_cast<int16_t>(32700);
    }
    else if (data <= -32700)
    {
        RCLCPP_INFO(logger_, "\033[1;32mPID output has reached the saturation limit.\033[0m");
        return static_cast<int16_t>(-32700);
    }

    return static_cast<int16_t>(data);
}

std::size_t motor::payload_index(std::size_t capacity) const
{
    if (id <= 0)
    {
        throw std::out_of_range("Motor id must be positive");
    }

    const auto index = static_cast<std::size_t>(id - 1);
    if (index >= capacity)
    {
        throw std::out_of_range("Motor id exceeds command payload capacity");
    }
    return index;
}

void motor::require_payload_capacity(std::size_t capacity) const
{
    if (max_motor_id_ < 0 || static_cast<std::size_t>(max_motor_id_) > capacity)
    {
        throw std::out_of_range("Configured motor id range exceeds command payload capacity");
    }
}

bool motor::prepare_command(uint8_t command, uint16_t length)
{
    if (tx_message_->head.s.cmd == command)
    {
        return false;
    }

    tx_message_->head.s.head = 0xF7;
    tx_message_->head.s.cmd = command;
    tx_message_->head.s.len = length;
    return true;
}

void motor::prepare_position_command()
{
    require_payload_capacity(kInt16PayloadCapacity);
    if (prepare_command(MODE_POSITION, max_motor_id_ * sizeof(int16_t)))
    {
        for (std::size_t i = 0; i < static_cast<std::size_t>(max_motor_id_); i++)
        {
            tx_message_->data.position[i] = kPositionNoCommand;
        }
    }
}

void motor::prepare_int16_array_command(uint8_t command)
{
    require_payload_capacity(kInt16PayloadCapacity);
    if (prepare_command(command, max_motor_id_ * sizeof(int16_t)))
    {
        std::memset(tx_message_->data.data, 0, tx_message_->head.s.len);
    }
}

void motor::prepare_byte_array_command(uint8_t command)
{
    require_payload_capacity(kBytePayloadCapacity);
    if (prepare_command(command, max_motor_id_ * sizeof(uint8_t)))
    {
        std::memset(tx_message_->data.data, 0, tx_message_->head.s.len);
    }
}

void motor::prepare_pos_val_tqe_command(uint8_t command)
{
    require_payload_capacity(kPosValTqePayloadCapacity);
    if (prepare_command(command, max_motor_id_ * sizeof(motor_pos_val_tqe_s)))
    {
        for (std::size_t i = 0; i < static_cast<std::size_t>(max_motor_id_); i++)
        {
            tx_message_->data.pos_val_tqe[i].pos = kPositionNoCommand;
            tx_message_->data.pos_val_tqe[i].val = 0x0000;
            tx_message_->data.pos_val_tqe[i].tqe = 0x0000;
        }
    }
}

void motor::prepare_pos_val_acc_command(uint8_t command)
{
    require_payload_capacity(kPosValAccPayloadCapacity);
    if (prepare_command(command, max_motor_id_ * sizeof(motor_pos_val_acc_s)))
    {
        for (std::size_t i = 0; i < static_cast<std::size_t>(max_motor_id_); i++)
        {
            tx_message_->data.pos_val_acc[i].pos = kPositionNoCommand;
            tx_message_->data.pos_val_acc[i].val = 0x0000;
            tx_message_->data.pos_val_acc[i].acc = 0x0000;
        }
    }
}

void motor::prepare_pos_val_tqe_rpd_command(uint8_t command)
{
    require_payload_capacity(kPosValTqeRpdPayloadCapacity);
    if (prepare_command(command, max_motor_id_ * sizeof(motor_pos_val_tqe_rpd_s)))
    {
        for (std::size_t i = 0; i < static_cast<std::size_t>(max_motor_id_); i++)
        {
            tx_message_->data.pos_val_tqe_rpd[i].pos = kPositionNoCommand;
            tx_message_->data.pos_val_tqe_rpd[i].val = 0x0000;
            tx_message_->data.pos_val_tqe_rpd[i].tqe = 0x0000;
            tx_message_->data.pos_val_tqe_rpd[i].rkp = 0x0000;
            tx_message_->data.pos_val_tqe_rpd[i].rkd = 0x0000;
        }
    }
}

void motor::prepare_pos_val_rpd_command(uint8_t command)
{
    require_payload_capacity(kPosValRpdPayloadCapacity);
    if (prepare_command(command, max_motor_id_ * sizeof(motor_pos_val_rpd_s)))
    {
        for (std::size_t i = 0; i < static_cast<std::size_t>(max_motor_id_); i++)
        {
            tx_message_->data.pos_val_rpd[i].pos = kPositionNoCommand;
            tx_message_->data.pos_val_rpd[i].val = 0x0000;
            tx_message_->data.pos_val_rpd[i].rkp = 0x0000;
            tx_message_->data.pos_val_rpd[i].rkd = 0x0000;
        }
    }
}

void motor::pos_vel_tqe_kp_kd_command(
    uint8_t command, float position, float velocity, float torque, float kp, float kd)
{
    prepare_pos_val_tqe_rpd_command(command);
    const auto index = payload_index(kPosValTqeRpdPayloadCapacity);
    tx_message_->data.pos_val_tqe_rpd[index].pos = pos_float2int(position, position_velocity_type_);
    tx_message_->data.pos_val_tqe_rpd[index].val = vel_float2int(velocity, position_velocity_type_);
    tx_message_->data.pos_val_tqe_rpd[index].tqe = tqe_float2int(torque, type_);
    tx_message_->data.pos_val_tqe_rpd[index].rkp = kp_float2int(kp, position_velocity_type_, type_);
    tx_message_->data.pos_val_tqe_rpd[index].rkd = kd_float2int(kd, position_velocity_type_, type_);
}


inline int16_t motor::pos_float2int(float in_data, uint8_t type)
{
    switch (type)
    {
    case pos_vel_convert_type::radian_2pi:
        return int16_limit(in_data / kTwoPi * 10000.0);
    case pos_vel_convert_type::angle_360:
        return int16_limit(in_data / 360.0 * 10000.0);
    case pos_vel_convert_type::turns:
        return int16_limit(in_data * 10000.0);
    default:
        return 0;
    }
}

inline int16_t motor::vel_float2int(float in_data, uint8_t type)
{
    switch (type)
    {
    case pos_vel_convert_type::radian_2pi:
        return int16_limit(in_data / kTwoPi * 4000.0);
    case pos_vel_convert_type::angle_360:
        return int16_limit(in_data / 360.0 * 4000.0);
    case pos_vel_convert_type::turns:
        return int16_limit(in_data * 4000.0);
    default:
        return 0;
    }
}

inline int16_t motor::tqe_float2int(float in_data, motor_type motor_type)
{
    auto it = kMotorTorqueAdjustments.find(motor_type);
    if (it == kMotorTorqueAdjustments.end())
    {
        throw std::runtime_error("Motor torque conversion type setting error");
    }

    return int16_limit(in_data / (it->second * 0.01f));
}


inline float motor::tqe_int2float(int16_t in_data, motor_type motor_type)
{
    auto it = kMotorTorqueAdjustments.find(motor_type);
    if (it == kMotorTorqueAdjustments.end())
    {
        throw std::runtime_error("Motor torque conversion type setting error");
    }

    return in_data * (it->second * 0.01f);
}


inline float motor::pid_scale(float in_data, motor_type motor_type)
{
    auto it = kMotorTorqueAdjustments.find(motor_type);
    if (it == kMotorTorqueAdjustments.end())
    {
        throw std::runtime_error("Motor PID scale type setting error");
    }

    return in_data / it->second;
}


inline int16_t motor::pid_gain_float2int(float in_data, uint8_t type, motor_type motor_type)
{
    in_data = pid_scale(in_data, motor_type);
    
    int32_t tqe = 0;
    switch (type)
    {
    case pos_vel_convert_type::radian_2pi:
        tqe = static_cast<int32_t>(in_data * 10 * kTwoPi);
        break;
    case pos_vel_convert_type::angle_360:
        tqe = static_cast<int32_t>(in_data * 10 * 360);
        break;
    case pos_vel_convert_type::turns:
        tqe = static_cast<int32_t>(in_data * 10);
        break;
    default:
        tqe = int16_t();
        break;
    }
    return int16_limit(tqe);
}


inline int16_t motor::kp_float2int(float in_data, uint8_t type, motor_type motor_type)
{
    return pid_gain_float2int(in_data, type, motor_type);
}


inline int16_t motor::ki_float2int(float in_data, uint8_t type, motor_type motor_type)
{
    return pid_gain_float2int(in_data, type, motor_type);
}

inline int16_t motor::kd_float2int(float in_data, uint8_t type, motor_type motor_type)
{
    return pid_gain_float2int(in_data, type, motor_type);
}

inline float motor::pos_int2float(int16_t in_data, uint8_t type)
{
    switch (type)
    {
    case pos_vel_convert_type::radian_2pi:
        return static_cast<float>(in_data * kTwoPi / 10000.0);
    case pos_vel_convert_type::angle_360:
        return static_cast<float>(in_data * 360.0 / 10000.0);
    case pos_vel_convert_type::turns:
        return static_cast<float>(in_data / 10000.0);
    default:
        return 0.0f;
    }
}

inline float motor::vel_int2float(int16_t in_data, uint8_t type)
{
    switch (type)
    {
    case pos_vel_convert_type::radian_2pi:
        return static_cast<float>(in_data * kTwoPi / 4000.0);
    case pos_vel_convert_type::angle_360:
        return static_cast<float>(in_data * 360.0 / 4000.0);
    case pos_vel_convert_type::turns:
        return static_cast<float>(in_data / 4000.0);
    default:
        return 0.0f;
    }
}


void motor::fresh_cmd_int16(float position, float velocity, float torque, float kp, float, float kd, float acc, float voltage, float current)
{
    switch (control_type)
    {
    case kModePosition:
        motor::position(position);
        break;
    case kModeVelocity:
        motor::velocity(velocity);
        break;
    case kModeTorque:
        motor::torque(torque);
        break;
    case kModeVoltage:
        motor::voltage(voltage);
        break;
    case kModeCurrent:
        motor::current(current);
        break;
    case kModePosVelMaxTorque:
        motor::pos_vel_MAXtqe(position, velocity, torque);
        break;
    case kModeDeprecated7:
        throw std::runtime_error("Operation mode 7 is deprecated");
    case kModeDeprecated8:
        throw std::runtime_error("Operation mode 8 is deprecated");
    case kModePosVelTorqueKpKd:
        motor::pos_vel_tqe_kp_kd(position, velocity, torque, kp, kd);
        break;
    case kModePosVelKpKd:
        motor::pos_vel_kp_kd(position, velocity, kp, kd);
        break;
    case kModePosVelAcc:
        motor::pos_vel_acc(position, velocity, acc);
        break;
    case kModePosVelTorqueKpKd2:
        motor::pos_vel_tqe_kp_kd2(position, velocity, torque, kp, kd);
        break;
    default:
        throw std::invalid_argument("Incorrect setting of operation mode");
    }
}

void motor::position(float position)
{
    prepare_position_command();
    tx_message_->data.position[payload_index(kInt16PayloadCapacity)] =
        pos_float2int(position, position_velocity_type_);
}

void motor::velocity(float velocity)
{
    prepare_int16_array_command(MODE_VELOCITY);
    tx_message_->data.velocity[payload_index(kInt16PayloadCapacity)] =
        vel_float2int(velocity, position_velocity_type_);
}

void motor::torque(float torque)
{
    prepare_int16_array_command(MODE_TORQUE);
    tx_message_->data.torque[payload_index(kInt16PayloadCapacity)] =
        tqe_float2int(torque, type_);
}

void motor::voltage(float voltage)
{
    prepare_int16_array_command(MODE_VOLTAGE);
    tx_message_->data.voltage[payload_index(kInt16PayloadCapacity)] =
        int16_limit(voltage * 10);
}

void motor::current(float current)
{
    prepare_int16_array_command(MODE_CURRENT);
    tx_message_->data.current[payload_index(kInt16PayloadCapacity)] =
        int16_limit(current * 10);
}

void motor::set_motor_timeout(int16_t t_ms)
{
    prepare_int16_array_command(MODE_TIME_OUT);
    tx_message_->data.timeout[payload_index(kInt16PayloadCapacity)] = t_ms;
}

void motor::pos_vel_MAXtqe(float position, float velocity, float torque_max)
{
    prepare_pos_val_tqe_command(MODE_POS_VEL_TQE);
    const auto index = payload_index(kPosValTqePayloadCapacity);
    tx_message_->data.pos_val_tqe[index].pos = pos_float2int(position, position_velocity_type_);
    tx_message_->data.pos_val_tqe[index].val = vel_float2int(velocity, position_velocity_type_);
    tx_message_->data.pos_val_tqe[index].tqe = tqe_float2int(torque_max, type_);
}

void motor::pos_vel_acc(float position, float velocity, float acc)
{
    prepare_pos_val_acc_command(MODE_POS_VEL_ACC);
    const auto index = payload_index(kPosValAccPayloadCapacity);
    tx_message_->data.pos_val_acc[index].pos = pos_float2int(position, position_velocity_type_);
    tx_message_->data.pos_val_acc[index].val = vel_float2int(velocity, position_velocity_type_);
    tx_message_->data.pos_val_acc[index].acc = int16_limit(acc * 1000);
}

void motor::pos_vel_tqe_kp_kd(float position, float velocity, float torque, float kp, float kd)
{
    pos_vel_tqe_kp_kd_command(MODE_POS_VEL_TQE_KP_KD, position, velocity, torque, kp, kd);
}

void motor::pos_vel_tqe_kp_kd2(float position, float velocity, float torque, float kp, float kd)
{
    pos_vel_tqe_kp_kd_command(MODE_POS_VEL_TQE_KP_KD2, position, velocity, torque, kp, kd);
}

void motor::pos_vel_kp_kd(float position, float velocity, float kp, float kd)
{
    prepare_pos_val_rpd_command(MODE_POS_VEL_KP_KD);
    const auto index = payload_index(kPosValRpdPayloadCapacity);
    tx_message_->data.pos_val_rpd[index].pos = pos_float2int(position, position_velocity_type_);
    tx_message_->data.pos_val_rpd[index].val = vel_float2int(velocity, position_velocity_type_);
    tx_message_->data.pos_val_rpd[index].rkp = kp_float2int(kp, position_velocity_type_, type_);
    tx_message_->data.pos_val_rpd[index].rkd = kd_float2int(kd, position_velocity_type_, type_);
}


void motor::stop()
{
    prepare_byte_array_command(MODE_STOP);
    tx_message_->data.data[payload_index(kBytePayloadCapacity)] = 1;
}


void motor::brake()
{
    prepare_byte_array_command(MODE_BRAKE);
    tx_message_->data.data[payload_index(kBytePayloadCapacity)] = 1;
}


void motor::reset()
{
    prepare_byte_array_command(MODE_RESET);
    tx_message_->data.data[payload_index(kBytePayloadCapacity)] = 1;
}


/**
 * @brief 仅发送查询电机状态指令
 */
void motor::send_state_cmd()
{
    prepare_byte_array_command(MODE_MOTOR_STATE2);
    tx_message_->data.data[payload_index(kBytePayloadCapacity)] = 1;
}


void motor::fresh_data(
    uint8_t mode, uint8_t fault, int16_t position, int16_t velocity, int16_t torque,
    double receive_time)
{
    data.mode = mode;
    data.fault = fault;
    data.position = pos_int2float(position, position_velocity_type_);
    data.velocity = vel_int2float(velocity, position_velocity_type_);
    data.torque = tqe_int2float(torque, type_);
    data.time = receive_time;
    if (pos_limit_enable)
    {
        // 判断是否超过电机限制角度
        if (data.position > pos_upper)
        {
            RCLCPP_ERROR(logger_, "Motor %d exceed position upper limit.", id);
            position_limit_state_ = 1;
        }
        else if (data.position < pos_lower)
        {
            RCLCPP_ERROR(logger_, "Motor %d exceed position lower limit.", id);
            position_limit_state_ = -1;
        }
    }
    
    if (tor_limit_enable)
    {
        // 判断是否超过电机扭矩限制
        if (data.torque > tor_upper)
        {
            RCLCPP_ERROR(logger_, "Motor %d exceed torque upper limit.", id);
            torque_limit_state_ = 1;
        }
        else if (data.torque < tor_lower)
        {
            RCLCPP_ERROR(logger_, "Motor %d exceed torque lower limit.", id);
            torque_limit_state_ = -1;
        }
    }
    
}


/***
 * @brief setting motor type
 * @param type correspond to  different motor type 0~null 1~5046 2~5047_36减速比 3~5047_9减速比
 */
int motor::motor_id() const
{
    return id;
}


int motor::canport_id() const
{
    return canport_num_;
}


int motor::canboard_id() const
{
    return canboard_num_;
}

const motor_back_t &motor::state() const
{
    return data;
}

const std::string &motor::name() const
{
    return motor_name;
}

int motor::position_limit_state() const
{
    return position_limit_state_;
}

int motor::torque_limit_state() const
{
    return torque_limit_state_;
}


void motor::update_version(const cdc_rx_motor_version_s &v)
{
    version.id = v.id;
    version.major = v.major;
    version.minor = v.minor;
    version.patch = v.patch;
}


const cdc_rx_motor_version_s &motor::firmware_version() const
{
    return version;
}


void motor::set_motor_type(motor_type t)
{
    type_ = t;
}
