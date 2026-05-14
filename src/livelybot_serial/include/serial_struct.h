#ifndef LIVELYBOT_SERIAL_SERIAL_STRUCT_H
#define LIVELYBOT_SERIAL_SERIAL_STRUCT_H
#include <cstddef>
#include <cstdint>
#include "crc/crc16.h"
#include "crc/crc8.h"

constexpr std::size_t kCdcTrMessageDataLen = 256;


constexpr uint8_t MODE_POSITION = 0X80;
constexpr uint8_t MODE_VELOCITY = 0X81;
constexpr uint8_t MODE_TORQUE = 0X82;
constexpr uint8_t MODE_VOLTAGE = 0X83;
constexpr uint8_t MODE_CURRENT = 0X84;
constexpr uint8_t MODE_TIME_OUT = 0x85;

constexpr uint8_t MODE_POS_VEL_TQE = 0X90;
constexpr uint8_t MODE_POS_VEL_TQE_KP_KD = 0X93;
constexpr uint8_t MODE_POS_VEL_KP_KD = 0X9E;
constexpr uint8_t MODE_POS_VEL_ACC = 0XAD;
constexpr uint8_t MODE_POS_VEL_TQE_KP_KD2 = 0XB0;


constexpr uint8_t MODE_RESET_ZERO = 0X01;     // 重置电机零位
constexpr uint8_t MODE_CONF_WRITE = 0X02;     // 保存设置
constexpr uint8_t MODE_STOP = 0X03;           // 电机停止
constexpr uint8_t MODE_BRAKE = 0X04;          // 电机刹车
constexpr uint8_t MODE_SET_NUM = 0X05;        // 设置通道电机数量，并查询固件版本
constexpr uint8_t MODE_MOTOR_STATE = 0X06;    // 电机状态
constexpr uint8_t MODE_RESET = 0X08;          // 电机重启
constexpr uint8_t MODE_RUNZERO = 0X09;        // 上电自动回零
constexpr uint8_t MODE_MOTOR_STATE2 = 0X0A;   // 电机状态2(带模式和错误码)
constexpr uint8_t MODE_MOTOR_VERSION = 0X0B;  // 电机版本号
constexpr uint8_t MODE_FUN_V = 0X0C;          // 设置功能版本号
constexpr uint8_t MODE_BOOTLOADER = 0X0D;     // 升级通讯板固件
constexpr uint8_t MODE_FDCAN_RESET = 0X0E;    // 重新初始化 FDCAN


constexpr uint16_t combine_version(uint16_t major, uint16_t minor, uint16_t patch)
{
    return static_cast<uint16_t>((major << 12) | (minor << 4) | patch);
}

enum fun_version
{
    fun_v1 = 1,  // v3
    fun_v2,      // v4 电机模式和错误码
    fun_v3,      // v4 5参数可选电机不响应
    fun_v4,      // v4 1701
    fun_v5,      // v4
    fun_v6,
    fun_v7,
    fun_v8,
    fun_v9,

    fun_vz = 99,
};

#pragma pack(1)
struct motor_pos_val_tqe_s
{
    int16_t pos;
    int16_t val;
    int16_t tqe;
};

struct motor_pos_val_tqe_rpd_s
{
    int16_t pos;
    int16_t val;
    int16_t tqe;
    int16_t rkp;
    int16_t rkd;
};

struct motor_pos_val_rpd_s
{
    int16_t pos;
    int16_t val;
    int16_t rkp;
    int16_t rkd;
};

struct motor_pos_val_acc_s
{
    int16_t pos;
    int16_t val;
    int16_t acc;
};

struct cdc_rx_motor_state_s
{
    uint8_t id;
    int16_t pos;
    int16_t val;
    int16_t tqe;
};

struct cdc_rx_motor_state2_s
{
    uint8_t id;
    uint8_t mode;
    uint8_t fault;
    int16_t pos;
    int16_t val;
    int16_t tqe;
};

struct cdc_rx_motor_version_s
{
    uint8_t id;
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
};

struct cdc_tr_message_head_data_s
{
    uint8_t head;
    uint8_t cmd;
    uint16_t len;
    uint8_t  crc8;
    uint16_t crc16;
};


struct cdc_tr_message_head_s
{
    union
    {
        cdc_tr_message_head_data_s s;
        uint8_t data[sizeof(cdc_tr_message_head_data_s)];
    };
};


struct cdc_tr_message_data_s
{
    union
    {
        int16_t position[kCdcTrMessageDataLen / sizeof(int16_t)];
        int16_t velocity[kCdcTrMessageDataLen / sizeof(int16_t)];
        int16_t torque[kCdcTrMessageDataLen / sizeof(int16_t)];
        int16_t voltage[kCdcTrMessageDataLen / sizeof(int16_t)];
        int16_t current[kCdcTrMessageDataLen / sizeof(int16_t)];
        int16_t timeout[kCdcTrMessageDataLen / sizeof(int16_t)];
        motor_pos_val_tqe_s pos_val_tqe[kCdcTrMessageDataLen / sizeof(motor_pos_val_tqe_s)];
        motor_pos_val_tqe_rpd_s pos_val_tqe_rpd[kCdcTrMessageDataLen / sizeof(motor_pos_val_tqe_rpd_s)];
        motor_pos_val_rpd_s pos_val_rpd[kCdcTrMessageDataLen / sizeof(motor_pos_val_rpd_s)];
        motor_pos_val_acc_s pos_val_acc[kCdcTrMessageDataLen / sizeof(motor_pos_val_acc_s)];
        cdc_rx_motor_state_s motor_state[kCdcTrMessageDataLen / sizeof(cdc_rx_motor_state_s)];
        cdc_rx_motor_state2_s motor_state2[kCdcTrMessageDataLen / sizeof(cdc_rx_motor_state2_s)];
        cdc_rx_motor_version_s motor_version[kCdcTrMessageDataLen / sizeof(cdc_rx_motor_version_s)];
        uint8_t data[kCdcTrMessageDataLen];
    };
};


struct cdc_tr_message_s
{
    cdc_tr_message_head_s head;
    cdc_tr_message_data_s data;
};


struct motor_back_t
{
    double time;
    uint8_t ID;
    uint8_t mode;
    uint8_t fault;
    float position;
    float velocity;
    float torque;
};

#pragma pack()


#endif
