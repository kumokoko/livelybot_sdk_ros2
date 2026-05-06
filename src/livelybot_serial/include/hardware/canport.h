#ifndef _CANPORT_H_
#define _CANPORT_H_

#include "../lively_serial.h"
#include "livelybot_serial/ros2_compat.hpp"
#include "motor.h"

#include <condition_variable>
#include <iostream>
#include <map>
#include <thread>
#include <unordered_set>
#include <vector>

#define PORT_MOTOR_NUM_MAX 30

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
    int motor_num = 0;
    std::vector<motor *> Motors;
    std::map<int, motor *> Map_Motors_p;
    int canboard_id = 0;
    int canport_id = 0;
    lively_serial *ser = nullptr;
    cdc_tr_message_s cdc_tr_message{};
    int id_max = 0;
    float port_version = 0.0f;
    fun_version fun_v = fun_v1;
    std::unordered_set<int> motors_id;
    int mode_flag = 0;
    std::vector<int> port_motor_id;
    std::vector<cdc_rx_motor_version_s *> motor_version;

public:
    canport(const config &_config, lively_serial *_ser);
    ~canport();

    float set_motor_num();
    int set_reset_zero();
    int set_reset_zero(int id);
    void set_stop();
    void set_motor_runzero();
    void set_reset();
    void set_conf_write();
    int set_conf_write(int id);
    void send_get_motor_state_cmd();
    void send_get_motor_state_cmd2();
    void send_get_motor_version_cmd();
    void set_fun_v(fun_version v);
    void set_data_reset();
    void set_time_out(int16_t t_ms);
    void puch_motor(std::vector<motor *> *_Motors);
    void motor_send_2();
    int get_motor_num();
    int get_canboard_id();
    int get_canport_id();
    void canboard_bootloader();
    void canboard_fdcan_reset();
};

#endif
