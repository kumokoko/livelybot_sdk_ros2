#include "lively_serial.h"

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstring>
#include <stdexcept>
#include <utility>


lively_serial::lively_serial(
    const std::string &port, uint32_t baudrate, std::function<double()> now_seconds,
    const rclcpp::Logger &logger)
: now_seconds_(std::move(now_seconds)),
  logger_(logger)
{
    if (!now_seconds_)
    {
        throw std::invalid_argument("Serial time provider must not be empty");
    }

    serial_port_.setPort(port); // 设置打开的串口名称
    serial_port_.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 创建timeout
    serial_port_.setTimeout(to);                                       // 设置串口的timeout

    // 打开串口
    try
    {
        serial_port_.open(); // 打开串口
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(logger_, "Motor unable to open port: %s", e.what());
        error_flag = true;
    }
    init_flag = serial_port_.isOpen();
}


lively_serial::~lively_serial()
{
    close();
}

void lively_serial::mark_error()
{
    error_flag = true;
    init_flag = false;
}

bool lively_serial::read_exact(uint8_t *buffer, std::size_t length, const char *description)
{
    if (length == 0)
    {
        return true;
    }

    const auto bytes_read = serial_port_.read(buffer, length);
    if (bytes_read == length)
    {
        return true;
    }
    if (!init_flag.load())
    {
        return false;
    }

    RCLCPP_ERROR(
        logger_, "Short serial read for %s: expected %zu bytes, got %zu",
        description, length, bytes_read);
    mark_error();
    return false;
}

void lively_serial::handle_ack_message(
    uint8_t command, const cdc_tr_message_data_s &message_data, uint16_t length)
{
    if (motor_ack_mode_ == nullptr || motor_ack_ids_ == nullptr)
    {
        return;
    }

    *motor_ack_mode_ = command;
    for (int i = 0; i < length; i++)
    {
        motor_ack_ids_->insert(message_data.data[i]);
    }
}

void lively_serial::handle_port_version_message(
    const cdc_tr_message_data_s &message_data, uint16_t length)
{
    if (length < 4)
    {
        RCLCPP_ERROR(logger_, "Invalid port version payload length: %u", length);
        return;
    }
    if (port_version_target_ == nullptr)
    {
        return;
    }

    *port_version_target_ = message_data.data[2];
    *port_version_target_ += static_cast<float>(message_data.data[3]) * 0.1f;
}

void lively_serial::handle_fun_version_message(
    const cdc_tr_message_data_s &message_data, uint16_t length)
{
    if (length < 1)
    {
        RCLCPP_ERROR(logger_, "Invalid function version payload length: %u", length);
        return;
    }
    if (function_version_target_ != nullptr)
    {
        *function_version_target_ = static_cast<fun_version>(message_data.data[0]);
    }
}

void lively_serial::handle_motor_version_message(
    const cdc_tr_message_data_s &message_data, uint16_t length)
{
    if (length % sizeof(cdc_rx_motor_version_s) != 0)
    {
        RCLCPP_ERROR(logger_, "Invalid motor version payload length: %u", length);
        return;
    }
    for (size_t i = 0; i < length / sizeof(cdc_rx_motor_version_s); i++)
    {
        auto it = motor_map_.find(message_data.motor_version[i].id);
        if (it != motor_map_.end())
        {
            it->second->update_version(message_data.motor_version[i]);
        }
    }
}

void lively_serial::handle_motor_state_message(
    const cdc_tr_message_data_s &message_data, uint16_t length)
{
    if (length % sizeof(cdc_rx_motor_state_s) != 0)
    {
        RCLCPP_ERROR(logger_, "Invalid motor state payload length: %u", length);
        return;
    }
    const double receive_time = now_seconds_();
    for (size_t i = 0; i < length / sizeof(cdc_rx_motor_state_s); i++)
    {
        auto it = motor_map_.find(message_data.motor_state[i].id);
        if (it != motor_map_.end())
        {
            it->second->fresh_data(
                0,
                0,
                message_data.motor_state[i].pos,
                message_data.motor_state[i].val,
                message_data.motor_state[i].tqe,
                receive_time);
        }
    }
}

void lively_serial::handle_motor_state2_message(
    const cdc_tr_message_data_s &message_data, uint16_t length)
{
    if (length % sizeof(cdc_rx_motor_state2_s) != 0)
    {
        RCLCPP_ERROR(logger_, "Invalid motor state2 payload length: %u", length);
        return;
    }
    const double receive_time = now_seconds_();
    for (size_t i = 0; i < length / sizeof(cdc_rx_motor_state2_s); i++)
    {
        auto it = motor_map_.find(message_data.motor_state2[i].id);
        if (it != motor_map_.end())
        {
            it->second->fresh_data(
                message_data.motor_state2[i].mode,
                message_data.motor_state2[i].fault,
                message_data.motor_state2[i].pos,
                message_data.motor_state2[i].val,
                message_data.motor_state2[i].tqe,
                receive_time);
        }
    }
}

void lively_serial::dispatch_frame(
    uint8_t command, const cdc_tr_message_data_s &message_data, uint16_t length)
{
    switch (command)
    {
    case MODE_RESET_ZERO:
    case MODE_CONF_WRITE:
        handle_ack_message(command, message_data, length);
        break;
    case MODE_SET_NUM:
        handle_port_version_message(message_data, length);
        break;
    case MODE_FUN_V:
        handle_fun_version_message(message_data, length);
        break;
    case MODE_MOTOR_VERSION:
        handle_motor_version_message(message_data, length);
        break;
    case MODE_MOTOR_STATE:
        handle_motor_state_message(message_data, length);
        break;
    case MODE_MOTOR_STATE2:
        handle_motor_state2_message(message_data, length);
        break;
    default:
        break;
    }
}

void lively_serial::receive_loop()
{
    uint16_t received_crc16 = 0;
    cdc_tr_message_data_s frame_data{};
    while (rclcpp::ok() && init_flag.load())
    {
        cdc_tr_message_head_data_s frame_header{};
        try
        {
            const auto head_bytes_read = serial_port_.read(&(frame_header.head), 1);
            if (head_bytes_read == 0)
            {
                continue;
            }
            if (head_bytes_read != 1)
            {
                if (init_flag.load())
                {
                    RCLCPP_ERROR(
                        logger_, "Short serial read for frame head: expected 1 byte, got %zu",
                        head_bytes_read);
                    mark_error();
                }
                break;
            }
            if (frame_header.head == 0xF7)
            {
                if (!read_exact(&(frame_header.cmd), 4, "frame header"))
                {
                    break;
                }
                if (frame_header.crc8 == Get_CRC8_Check_Sum(
                        reinterpret_cast<const uint8_t *>(&(frame_header.cmd)), 3, 0xFF))
                {
                    if (frame_header.len > sizeof(frame_data))
                    {
                        RCLCPP_ERROR(
                            logger_, "Serial frame payload is too large: %u bytes",
                            static_cast<unsigned int>(frame_header.len));
                        mark_error();
                        break;
                    }
                    if (!read_exact(reinterpret_cast<uint8_t *>(&received_crc16), 2, "frame crc16"))
                    {
                        break;
                    }
                    if (!read_exact(
                            reinterpret_cast<uint8_t *>(&frame_data), frame_header.len,
                            "frame payload"))
                    {
                        break;
                    }
                    if (received_crc16 != crc_ccitt(
                            0xFFFF, reinterpret_cast<const uint8_t *>(&frame_data),
                            frame_header.len))
                    {
                        RCLCPP_WARN_THROTTLE(
                            logger_, steady_clock_, 1000, "Serial frame CRC16 check failed");
                        std::memset(&frame_data, 0, sizeof(frame_data));
                    }
                    else
                    {
                        dispatch_frame(frame_header.cmd, frame_data, frame_header.len);
                    }
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(
                        logger_, steady_clock_, 1000, "Serial frame CRC8 check failed");
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger_, "Serial receive error: %s", e.what());
            close();
            mark_error();
            break;
        }
    }
}

bool lively_serial::is_serial_error()
{
    return error_flag.load();
}

void lively_serial::request_stop()
{
    init_flag = false;
}

void lively_serial::close()
{
    init_flag = false;
    try
    {
        if (serial_port_.isOpen())
        {
            serial_port_.flush();
        }
        serial_port_.close();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(logger_, "Serial close error: %s", e.what());
    }
}

void lively_serial::send_frame(cdc_tr_message_s &cdc_tr_message)
{
    if (cdc_tr_message.head.s.len > kCdcTrMessageDataLen)
    {
        RCLCPP_ERROR(
            logger_, "Serial transmit payload is too large: %u bytes",
            static_cast<unsigned int>(cdc_tr_message.head.s.len));
        mark_error();
        return;
    }

    cdc_tr_message.head.s.crc8 = Get_CRC8_Check_Sum(&(cdc_tr_message.head.data[1]), 3, 0xFF);
    cdc_tr_message.head.s.crc16 = crc_ccitt(0xFFFF, &(cdc_tr_message.data.data[0]), cdc_tr_message.head.s.len);

    try
    {
        if (serial_port_.isOpen())
        {
            serial_port_.write(
                reinterpret_cast<const uint8_t *>(&cdc_tr_message.head.s.head),
                cdc_tr_message.head.s.len + sizeof(cdc_tr_message_head_s));
        }
        else
        {
            RCLCPP_ERROR_THROTTLE(
                logger_, steady_clock_, 1000, "Cannot send serial frame because the port is closed");
            mark_error();
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(logger_, "Serial send error: %s", e.what());
        close();
        mark_error();
    }
}


void lively_serial::set_port_version_target(float *port_version)
{
    if (port_version == nullptr)
    {
        throw std::invalid_argument("Port version pointer must not be null");
    }
    port_version_target_ = port_version;
}


void lively_serial::set_motor_ack_targets(std::unordered_set<int> *motor_ids, int *mode)
{
    if (motor_ids == nullptr || mode == nullptr)
    {
        throw std::invalid_argument("Port motor ack pointers must not be null");
    }
    motor_ack_ids_ = motor_ids;
    motor_ack_mode_ = mode;
}


void lively_serial::set_motor_map(const std::map<int, motor *> &motors)
{
    motor_map_ = motors;
}


void lively_serial::set_function_version_target(fun_version *function_version)
{
    if (function_version == nullptr)
    {
        throw std::invalid_argument("Port function version pointer must not be null");
    }
    function_version_target_ = function_version;
}
