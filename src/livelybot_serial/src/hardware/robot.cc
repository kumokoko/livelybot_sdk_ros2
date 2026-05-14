#include "hardware/robot.h"

#include "rclcpp/logging.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <unordered_set>

#include <libserialport.h>


namespace livelybot_serial
{
    namespace
    {
        std::string parameter_key_from_legacy_path(const std::string &key)
        {
            std::string converted = key;
            std::replace(converted.begin(), converted.end(), '/', '.');
            if (!converted.empty() && converted.front() == '.')
            {
                converted.erase(converted.begin());
            }
            return converted;
        }

        template<typename ParamT>
        bool get_legacy_path_parameter(rclcpp::Node &node, const std::string &key, ParamT &value)
        {
            return node.get_parameter(parameter_key_from_legacy_path(key), value);
        }

        template<typename ParamT>
        void require_legacy_path_parameter(rclcpp::Node &node, const std::string &key, ParamT &value)
        {
            if (!get_legacy_path_parameter(node, key, value))
            {
                throw std::runtime_error("Failed to get parameter: " + key);
            }
        }

        template<typename ParamT>
        void require_parameter_alias(
            rclcpp::Node &node, const std::string &legacy_key,
            std::initializer_list<const char *> aliases, ParamT &value)
        {
            if (get_legacy_path_parameter(node, legacy_key, value))
            {
                return;
            }
            for (const char *alias : aliases)
            {
                if (node.get_parameter(alias, value))
                {
                    return;
                }
            }
            throw std::runtime_error("Failed to get parameter: " + legacy_key);
        }

        constexpr float kMaxImuLimitNum = 1.57f;
        constexpr int kMaxMotorTimeoutMs = 32760;
        constexpr double kJointStateStaleSeconds = 0.1;
        constexpr double kStaleJointPosition = -999.0;
        constexpr float kUninitializedMotorPosition = 999.0f;
        constexpr int kMaxConfiguredCanPorts = 7;
        constexpr const char *kSdkVersion = "4.4.6";

        bool is_valid_control_type(int control_type)
        {
            return control_type == 0 ||
                (control_type >= 1 && control_type <= 6) ||
                (control_type >= 9 && control_type <= 12);
        }

        struct MissingMotor
        {
            int board = 0;
            int port = 0;
            int id = 0;
        };

        struct MotorConnectionStatus
        {
            int connected_count = 0;
            std::vector<MissingMotor> missing_motors;
        };

        void sleep_for_seconds(double seconds);

        template<typename IsConnected>
        MotorConnectionStatus collect_motor_connection_status(
            const std::vector<motor *> &motors, IsConnected is_connected)
        {
            MotorConnectionStatus status;
            for (motor *m : motors)
            {
                if (is_connected(m))
                {
                    ++status.connected_count;
                    continue;
                }

                status.missing_motors.push_back(
                    MissingMotor{
                        m->canboard_id(),
                        m->canport_id(),
                        m->motor_id()});
            }
            return status;
        }

        bool report_motor_connection_status(
            const rclcpp::Logger &logger, const MotorConnectionStatus &status,
            std::size_t expected_motor_count, double failure_sleep_seconds)
        {
            if (status.connected_count == static_cast<int>(expected_motor_count))
            {
                RCLCPP_INFO(logger, "\033[1;32mAll motor connections are normal\033[0m");
                return true;
            }

            for (const MissingMotor &missing_motor : status.missing_motors)
            {
                RCLCPP_ERROR(
                    logger,
                    "CANboard(%d) CANport(%d) id(%d) Motor connection disconnected!!!",
                    missing_motor.board, missing_motor.port, missing_motor.id);
            }
            sleep_for_seconds(failure_sleep_seconds);
            return false;
        }

        bool wait_while_running(
            const std::atomic<bool> &running, const std::chrono::milliseconds duration)
        {
            constexpr auto kPollInterval = std::chrono::milliseconds(100);
            auto waited = std::chrono::milliseconds(0);
            while (running && waited < duration)
            {
                const auto remaining = duration - waited;
                const auto interval = remaining < kPollInterval ? remaining : kPollInterval;
                std::this_thread::sleep_for(interval);
                waited += interval;
            }
            return running;
        }

        void sleep_for_seconds(double seconds)
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
        }

        struct SerialPortDeleter
        {
            void operator()(sp_port *port) const
            {
                if (port != nullptr)
                {
                    sp_free_port(port);
                }
            }
        };

    }  // namespace

    motor::config robot::load_motor_config(
        rclcpp::Node &node, int cb_id, int cp_id, int motor_index, int control_type)
    {
        motor::config motor_config;
        motor_config.canboard_num = cb_id;
        motor_config.canport_num = cp_id;
        motor_config.control_type = control_type;

        const auto base_key =
            "robot/CANboard/No_" + std::to_string(cb_id) +
            "_CANboard/CANport/CANport_" + std::to_string(cp_id) +
            "/motor/motor" + std::to_string(motor_index);

        require_legacy_path_parameter(node, base_key + "/name", motor_config.motor_name);
        require_legacy_path_parameter(node, base_key + "/id", motor_config.id);
        require_legacy_path_parameter(node, base_key + "/type", motor_config.type_name);
        require_legacy_path_parameter(node, base_key + "/num", motor_config.num);
        require_legacy_path_parameter(node, base_key + "/pos_limit_enable", motor_config.pos_limit_enable);
        require_legacy_path_parameter(node, base_key + "/pos_upper", motor_config.pos_upper);
        require_legacy_path_parameter(node, base_key + "/pos_lower", motor_config.pos_lower);
        require_legacy_path_parameter(node, base_key + "/tor_limit_enable", motor_config.tor_limit_enable);
        require_legacy_path_parameter(node, base_key + "/tor_upper", motor_config.tor_upper);
        require_legacy_path_parameter(node, base_key + "/tor_lower", motor_config.tor_lower);

        return motor_config;
    }

    canport::config robot::load_port_config(
        rclcpp::Node &node, int cb_id, int cp_id, int control_type)
    {
        canport::config port_config;
        port_config.canboard_id = cb_id;
        port_config.canport_id = cp_id;

        const auto base_key =
            "robot/CANboard/No_" + std::to_string(cb_id) +
            "_CANboard/CANport/CANport_" + std::to_string(cp_id);
        require_legacy_path_parameter(node, base_key + "/motor_num", port_config.motor_num);
        require_legacy_path_parameter(node, base_key + "/serial_id", port_config.serial_id);

        for (int motor_index = 1; motor_index <= port_config.motor_num; ++motor_index)
        {
            port_config.motors.push_back(
                load_motor_config(node, cb_id, cp_id, motor_index, control_type));
        }

        return port_config;
    }

    canboard::config robot::load_board_config(rclcpp::Node &node, int cb_id, int control_type)
    {
        canboard::config board_config;
        board_config.canboard_id = cb_id;
        require_legacy_path_parameter(
            node,
            "robot/CANboard/No_" + std::to_string(cb_id) + "_CANboard/CANport_num",
            board_config.canport_num);

        for (int cp_id = 1; cp_id <= board_config.canport_num; ++cp_id)
        {
            board_config.ports.push_back(load_port_config(node, cb_id, cp_id, control_type));
        }

        return board_config;
    }

    robot::runtime_config robot::load_runtime_config(rclcpp::Node &node)
    {
        runtime_config config;
        require_parameter_alias(
            node, "robot/Seial_baudrate", {"robot.serial_baudrate"}, config.serial_baudrate);
        require_legacy_path_parameter(node, "robot/robot_name", config.robot_name);
        require_parameter_alias(
            node, "robot/CANboard_num", {"robot.canboard_num"}, config.canboard_num);
        require_parameter_alias(
            node, "robot/Serial_Type", {"robot.serial_type"}, config.serial_type);
        require_legacy_path_parameter(node, "robot/control_type", config.control_type);
        require_parameter_alias(
            node, "robot/imu_limt_flag", {"robot.imu_limit_flag"}, config.imu_limit_flag);
        require_legacy_path_parameter(node, "robot/imu_dir", config.imu_dir);
        require_parameter_alias(
            node, "robot/imu_limt_num", {"robot.imu_limit_num"}, config.imu_limit_num);
        require_legacy_path_parameter(node, "robot/motor_timeout_ms", config.motor_timeout_ms);

        for (int cb_id = 1; cb_id <= config.canboard_num; ++cb_id)
        {
            config.boards.push_back(load_board_config(node, cb_id, config.control_type));
        }
        validate_runtime_config(config);
        return config;
    }

    void robot::validate_runtime_config(const runtime_config &config)
    {
        if (config.robot_name.empty())
        {
            throw std::invalid_argument("robot_name must not be empty");
        }
        if (config.serial_type.empty())
        {
            throw std::invalid_argument("Serial_Type must not be empty");
        }
        if (config.serial_baudrate <= 0)
        {
            throw std::invalid_argument("Seial_baudrate must be positive");
        }
        if (config.canboard_num < 1)
        {
            throw std::invalid_argument("CANboard_num must be positive");
        }
        if (!is_valid_control_type(config.control_type))
        {
            throw std::invalid_argument("control_type must be 0, 1-6, or 9-12");
        }
        if (config.imu_limit_num < 0.0f || config.imu_limit_num > kMaxImuLimitNum)
        {
            throw std::invalid_argument("The value of imu_limt_num must be in the range [0, 1.57]");
        }
        if (config.motor_timeout_ms < 0 || config.motor_timeout_ms > kMaxMotorTimeoutMs)
        {
            throw std::invalid_argument(
                "The value of motor_timeout_ms is out of the valid range [0, 32760]");
        }
        if (config.canboard_num != static_cast<int>(config.boards.size()))
        {
            throw std::invalid_argument("CANboard_num does not match the number of configured CANboards");
        }
        int configured_can_ports = 0;
        for (size_t board_index = 0; board_index < config.boards.size(); ++board_index)
        {
            const auto &board_config = config.boards[board_index];
            if (board_config.canboard_id < 1)
            {
                throw std::invalid_argument("CANboard id must be positive");
            }
            if (board_config.canboard_id != static_cast<int>(board_index + 1))
            {
                throw std::invalid_argument("CANboard id does not match its configured index");
            }
            if (board_config.canport_num != static_cast<int>(board_config.ports.size()))
            {
                throw std::invalid_argument("CANport_num does not match the number of configured CANports");
            }
            configured_can_ports += board_config.canport_num;
            for (size_t port_index = 0; port_index < board_config.ports.size(); ++port_index)
            {
                const auto &port_config = board_config.ports[port_index];
                if (port_config.canport_id < 1)
                {
                    throw std::invalid_argument("CANport id must be positive");
                }
                if (port_config.motor_num < 1)
                {
                    throw std::invalid_argument("motor_num must be positive");
                }
                if (port_config.canboard_id != board_config.canboard_id)
                {
                    throw std::invalid_argument("CANport canboard id does not match its parent CANboard");
                }
                if (port_config.canport_id != static_cast<int>(port_index + 1))
                {
                    throw std::invalid_argument("CANport id does not match its configured index");
                }
                if (port_config.serial_id < 1)
                {
                    throw std::invalid_argument("serial_id must be positive");
                }
                if (port_config.motor_num != static_cast<int>(port_config.motors.size()))
                {
                    throw std::invalid_argument("motor_num does not match the number of configured motors");
                }
                std::unordered_set<int> motor_ids;
                for (size_t motor_index = 0; motor_index < port_config.motors.size(); ++motor_index)
                {
                    const auto &motor_config = port_config.motors[motor_index];
                    if (motor_config.id < 1)
                    {
                        throw std::invalid_argument("motor id must be positive");
                    }
                    if (motor_config.type_name.empty())
                    {
                        throw std::invalid_argument("motor type must not be empty");
                    }
                    if (motor_config.canboard_num != board_config.canboard_id)
                    {
                        throw std::invalid_argument("motor CANboard id does not match its parent CANboard");
                    }
                    if (motor_config.canport_num != port_config.canport_id)
                    {
                        throw std::invalid_argument("motor CANport id does not match its parent CANport");
                    }
                    if (motor_config.num != static_cast<int>(motor_index + 1))
                    {
                        throw std::invalid_argument("motor num does not match its configured index");
                    }
                    if (!motor_ids.insert(motor_config.id).second)
                    {
                        throw std::invalid_argument("duplicate motor id within a CANport");
                    }
                    if (motor_config.pos_limit_enable && motor_config.pos_upper < motor_config.pos_lower)
                    {
                        throw std::invalid_argument("motor position upper limit must be greater than or equal to lower limit");
                    }
                    if (motor_config.tor_limit_enable && motor_config.tor_upper < motor_config.tor_lower)
                    {
                        throw std::invalid_argument("motor torque upper limit must be greater than or equal to lower limit");
                    }
                }
            }
        }
        if (configured_can_ports > kMaxConfiguredCanPorts)
        {
            throw std::invalid_argument("Configured CAN ports exceed the supported maximum of 7");
        }
    }

    std::vector<lively_serial *> robot::collect_board_serials(
        const canboard::config &board_config, size_t &serial_offset)
    {
        std::vector<lively_serial *> board_serials;
        board_serials.reserve(board_config.ports.size());
        for (size_t port_index = 0; port_index < board_config.ports.size(); ++port_index)
        {
            if (serial_offset >= serials_.size())
            {
                throw std::runtime_error("serial_id count does not match configured CAN ports");
            }
            board_serials.push_back(serials_[serial_offset++]);
        }
        return board_serials;
    }

    void robot::build_runtime_topology()
    {
        size_t serial_offset = 0;
        for (const auto &board_config : config_.boards)
        {
            can_boards_.emplace_back(board_config, collect_board_serials(board_config, serial_offset), logger_);
        }

        can_ports_.clear();
        motors_.clear();
        for (canboard &cb : can_boards_)
        {
            cb.append_can_ports_to(can_ports_);
        }
        for (canport *cp : can_ports_)
        {
            cp->append_motors_to(motors_);
        }
        validate_runtime_topology();
    }

    void robot::validate_runtime_topology() const
    {
        if (can_boards_.size() != config_.boards.size())
        {
            throw std::runtime_error("Runtime CANboard topology does not match configuration");
        }
        if (static_cast<int>(can_ports_.size()) != expected_serial_device_count())
        {
            throw std::runtime_error("Runtime CANport topology does not match configuration");
        }

        std::size_t expected_motors = 0;
        for (const auto &board_config : config_.boards)
        {
            for (const auto &port_config : board_config.ports)
            {
                expected_motors += port_config.motors.size();
            }
        }
        if (motors_.size() != expected_motors)
        {
            throw std::runtime_error("Runtime motor topology does not match configuration");
        }
    }

    void robot::clear_runtime_topology()
    {
        motors_.clear();
        can_ports_.clear();
        can_boards_.clear();
    }

    void robot::stop_serial_receivers()
    {
        for (lively_serial *s : serials_)
        {
            s->request_stop();
        }
        for (auto &thread : serial_receive_threads_)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }
        serial_receive_threads_.clear();
    }

    void robot::destroy_serial_devices()
    {
        serials_.clear();
        serial_devices_.clear();
    }

    void robot::start_error_check_thread()
    {
        error_check_running_ = true;
        error_check_thread_ = std::thread(&robot::check_error, this);
    }

    void robot::stop_error_check_thread()
    {
        error_check_running_ = false;
        if (error_check_thread_.joinable())
        {
            error_check_thread_.join();
        }
    }

    void robot::initialize_hardware()
    {
        initialize_serial_devices();

        build_runtime_topology();
        configure_port_motor_counts(); // 设置通道上挂载的电机数，并获取主控板固件版本号
        if (slave_version_ >= 4.1f)
        {
            reset_canboard_fdcan_unlocked();
        }

        check_motor_connection_for_firmware();

        if (config_.motor_timeout_ms != 0)
        {
            set_motor_timeout_unlocked(config_.motor_timeout_ms);
        }

        request_motor_state();

        RCLCPP_INFO(logger_, "\033[1;32mThe robot has %zu motors\033[0m", motors_.size());
        RCLCPP_INFO(logger_, "robot init");
    }

    void robot::cleanup_runtime()
    {
        stop_serial_receivers();
        clear_runtime_topology();
        destroy_serial_devices();
    }

    void robot::reset_all_boards(int repeat_count)
    {
        for (int i = 0; i < repeat_count; ++i)
        {
            reset_motors_unlocked();
        }
    }

    void robot::repeat_for_all_boards(
        int repeat_count, double interval_seconds, void (canboard::*operation)())
    {
        for (int i = 0; i < repeat_count; ++i)
        {
            for_each_board(operation);
            sleep_for_seconds(interval_seconds);
        }
    }

    void robot::for_each_board(void (canboard::*operation)())
    {
        for (canboard &cb : can_boards_)
        {
            (cb.*operation)();
        }
    }

    void robot::repeat_timeout_for_all_boards(
        int repeat_count, double interval_seconds, int16_t t_ms)
    {
        for (int i = 0; i < repeat_count; ++i)
        {
            for (canboard &cb : can_boards_)
            {
                cb.set_motor_timeout(t_ms);
            }
            sleep_for_seconds(interval_seconds);
        }
    }

    void robot::repeat_timeout_for_port(
        int repeat_count, double interval_seconds, uint8_t portx, int16_t t_ms)
    {
        for (int i = 0; i < repeat_count; ++i)
        {
            require_board(0).set_motor_timeout(portx, t_ms);
            sleep_for_seconds(interval_seconds);
        }
    }

    void robot::validate_timeout_ms(int16_t t_ms) const
    {
        if (t_ms < 0 || t_ms > kMaxMotorTimeoutMs)
        {
            throw std::invalid_argument("timeout is out of the valid range [0, 32760]");
        }
    }

    int robot::expected_serial_device_count() const
    {
        int count = 0;
        for (const auto &board_config : config_.boards)
        {
            count += static_cast<int>(board_config.ports.size());
        }
        return count;
    }

    robot::robot(
        const runtime_config &_config, std::function<double()> now_seconds,
        const rclcpp::Logger &logger)
    : config_(_config),
      now_seconds_(std::move(now_seconds)),
      logger_(logger)
    {
        validate_runtime_config(config_);
        if (!now_seconds_)
        {
            throw std::invalid_argument("Robot time provider must not be empty");
        }

        if (config_.imu_limit_flag)
        {
            RCLCPP_INFO(logger_, "IMU needs to be enabled");
        }

        RCLCPP_INFO(logger_, "\033[1;32mGot params SDK_version: v%s\033[0m", kSdkVersion);
        RCLCPP_INFO(logger_, "\033[1;32mThe robot name is %s\033[0m", config_.robot_name.c_str());
        RCLCPP_INFO(logger_, "\033[1;32mThe robot has %d CANboards\033[0m", config_.canboard_num);
        RCLCPP_INFO(logger_, "\033[1;32mThe Serial type is %s\033[0m", config_.serial_type.c_str());
        try
        {
            initialize_hardware();
            start_error_check_thread();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger_, "robot initialization failed: %s", e.what());
            stop_error_check_thread();
            cleanup_runtime();
            throw;
        }
        catch (...)
        {
            RCLCPP_ERROR(logger_, "robot initialization failed with an unknown exception");
            stop_error_check_thread();
            cleanup_runtime();
            throw;
        }
    }
    robot::~robot()
    {
        try
        {
            stop_error_check_thread();
            std::lock_guard<std::mutex> lock(runtime_mutex_);
            reset_all_boards(3);
            cleanup_runtime();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger_, "robot cleanup failed: %s", e.what());
        }
    }

    void robot::update_imu_state(const sensor_msgs::msg::Imu &msg)
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        const double w = msg.orientation.w;
        const double x = msg.orientation.x;
        const double y = msg.orientation.y;
        const double z = msg.orientation.z;

        const double sinr_cosp = 2 * (w * x + y * z);
        const double cosr_cosp = 1 - 2 * (x * x + y * y);
        roll_ = std::atan2(sinr_cosp, cosr_cosp);

        const double sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
        {
            pitch_ = std::copysign(M_PI / 2, sinp);
        }
        else
        {
            pitch_ = std::asin(sinp);
        }
    }

    void robot::apply_motor_commands(const std::vector<motor_command> &commands)
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        if (commands.size() < motors_.size())
        {
            throw std::invalid_argument("Motor command count is smaller than configured motor count");
        }

        for (std::size_t motor_index = 0; motor_index < motors_.size(); ++motor_index)
        {
            const motor_command &command = commands[motor_index];
            motors_[motor_index]->fresh_cmd_int16(
                command.position,
                command.velocity,
                command.torque,
                command.kp,
                0.0f,
                command.kd,
                0.0f,
                0.0f,
                0.0f);
        }
    }

    void robot::append_motor_state(
        sensor_msgs::msg::JointState &joint_state_msg, motor *m, double now_seconds) const
    {
        joint_state_msg.name.push_back(m->name());
        const motor_back_t &data = m->state();
        if (now_seconds - data.time > kJointStateStaleSeconds)
        {
            joint_state_msg.position.push_back(kStaleJointPosition);
            joint_state_msg.velocity.push_back(0);
            joint_state_msg.effort.push_back(0);
        }
        else
        {
            joint_state_msg.position.push_back(data.position);
            joint_state_msg.velocity.push_back(data.velocity);
            joint_state_msg.effort.push_back(data.torque);
        }
    }

    sensor_msgs::msg::JointState robot::build_joint_state_message(
        const builtin_interfaces::msg::Time &stamp, double now_seconds) const
    {
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = stamp;

        for (motor *m : motors_)
        {
            append_motor_state(joint_state_msg, m, now_seconds);
        }

        return joint_state_msg;
    }

    sensor_msgs::msg::JointState robot::run_control_cycle(
        const builtin_interfaces::msg::Time &stamp, double now_seconds)
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        detect_motor_limit();
        send_motor_command_frame();
        return build_joint_state_message(stamp, now_seconds);
    }

    void robot::detect_motor_limit()
    {
        // 电机正常运行时检测是否超过限位，停机之后不检测
        if (!motor_position_limit_state_ && !motor_torque_limit_state_)
        {
            for (motor *m : motors_)
            {
                const int position_limit_state = m->position_limit_state();
                if (position_limit_state != 0)
                {
                    RCLCPP_ERROR(logger_, "robot pos limit, motor stop.");
                    stop_motors_unlocked();
                    motor_position_limit_state_ = position_limit_state;
                    break;
                }

                const int torque_limit_state = m->torque_limit_state();
                if (torque_limit_state != 0)
                {
                    RCLCPP_ERROR(logger_, "robot torque limit, motor stop.");
                    stop_motors_unlocked();
                    motor_torque_limit_state_ = torque_limit_state;
                    break;
                }
            }
        }
    }


    bool robot::imu_limit_ok()
    {
        float roll_err = 0.0f;

        if (!config_.imu_limit_flag)
        {
            return true;
        }

        if (config_.imu_dir)
        {
            roll_err = std::abs(roll_);
        }
        else
        {
            if (roll_ < 0)
            {
                roll_err = roll_ + static_cast<float>(M_PI);
            }
            else
            {
                roll_err = static_cast<float>(M_PI) - roll_;
            }
        }

        if (roll_err > config_.imu_limit_num || pitch_ > config_.imu_limit_num)
        {
            return false;
        }

        return true;
    }

    motor &robot::require_motor(size_t motor_index)
    {
        if (motor_index >= motors_.size())
        {
            throw std::out_of_range("Motor index is outside the configured range");
        }
        return *motors_[motor_index];
    }

    canport &robot::require_port(size_t port_index)
    {
        if (port_index >= can_ports_.size())
        {
            throw std::out_of_range("CAN port index is outside the configured range");
        }
        return *can_ports_[port_index];
    }

    canboard &robot::require_board(size_t board_index)
    {
        if (board_index >= can_boards_.size())
        {
            throw std::out_of_range("CAN board index is outside the configured range");
        }
        return can_boards_[board_index];
    }

    canport &robot::require_motor_port(const motor &selected_motor)
    {
        const int board_id = selected_motor.canboard_id();
        const int port_id = selected_motor.canport_id();
        if (board_id < 1 || port_id < 1)
        {
            throw std::out_of_range("Motor belongs to an invalid CAN board or port");
        }

        std::size_t flat_port_index = 0;
        for (int board_index = 1; board_index < board_id; ++board_index)
        {
            const auto config_index = static_cast<std::size_t>(board_index - 1);
            if (config_index >= config_.boards.size())
            {
                throw std::out_of_range("Motor CAN board is outside the configured range");
            }
            flat_port_index += config_.boards[config_index].ports.size();
        }
        flat_port_index += static_cast<std::size_t>(port_id - 1);
        return require_port(flat_port_index);
    }


    void robot::send_motor_command_frame()
    {
        if (!imu_limit_ok())
        {
            for (motor *m : motors_)
            {
                m->pos_vel_tqe_kp_kd(m->state().position, 0, 0, 10, 1);
            }
        }

        if (!motor_position_limit_state_ && !motor_torque_limit_state_)
        {
            for (canboard &cb : can_boards_)
            {
                cb.send_motor_command_frame();
            }
        }

    }


    bool robot::is_supported_serial_port(const std::string &name)
    {
        int pid = 0;
        int vid = 0;
        sp_port *raw_port = nullptr;
        if (sp_get_port_by_name(name.c_str(), &raw_port) != SP_OK || raw_port == nullptr)
        {
            return false;
        }

        std::unique_ptr<sp_port, SerialPortDeleter> port(raw_port);
        if (sp_get_port_usb_vid_pid(port.get(), &vid, &pid) != SP_OK)
        {
            return false;
        }

        if (pid != 0xFFFF)
        {
            return false;
        }

        switch (vid)
        {
        case 0xCAF1:
        case 0xCAE1:
            return true;
        default:
            return false;
        }
    }


    std::vector<std::string> robot::list_serial_ports(const std::string &full_prefix)
    {
        const auto separator = full_prefix.rfind('/');
        const std::string base_path = separator == std::string::npos
            ? std::string("./")
            : full_prefix.substr(0, separator + 1);
        const std::string prefix = separator == std::string::npos
            ? full_prefix
            : full_prefix.substr(separator + 1);
        std::vector<std::string> serial_ports;
        const std::filesystem::path directory_path(base_path);
        std::error_code error;
        if (!std::filesystem::is_directory(directory_path, error))
        {
            RCLCPP_ERROR(logger_, "Could not open the directory %s", base_path.c_str());
            return serial_ports;
        }

        for (const auto &entry : std::filesystem::directory_iterator(directory_path, error))
        {
            if (error)
            {
                RCLCPP_ERROR(
                    logger_, "Could not read the directory %s: %s",
                    base_path.c_str(), error.message().c_str());
                return serial_ports;
            }
            const std::string entry_name = entry.path().filename().string();
            if (entry_name.find(prefix) == 0)
            {
                serial_ports.push_back((directory_path / entry_name).string());
            }
        }

        std::reverse(serial_ports.begin(), serial_ports.end());

        return serial_ports;
    }

    std::vector<std::string> robot::discover_matching_serial_ports()
    {
        std::vector<std::string> matched_ports;
        std::vector<std::string> ports = list_serial_ports(config_.serial_type);
        RCLCPP_INFO(logger_, "Serial Port List:");
        for (const std::string &port : ports)
        {
            if (is_supported_serial_port(port))
            {
                RCLCPP_INFO(logger_, "Serial Port%zu = %s", matched_ports.size(), port.c_str());
                matched_ports.push_back(port);
            }
        }
        return matched_ports;
    }

    void robot::create_serial_devices_for_config(const std::vector<std::string> &ports)
    {
        std::unordered_set<int> used_serial_ids;
        for (const auto &board_config : config_.boards)
        {
            RCLCPP_INFO(logger_, "board %d has %d port", board_config.canboard_id, board_config.canport_num);

            for (const auto &port_config : board_config.ports)
            {
                const int serial_id = port_config.serial_id;
                if (serial_id > static_cast<int>(ports.size()) || serial_id < 1)
                {
                    throw std::runtime_error("serial_id is outside the discovered serial port range");
                }
                if (!used_serial_ids.insert(serial_id).second)
                {
                    throw std::runtime_error("The serial_id is duplicated across configured CAN ports");
                }

                auto serial_device = std::make_unique<lively_serial>(
                    ports[serial_id - 1], config_.serial_baudrate, now_seconds_, logger_);
                lively_serial *s = serial_device.get();
                serial_devices_.push_back(std::move(serial_device));
                serials_.push_back(s);
            }
        }
    }

    void robot::start_serial_receivers()
    {
        serial_receive_threads_.reserve(serials_.size());
        for (lively_serial *s : serials_)
        {
            serial_receive_threads_.push_back(std::thread(&lively_serial::receive_loop, s));
        }
    }

    void robot::initialize_serial_devices()
    {
        serials_.clear();
        serial_receive_threads_.clear();
        create_serial_devices_for_config(discover_matching_serial_ports());
        start_serial_receivers();
    }

    enum class ErrorRunState
    {
        Check = 0,
        Clear,
        WaitDevice,
        Reconnect,
    };

    void robot::check_error()
    {
        ErrorRunState last_error_run_state = ErrorRunState::Reconnect;
        ErrorRunState error_run_state = ErrorRunState::Check;
        while (error_check_running_)
        {
            try
            {
                switch (error_run_state)
                {
                case ErrorRunState::Check:
                {
                    std::lock_guard<std::mutex> lock(runtime_mutex_);
                    bool serial_error = false;
                    for (lively_serial *s : serials_)
                    {
                        if (s->is_serial_error())
                        {
                            serial_error = true;
                            break;
                        }
                    }
                    if (serial_error)
                    {
                        error_run_state = ErrorRunState::Clear;
                        RCLCPP_ERROR(logger_, "Serial error");
                    }
                }
                break;
                case ErrorRunState::Clear:
                {
                    std::lock_guard<std::mutex> lock(runtime_mutex_);
                    cleanup_runtime();
                    error_run_state = ErrorRunState::WaitDevice;
                    RCLCPP_INFO(logger_, "Cleared serial objects and receive threads");
                }
                break;
                case ErrorRunState::WaitDevice:
                {
                    const int expected_serial_devices = expected_serial_device_count();
                    const int scan_limit = std::max(8, expected_serial_devices);
                    int exist_num = this->count_existing_serial_devices(scan_limit);
                    RCLCPP_INFO(logger_, "Found %d serial device(s)", exist_num);
                    if (exist_num < expected_serial_devices)
                    {
                        RCLCPP_ERROR(
                            logger_,
                            "Cannot find %d motor serial ports, please check whether the USB connection is normal",
                            expected_serial_devices);
                    }
                    else
                    {
                        RCLCPP_INFO(logger_, "Found all serial devices");
                        error_run_state = ErrorRunState::Reconnect;
                        if (!wait_while_running(error_check_running_, std::chrono::milliseconds(5000)))
                        {
                            break;
                        }
                    }
                }
                break;
                case ErrorRunState::Reconnect:
                {
                    std::lock_guard<std::mutex> lock(runtime_mutex_);
                    RCLCPP_INFO(logger_, "Reconnect start");
                    this->initialize_serial_devices();
                    build_runtime_topology();
                    configure_port_motor_counts(); // 设置通道上挂载的电机数，并获取主控板固件版本号
                    check_motor_connection_for_firmware();
                    error_run_state = ErrorRunState::Check;
                    RCLCPP_INFO(logger_, "Reconnect end");
                }
                break;
                default:
                    break;
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(logger_, "Serial error recovery failed: %s", e.what());
                {
                    std::lock_guard<std::mutex> lock(runtime_mutex_);
                    cleanup_runtime();
                }
                error_run_state = ErrorRunState::WaitDevice;
            }
            if (!wait_while_running(error_check_running_, std::chrono::milliseconds(1000)))
            {
                break;
            }
            // Only print when the state changes.
            if (error_run_state != last_error_run_state)
            {
                last_error_run_state = error_run_state;
                RCLCPP_INFO(logger_, "error_run_state = %d", static_cast<int>(error_run_state));
            }
        }
    }

    int robot::count_existing_serial_devices(int scan_limit)
    {
        int exist_num = 0;
        RCLCPP_INFO(logger_, "Checking serial device existence");
        for (int i = 0; i < scan_limit; i++)
        {
            std::string device_path = std::string("/dev/ttyACM") + std::to_string(i);
            RCLCPP_INFO(logger_, "Checking: %s", device_path.c_str());
            std::error_code error;
            if (std::filesystem::exists(device_path, error))
            {
                exist_num++;
                RCLCPP_INFO(logger_, "Exists: %s", device_path.c_str());
            }
        }
        RCLCPP_INFO(logger_, "exist_num = %d", exist_num);
        return exist_num;
    }


    /**
     * @brief 设置每个通道的电机数量，并查询主控板固件版本
     */
    void robot::configure_port_motor_counts()
    {
        for (canboard &cb : can_boards_)
        {
            slave_version_ = cb.configure_port_motor_counts();
        }
    }
    
    
    void robot::request_motor_state()
    {
        if (function_version_ >= fun_v4)
        {
            for_each_board(&canboard::request_motor_state_with_mode);
        }
        else if (function_version_ >= fun_v2 || config_.control_type == 0)
        {
            for_each_board(&canboard::request_motor_state);
        }
        else
        {
            for (motor *m : motors_)
            {
                m->velocity(0.0f);
            }
            send_motor_command_frame();
        }
    }


    void robot::request_motor_version()
    {
        if (slave_version_ < 4.0f)
        {
            throw std::runtime_error("The current communication board does not support this function");
        }

        for_each_board(&canboard::request_motor_version);
    }


    void robot::motor_version_detection()
    {
        uint16_t v_old = 0xFFFF;
        uint16_t i = 0;

        RCLCPP_INFO(logger_, "---------------motor version---------------------");
        for (motor *m : motors_)
        {
            const auto v = m->firmware_version();
            RCLCPP_INFO(logger_, "motors[%02d]: id:%02d v%d.%d.%d", i++, v.id, v.major, v.minor, v.patch);
            const uint16_t v_new = v.major << 12 | (v.minor << 4) | v.patch;
            if (v_old > v_new && v_new != 0)
            {
                v_old = v_new;
            }
        }
        RCLCPP_INFO(logger_, "-------------------------------------------------");

        if (v_old >= combine_version(4, 4, 6))
        {
            function_version_ = fun_v5;
        }
        else if (v_old >= combine_version(4, 2, 3))
        {
            function_version_ = fun_v4;
        }
        else if (v_old >= combine_version(4, 2, 2))
        {
            function_version_ = fun_v3;
        }
        else if (v_old >= combine_version(4, 2, 0))
        {
            function_version_ = fun_v2;
        }
        else
        {
            function_version_ = fun_v1;
        }

        for (canboard &cb : can_boards_)
        {
            cb.set_function_version(function_version_);
        }

        RCLCPP_INFO(logger_, "function version = %d", function_version_);


        uint8_t v = 0;
        uint8_t v2 = 0;

        for (motor *m : motors_)
        {
            v = m->firmware_version().major;

            if (v == 5 && v2 != 0 && v != v2)
            {
                throw std::runtime_error("Inconsistent motor version");
            }

            v2 = v;

            if (v == 5)
            {
                m->set_motor_type(mGeneral);
            }
        }
    }


    // 将所有数据置为 0xFF
    void robot::reset_data()
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        reset_data_unlocked();
    }

    void robot::reset_data_unlocked()
    {
        if (function_version_ < fun_v3)
        {
            throw std::runtime_error("The current feature version does not support data reset");
        }

        for_each_board(&canboard::reset_data);
    }


    void robot::check_motor_connection_for_firmware()
    {
        if (slave_version_ < 4.0f)
        {
            function_version_ = fun_v1;
            check_motor_connection_position();
            return;
        }

        function_version_ = fun_v2;
        check_motor_connection_version();
    }


    void robot::check_motor_connection_version()
    {
        int t = 0;
        MotorConnectionStatus status;

        RCLCPP_INFO(logger_, "Detecting motor connection");
        while (t++ < 20)
        {
            request_motor_version();
            sleep_for_seconds(0.1);

            status = collect_motor_connection_status(
                motors_,
                [](motor *m) {
                    const cdc_rx_motor_version_s &v = m->firmware_version();
                    return v.major != 0;
                });

            if (status.connected_count == static_cast<int>(motors_.size()))
            {
                break;
            }

            if (t % 100 == 0)
            {
                RCLCPP_INFO(logger_, ".");
            }
        }

        report_motor_connection_status(logger_, status, motors_.size(), 3.0);
        motor_version_detection();
    }


    void robot::check_motor_connection_position()
    {
        int t = 0;
        MotorConnectionStatus status;

        constexpr int kMaxDelay = 2000; // 单位 ms

        RCLCPP_INFO(logger_, "Detecting motor connection");
        while (t++ < kMaxDelay)
        {
            request_motor_state();
            sleep_for_seconds(0.001);

            status = collect_motor_connection_status(
                motors_,
                [](motor *m) {
                    return m->state().position != kUninitializedMotorPosition;
                });

            if (status.connected_count == static_cast<int>(motors_.size()))
            {
                break;
            }

            if (t % 1000 == 0)
            {
                RCLCPP_INFO(logger_, ".");
            }
        }

        report_motor_connection_status(logger_, status, motors_.size(), 5.0);
    }


    void robot::stop_motors()
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        stop_motors_unlocked();
    }

    void robot::stop_motors_unlocked()
    {
        for_each_board(&canboard::stop_motors);
    }


    void robot::reset_motors()
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        reset_motors_unlocked();
    }

    void robot::reset_motors_unlocked()
    {
        for_each_board(&canboard::reset_motors);
        sleep_for_seconds(0.2);
    }


    void robot::reset_zero_positions()
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        reset_zero_positions_unlocked();
    }

    void robot::reset_zero_positions_unlocked()
    {
        for_each_board(&canboard::reset_zero_positions);
    }


    void robot::reset_zero_positions(std::initializer_list<int> motors)
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        reset_zero_positions_unlocked(motors);
    }

    void robot::reset_zero_positions_unlocked(std::initializer_list<int> motors)
    {
        for (auto const &motor_index : motors)
        {
            motor &selected_motor = require_motor(motor_index);
            int motor_id = selected_motor.motor_id();
            RCLCPP_INFO(
                logger_, "%d, %d, %d",
                selected_motor.canboard_id(),
                selected_motor.canport_id(),
                motor_id);

            reset_motors_unlocked();
            sleep_for_seconds(0.1);
            
            RCLCPP_INFO(logger_, "Motor %d settings have been successfully restored. Initiating zero position reset.", motor_index);
            canport &selected_port = require_motor_port(selected_motor);
            if (selected_port.reset_zero_positions(motor_id) == 0)
            {
                RCLCPP_INFO(logger_, "Motor %d reset to zero position successfully, awaiting settings save.", motor_index);
                if (selected_port.save_configuration(motor_id) == 0)
                {
                    RCLCPP_INFO(logger_, "Motor %d settings saved successfully.", motor_index);
                }
                else
                {
                    RCLCPP_ERROR(logger_, "Motor %d settings saved failed.", motor_index);
                }
            }
            else
            {
                RCLCPP_ERROR(logger_, "Motor %d reset to zero position failed.", motor_index);
            }
        }
    }


    void robot::enable_motor_runzero()
    {
        repeat_for_all_boards(5, 0.01, &canboard::enable_motor_runzero);
        sleep_for_seconds(1.0);
    }


    void robot::set_motor_timeout(int16_t t_ms)
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        set_motor_timeout_unlocked(t_ms);
    }

    void robot::set_motor_timeout_unlocked(int16_t t_ms)
    {
        validate_timeout_ms(t_ms);
        repeat_timeout_for_all_boards(5, 0.01, t_ms);
    }

    void robot::set_motor_timeout(uint8_t portx, int16_t t_ms)
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        set_motor_timeout_unlocked(portx, t_ms);
    }

    void robot::set_motor_timeout_unlocked(uint8_t portx, int16_t t_ms)
    {
        validate_timeout_ms(t_ms);
        repeat_timeout_for_port(5, 0.01, portx, t_ms);
    }

    void robot::enter_canboard_bootloader()
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        enter_canboard_bootloader_unlocked();
    }

    void robot::enter_canboard_bootloader_unlocked()
    {
        for_each_board(&canboard::enter_canboard_bootloader);
    }

    void robot::reset_canboard_fdcan()
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        reset_canboard_fdcan_unlocked();
    }

    void robot::reset_canboard_fdcan_unlocked()
    {
        RCLCPP_INFO(logger_, "canboard fdcan reset");
        for_each_board(&canboard::reset_canboard_fdcan);
        sleep_for_seconds(0.01);
    }
}
