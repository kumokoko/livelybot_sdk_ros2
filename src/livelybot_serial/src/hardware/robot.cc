#include "robot.h"



namespace livelybot_serial
{
    namespace
    {
        template<typename ParamT>
        bool get_ros1_style_parameter(rclcpp::Node &node, const std::string &key, ParamT &value)
        {
            return node.get_parameter(livelybot_serial_ros2::parameter_key_from_ros1(key), value);
        }

        constexpr float kMaxImuLimitNum = 1.57f;
        constexpr int kMaxMotorTimeoutMs = 32760;
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

        if (!get_ros1_style_parameter(node, base_key + "/name", motor_config.motor_name) ||
            !get_ros1_style_parameter(node, base_key + "/id", motor_config.id) ||
            !get_ros1_style_parameter(node, base_key + "/type", motor_config.type_name) ||
            !get_ros1_style_parameter(node, base_key + "/num", motor_config.num) ||
            !get_ros1_style_parameter(node, base_key + "/pos_limit_enable", motor_config.pos_limit_enable) ||
            !get_ros1_style_parameter(node, base_key + "/pos_upper", motor_config.pos_upper) ||
            !get_ros1_style_parameter(node, base_key + "/pos_lower", motor_config.pos_lower) ||
            !get_ros1_style_parameter(node, base_key + "/tor_limit_enable", motor_config.tor_limit_enable) ||
            !get_ros1_style_parameter(node, base_key + "/tor_upper", motor_config.tor_upper) ||
            !get_ros1_style_parameter(node, base_key + "/tor_lower", motor_config.tor_lower))
        {
            ROS_ERROR("Failed to get motor config: %s", base_key.c_str());
            exit(-1);
        }

        return motor_config;
    }

    canport::config robot::load_port_config(
        rclcpp::Node &node, int cb_id, int cp_id, int control_type)
    {
        canport::config port_config;
        port_config.canboard_id = cb_id;
        port_config.canport_id = cp_id;

        if (!get_ros1_style_parameter(
                node,
                "robot/CANboard/No_" + std::to_string(cb_id) +
                  "_CANboard/CANport/CANport_" + std::to_string(cp_id) + "/motor_num",
                port_config.motor_num))
        {
            ROS_ERROR("Faile to get params motor_num");
            exit(-1);
        }
        if (!get_ros1_style_parameter(
                node,
                "robot/CANboard/No_" + std::to_string(cb_id) +
                  "_CANboard/CANport/CANport_" + std::to_string(cp_id) + "/serial_id",
                port_config.serial_id))
        {
            ROS_ERROR("serial_id error!!!");
            exit(-1);
        }

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
        if (!get_ros1_style_parameter(
                node,
                "robot/CANboard/No_" + std::to_string(cb_id) + "_CANboard/CANport_num",
                board_config.canport_num))
        {
            ROS_ERROR("Faile to get params CANport_num");
            exit(-1);
        }

        for (int cp_id = 1; cp_id <= board_config.canport_num; ++cp_id)
        {
            board_config.ports.push_back(load_port_config(node, cb_id, cp_id, control_type));
        }

        return board_config;
    }

    robot::runtime_config robot::load_runtime_config(rclcpp::Node &node)
    {
        runtime_config config;
        if (!get_ros1_style_parameter(node, "robot/Seial_baudrate", config.serial_baudrate))
        {
            ROS_ERROR("Faile to get params seial_baudrate");
        }
        if (!get_ros1_style_parameter(node, "robot/robot_name", config.robot_name))
        {
            ROS_ERROR("Faile to get params robot_name");
        }
        if (!get_ros1_style_parameter(node, "robot/CANboard_num", config.canboard_num))
        {
            ROS_ERROR("Faile to get params CANboard_num");
        }
        if (!get_ros1_style_parameter(node, "robot/Serial_Type", config.serial_type))
        {
            ROS_ERROR("Faile to get params Serial_Type");
        }
        if (!get_ros1_style_parameter(node, "robot/control_type", config.control_type))
        {
            ROS_ERROR("Faile to get params control_type");
        }
        if (!get_ros1_style_parameter(node, "robot/imu_limt_flag", config.imu_limit_flag))
        {
            ROS_ERROR("Faile to get params imu_limt_flag");
            exit(-1);
        }
        if (!get_ros1_style_parameter(node, "robot/imu_dir", config.imu_dir))
        {
            ROS_ERROR("Faile to get params imu_dir");
            exit(-1);
        }
        if (!get_ros1_style_parameter(node, "robot/imu_limt_num", config.imu_limit_num))
        {
            ROS_ERROR("Faile to get params imu_limt_num");
            exit(-1);
        }
        if (!get_ros1_style_parameter(node, "robot/motor_timeout_ms", config.motor_timeout_ms))
        {
            ROS_ERROR("Faile to get params motor_timeout_ms");
            exit(-1);
        }

        for (int cb_id = 1; cb_id <= config.canboard_num; ++cb_id)
        {
            config.boards.push_back(load_board_config(node, cb_id, config.control_type));
        }
        validate_runtime_config(config);
        return config;
    }

    void robot::validate_runtime_config(const runtime_config &config)
    {
        if (config.imu_limit_num > kMaxImuLimitNum)
        {
            ROS_ERROR("The value of imu_limt_num must not exceed %.2f", kMaxImuLimitNum);
            exit(-1);
        }
        if (config.motor_timeout_ms < 0 || config.motor_timeout_ms > kMaxMotorTimeoutMs)
        {
            ROS_ERROR("The value of motor_timeout_ms is out of the valid range [0, %d]", kMaxMotorTimeoutMs);
            exit(-1);
        }
    }

    std::vector<lively_serial *> robot::collect_board_serials(
        const canboard::config &board_config, size_t &serial_offset)
    {
        std::vector<lively_serial *> board_serials;
        board_serials.reserve(board_config.ports.size());
        for (size_t port_index = 0; port_index < board_config.ports.size(); ++port_index)
        {
            if (serial_offset >= ser.size())
            {
                ROS_ERROR("serial_id error!!!");
                exit(-1);
            }
            board_serials.push_back(ser[serial_offset++]);
        }
        return board_serials;
    }

    void robot::build_runtime_topology()
    {
        size_t serial_offset = 0;
        for (const auto &board_config : config_.boards)
        {
            CANboards.emplace_back(board_config, collect_board_serials(board_config, serial_offset));
        }

        CANPorts.clear();
        Motors.clear();
        for (canboard &cb : CANboards)
        {
            cb.push_CANport(&CANPorts);
        }
        for (canport *cp : CANPorts)
        {
            cp->puch_motor(&Motors);
        }
    }

    void robot::clear_runtime_topology()
    {
        Motors.clear();
        CANPorts.clear();
        CANboards.clear();
    }

    void robot::stop_serial_receivers()
    {
        for (lively_serial *s : ser)
        {
            s->set_run_flag(false);
        }
        for (auto &thread : ser_recv_threads)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }
        ser_recv_threads.clear();
    }

    void robot::destroy_serial_devices()
    {
        for (lively_serial *s : ser)
        {
            delete s;
        }
        ser.clear();
    }

    robot::robot(const runtime_config &_config)
    {
        config_ = _config;
        Seial_baudrate = _config.serial_baudrate;
        robot_name = _config.robot_name;
        CANboard_num = _config.canboard_num;
        Serial_Type = _config.serial_type;
        control_type = _config.control_type;
        imu_limt_flag = _config.imu_limit_flag;
        imu_dir = _config.imu_dir;
        imu_limt_num = _config.imu_limit_num;
        motor_timeout_ms = _config.motor_timeout_ms;

        if (imu_limt_flag)
        {
            ROS_INFO("IMU needs to be enabled");
        }

        ROS_INFO("\033[1;32mGot params SDK_version: v%s\033[0m", SDK_version2.c_str());
        ROS_INFO("\033[1;32mThe robot name is %s\033[0m", robot_name.c_str());
        ROS_INFO("\033[1;32mThe robot has %d CANboards\033[0m", CANboard_num);
        ROS_INFO("\033[1;32mThe Serial type is %s\033[0m", Serial_Type.c_str());
        init_ser();
        error_check_thread_ = std::thread(&robot::check_error, this);

        build_runtime_topology();
        set_port_motor_num(); // 设置通道上挂载的电机数，并获取主控板固件版本号
        if (slave_v >= 4.1f)
        {
            canboard_fdcan_reset();
        }

        if (slave_v < 4.0f)  // 检测电机连接是否正常
        {
            fun_v = fun_v1;
            chevk_motor_connection_position();   
        }
        else
        {
            fun_v = fun_v2;
            chevk_motor_connection_version();
        }

        if (motor_timeout_ms != 0)
        {
            set_timeout(motor_timeout_ms);
        }
        
        send_get_motor_state_cmd();

        ROS_INFO("\033[1;32mThe robot has %ld motors\033[0m", Motors.size());
        ROS_INFO("robot init");
    }
    robot::~robot()
    {
        set_reset();
        set_reset();
        set_reset();
        stop_serial_receivers();
        clear_runtime_topology();
        destroy_serial_devices();
        if(error_check_thread_.joinable())
        {
            error_check_thread_.join(); 
        }
    }

    void robot::update_imu_state(const sensor_msgs::msg::Imu &msg)
    {
        const double w = msg.orientation.w;
        const double x = msg.orientation.x;
        const double y = msg.orientation.y;
        const double z = msg.orientation.z;

        const double sinr_cosp = 2 * (w * x + y * z);
        const double cosr_cosp = 1 - 2 * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        const double sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
        {
            pitch = std::copysign(M_PI / 2, sinp); 
        }
        else
        {
            pitch = std::asin(sinp);
        }
    }

    sensor_msgs::msg::JointState robot::build_joint_state_message() const
    {
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = livelybot_serial_ros2::now();

        for (motor *m : Motors)
        {    
            joint_state_msg.name.push_back(m->get_motor_name());
            motor_back_t* data_ptr=m->get_current_motor_state();
            const double now_time = livelybot_serial_ros2::now_seconds();
            if(now_time - data_ptr->time > 0.1)
            {
                joint_state_msg.position.push_back(-999);
                joint_state_msg.velocity.push_back(0);
                joint_state_msg.effort.push_back(0);
            }
            else
            {
                joint_state_msg.position.push_back(data_ptr->position);
                joint_state_msg.velocity.push_back(data_ptr->velocity);
                joint_state_msg.effort.push_back(data_ptr->torque);
            }
        }
        return joint_state_msg;
    }
    void robot::detect_motor_limit()
    {
        // 电机正常运行时检测是否超过限位，停机之后不检测
        if(!motor_position_limit_flag && !motor_torque_limit_flag)
        {
            for (motor *m : Motors)
            {
                if(m->pos_limit_flag)
                {                    
                    ROS_ERROR("robot pos limit, motor stop.");
                    set_stop();
                    motor_position_limit_flag = m->pos_limit_flag;
                    break;
                }

                if(m->tor_limit_flag)
                {
                    ROS_ERROR("robot torque limit, motor stop.");
                    set_stop();
                    motor_torque_limit_flag = m->tor_limit_flag;
                    break;
                }
            }            
        }
    }


    bool robot::imu_limt()
    {
        float roll_err = 0.0f;

        if (imu_limt_flag != true)
        {
            return true;
        }

        if (imu_dir == true)
        {
            roll_err = fabs(roll - 0.0f);
        }
        else
        {
            if(roll < 0)
            {
                roll_err = roll + 3.14;
            }
            else
            {
                roll_err = 3.14 - roll;
            }
        }

        if (roll_err > imu_limt_num || pitch > imu_limt_num)
        {
            return false;
        }

        return true;
    }


    void robot::motor_send_2()
    {
        if (imu_limt() == false)
        {
            for (motor *m : Motors)
            {
                m->pos_vel_tqe_kp_kd(m->get_current_motor_state()->position, 0, 0, 10, 1);
            }
        }

        if(!motor_position_limit_flag && !motor_torque_limit_flag)
        {
            for (canboard &cb : CANboards)
            {
                cb.motor_send_2();
            }
        }
        
    }


    int robot::serial_pid_vid(const char *name, int *pid, int *vid)
    {
        int r = 0;
        struct sp_port *port;
        try
        {
            /* code */
            sp_get_port_by_name(name, &port);
            sp_open(port, SP_MODE_READ);
            if (sp_get_port_usb_vid_pid(port, vid, pid) != SP_OK) 
            {
                r = 1;
            } 
            std::cout << "Port: " << name << ", PID: 0x" << std::hex << *pid << ", VID: 0x" << *vid << std::dec << std::endl;

            // 关闭端口
            sp_close(port);
            sp_free_port(port);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            sp_close(port);
            sp_free_port(port);
        }
        return r;
    }


    int robot::serial_pid_vid(const char *name)
    {
        int pid, vid;
        int r = 0;
        struct sp_port *port;
        try
        {
            sp_get_port_by_name(name, &port);
            if (sp_get_port_usb_vid_pid(port, &vid, &pid) != SP_OK) 
            {
                r = -1;
            } 
            else 
            {
                if (pid == 0xFFFF)
                {
                    switch (vid)
                    {
                    case (0xCAF1):
                    case (0xCAE1):
                        r = 1;
                        break;
                    default:
                        r = -3;
                        break;
                    }
                }
                else
                {
                    r = -1;
                }
            }
            // std::cout << "Port: " << name << ", PID: 0x" << std::hex << pid << ", VID: 0x" << vid << std::dec << std::endl;
            sp_free_port(port);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            r = -3;
            sp_close(port);
            sp_free_port(port);
        }

        return r;
    }


    std::vector<std::string> robot::list_serial_ports(const std::string& full_prefix) 
    {
        std::string base_path = full_prefix.substr(0, full_prefix.rfind('/') + 1);
        std::string prefix = full_prefix.substr(full_prefix.rfind('/') + 1);
        std::vector<std::string> serial_ports;
        DIR *directory;
        struct dirent *entry;

        directory = opendir(base_path.c_str());
        if (!directory)
        {
            std::cerr << "Could not open the directory " << base_path << std::endl;
            return serial_ports; 
        }

        while ((entry = readdir(directory)) != NULL)
        {
            std::string entryName = entry->d_name;
            if (entryName.find(prefix) == 0)
            { 
                serial_ports.push_back(base_path + entryName);
            }
        }

        closedir(directory);

        std::reverse(serial_ports.begin(), serial_ports.end());

        return serial_ports;
    }

    std::vector<std::string> robot::discover_matching_serial_ports()
    {
        std::vector<std::string> matched_ports;
        std::vector<std::string> ports = list_serial_ports(Serial_Type);
        std::cout << "Serial Port List: " << std::endl;
        for (const std::string &port : ports)
        {
            if (serial_pid_vid(port.c_str()) > 0)
            {
                ROS_INFO("Serial Port%ld = %s", matched_ports.size(), port.c_str());
                matched_ports.push_back(port);
            }
        }
        return matched_ports;
    }

    void robot::create_serial_devices_for_config(const std::vector<std::string> &ports)
    {
        for (const auto &board_config : config_.boards)
        {
            ROS_INFO("board %d has %d port", board_config.canboard_id, board_config.canport_num);

            std::vector<int> serial_id_old;
            for (const auto &port_config : board_config.ports)
            {
                const int serial_id = port_config.serial_id;
                if (serial_id > static_cast<int>(ports.size()) || serial_id < 1)
                {
                    ROS_ERROR("serial_id error!!!");
                    exit(-1);
                }
                if (
                    !serial_id_old.empty() &&
                    std::find(serial_id_old.begin(), serial_id_old.end(), serial_id) != serial_id_old.end())
                {
                    ROS_ERROR("The serial_id is duplicated!!!");
                    exit(-1);
                }
                serial_id_old.push_back(serial_id);

                lively_serial *s = new lively_serial(ports[serial_id - 1], Seial_baudrate);
                ser.push_back(s);
                ser_recv_threads.push_back(std::thread(&lively_serial::recv_1for6_42, s));
            }
        }
    }


    void robot::init_ser()
    {
        ser.clear();
        ser_recv_threads.clear();
        str = discover_matching_serial_ports();
        create_serial_devices_for_config(str);
    }

    typedef enum{
        error_check = 0,    // 正常
        error_clear,        // 报错，清理     
        error_wait_dev,     // 报错，等待设备
        error_reconnect,    // 报错，重连
    }error_run_state_e;

    void robot::check_error(void)
    {
        std::mutex robot_mutex;
        while(true)
        {
            static error_run_state_e last_error_run_state = error_reconnect;
            static error_run_state_e error_run_state = error_check;// 0：正常，1：报错,清理，2：重连
            switch(error_run_state)
            {
                case 0:
                {
                    bool serial_error = false;
                    for (lively_serial *s : ser)
                    {
                        if (s->is_serial_error())
                        {
                            serial_error = true;
                            break;
                        }
                    }
                    if(serial_error)
                    {
                        serial_error = false;
                        error_run_state = error_clear;
                        std::cerr << "Serial error" << std::endl;
                    }
                }
                break;
                case error_clear:
                {
                    std::lock_guard<std::mutex> lock(robot_mutex);
                    stop_serial_receivers();
                    clear_runtime_topology();
                    destroy_serial_devices();
                    error_run_state = error_wait_dev;
                    std::cerr << "clear obj and thread" << std::endl;
                }
                break;
                case error_wait_dev:
                {
                    int exist_num = this->check_serial_dev_exist(8);
                    std::cerr << "find "  << exist_num << " device(s)" << std::endl;
                    if (exist_num < 4)
                    {
                        std::cerr << "Cannot find 4 motor serial port, please check if the USB connection is normal." << std::endl;
                    }
                    else
                    {
                        std::cout << "file all diveces" << std::endl;
                        error_run_state = error_reconnect;
                        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                    }
                }
                break;
                case error_reconnect:
                {
                    std::cerr << "reconnect start " << std::endl;
                    this->init_ser();
                    build_runtime_topology();
                    set_port_motor_num(); // 设置通道上挂载的电机数，并获取主控板固件版本号
                    chevk_motor_connection_version();  // 检测电机连接是否正常
                    error_run_state = error_check;
                    std::cerr << "reconnect end" << std::endl;
                }
                break;
                default:
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            // only state chaged, print state.
            if(error_run_state != last_error_run_state)
            {
                last_error_run_state = error_run_state;
                std::cout << "error_run_state = " << error_run_state << std::endl;
            }
        }
    }

    int robot::check_serial_dev_exist(int file_num)
    {
        int exist_num = 0;
        std::cout << "check serial dev exist" << std::endl;
        std::vector<std::string> dev_vec;
        for (size_t i = 0; i < file_num; i++)
        {
            std::string _dev = std::string("/dev/ttyACM") + std::to_string(i);
            std::cout << "check: " << _dev << std::endl;
            dev_vec.push_back(_dev);
        }
        for (auto &dev : dev_vec)
        {
            if(access(dev.c_str(),F_OK) == 0)
            {
                exist_num++;
                std::cout << "exist: " << dev << std::endl;
            }
        }
        std::cout << "exist_num = " << exist_num << std::endl;
        return exist_num;
    }


    /**
     * @brief 设置每个通道的电机数量，并查询主控板固件版本
     */
    void robot::set_port_motor_num()
    {
        for (canboard &cb : CANboards)
        {
            slave_v = cb.set_port_motor_num();
        }
    }
    
    
    void robot::send_get_motor_state_cmd()
    {
        if (fun_v >= fun_v4)
        {
            for (canboard &cb : CANboards)
            {
                cb.send_get_motor_state_cmd2();
            }
        }
        else if (fun_v >= fun_v2 || control_type == 0)
        {
            for (canboard &cb : CANboards)
            {
                cb.send_get_motor_state_cmd();
            }
        }
        else
        {
            for (motor *m : Motors)
            {
                m->velocity(0.0f);
            }
            motor_send_2();
        }
    }


    void robot::send_get_motor_version_cmd()
    {
        if (slave_v < 4.0f)
        {
            ROS_ERROR("The current communication board does not support this function!!!");
            exit(-1);
        }

        for (canboard &cb : CANboards)
        {
            cb.send_get_motor_version_cmd();
        }
    }


    void robot::motor_version_detection()
    {
        uint16_t v_old = 0xFFFF;
        uint16_t i = 0;

        ROS_INFO("---------------motor version---------------------");
        for (motor *m : Motors)
        {
            const auto v = m->get_version();
            ROS_INFO("motors[%02d]: id:%02d v%d.%d.%d", i++, v.id, v.major, v.minor, v.patch);
            const uint16_t v_new = v.major << 12 | (v.minor << 4) | v.patch;
            if (v_old > v_new && v_new != 0)
            {
                v_old = v_new;
            }
        }
        ROS_INFO("-------------------------------------------------");

        if (v_old >= COMBINE_VERSION(4, 4, 6))
        {
            fun_v = fun_v5;
        }
        else if (v_old >= COMBINE_VERSION(4, 2, 3))
        {
            fun_v = fun_v4;
        }
        else if (v_old >= COMBINE_VERSION(4, 2, 2))
        {
            fun_v = fun_v3;
        }
        else if (v_old >= COMBINE_VERSION(4, 2, 0))
        {
            fun_v = fun_v2;
        }
        else
        {
            fun_v = fun_v1;
        }

        for (canboard &cb : CANboards)
        {
            cb.set_fun_v(fun_v);
        }

        ROS_INFO("fun_v = %d", fun_v);


        uint8_t v = 0;
        uint8_t v2 = 0;

        for (motor *m : Motors)
        {
            v = m->get_version().major;

            if (v == 5 && v2 != 0 && v != v2)
            {
                ROS_ERROR("The motor version is notInconsistent motor version!!!");
                exit(-1);
            }

            v2 = v;

            if (v == 5)
            {
                m->set_type(mGeneral);
            }
        }
    }


    // 将所有数据置为 0xFF
    void robot::set_data_reset()
    {
        if (fun_v < fun_v3)
        {
            ROS_ERROR("The current feature version is not supported!!!");
            exit(-3);
        }

        for (canboard &cb : CANboards)
        {
            cb.set_data_reset();
        }
    }


    void robot::chevk_motor_connection_version()
    {
        int t = 0;
        int num = 0;
        std::vector<int> board;
        std::vector<int> port;
        std::vector<int> id;

        ROS_INFO("Detecting motor connection");
        while (t++ < 20)
        {
            send_get_motor_version_cmd();
            livelybot_serial_ros2::sleep_for_seconds(0.1);

            num = 0;
            std::vector<int>().swap(board);
            std::vector<int>().swap(port);
            std::vector<int>().swap(id);
            for (motor *m : Motors)
            {
                cdc_rx_motor_version_s &v = m->get_version();
                if (v.major != 0)
                {
                    ++num;
                }
                else
                {
                    board.push_back(m->get_motor_belong_canboard());
                    port.push_back(m->get_motor_belong_canport());
                    id.push_back(m->get_motor_id());
                }
            }

            if (num == Motors.size())
            {
                break;
            }

            if (t % 100 == 0)
            {
                ROS_INFO(".");
            }
        }

        if (num == Motors.size())
        {
            ROS_INFO("\033[1;32mAll motor connections are normal\033[0m");
        }
        else
        {
            for (int i = 0; i < Motors.size() - num; i++)
            {
                ROS_ERROR("CANboard(%d) CANport(%d) id(%d) Motor connection disconnected!!!", board[i], port[i], id[i]);
            }
            livelybot_serial_ros2::sleep_for_seconds(3.0);
        }
        motor_version_detection();
    }


    void robot::chevk_motor_connection_position()
    {
        int t = 0;
        int num = 0;
        std::vector<int> board;
        std::vector<int> port;
        std::vector<int> id;

#define MAX_DELAY 2000 // 单位 ms

        ROS_INFO("Detecting motor connection");
        while (t++ < MAX_DELAY)
        {
            send_get_motor_state_cmd();
            livelybot_serial_ros2::sleep_for_seconds(0.001);

            num = 0;
            std::vector<int>().swap(board);
            std::vector<int>().swap(port);
            std::vector<int>().swap(id);
            for (motor *m : Motors)
            {
                if (m->get_current_motor_state()->position != 999.0f)
                {
                    ++num;
                }
                else
                {
                    board.push_back(m->get_motor_belong_canboard());
                    port.push_back(m->get_motor_belong_canport());
                    id.push_back(m->get_motor_id());
                }
            }

            if (num == Motors.size())
            {
                break;
            }

            if (t % 1000 == 0)
            {
                ROS_INFO(".");
            }
        }

        if (num == Motors.size())
        {
            ROS_INFO("\033[1;32mAll motor connections are normal\033[0m");
        }
        else
        {
            for (int i = 0; i < Motors.size() - num; i++)
            {
                ROS_ERROR("CANboard(%d) CANport(%d) id(%d) Motor connection disconnected!!!", board[i], port[i], id[i]);
            }
            // exit(-1);
            livelybot_serial_ros2::sleep_for_seconds(5.0);
        }
    }


    void robot::set_stop()
    {
        for (canboard &cb : CANboards)
        {
            cb.set_stop();
        }
    }


    void robot::set_reset()
    {
        for (canboard &cb : CANboards)
        {
            cb.set_reset();
        }

        livelybot_serial_ros2::sleep_for_seconds(0.2);
    }


    void robot::set_reset_zero()
    {
        for (canboard &cb : CANboards)
        {
            cb.set_reset_zero();
        }
    }


    void robot::set_reset_zero(std::initializer_list<int> motors)
    {
        for (auto const &motor : motors)
        {
            int board_id = Motors[motor]->get_motor_belong_canboard() - 1;
            int port_id = Motors[motor]->get_motor_belong_canport() - 1;
            int motor_id = Motors[motor]->get_motor_id();
            ROS_INFO("%d, %d, %d\n", board_id, port_id, motor_id);

            set_reset();
            livelybot_serial_ros2::sleep_for_seconds(0.1);
            
            ROS_INFO("Motor %d settings have been successfully restored. Initiating zero position reset.", motor);
            if (CANPorts[port_id]->set_reset_zero(motor_id) == 0)
            {
                ROS_INFO("Motor %d reset to zero position successfully, awaiting settings save.null", motor);
                if (CANPorts[port_id]->set_conf_write(motor_id) == 0)
                {
                    ROS_INFO("Motor %d settings saved successfully.", motor);
                }
                else
                {
                    ROS_ERROR("Motor %d settings saved failed.", motor);
                }
            }
            else
            {
                ROS_ERROR("Motor %d reset to zero position failed.", motor);
            }
        }
    }


    void robot::set_motor_runzero()
    {
        for (int i = 0; i < 5; i++)
        {
            for (canboard &cb : CANboards)
            {
                cb.set_motor_runzero();
            }
            livelybot_serial_ros2::sleep_for_seconds(0.01);
        }
        livelybot_serial_ros2::sleep_for_seconds(1.0);
    }


    void robot::set_timeout(int16_t t_ms)
    {
        for (int i = 0; i < 5; i++)
        {
            for (canboard &cb : CANboards)
            {
                cb.set_time_out(t_ms);
            }
            livelybot_serial_ros2::sleep_for_seconds(0.01);
        }
    }

    void robot::set_timeout(uint8_t portx, int16_t t_ms)
    {
        for (int i = 0; i < 5; i++)
        {
            CANboards[0].set_time_out(portx, t_ms);
            livelybot_serial_ros2::sleep_for_seconds(0.01);
        }
    }

    void robot::canboard_bootloader()
    {
        for (canboard &cb : CANboards)
        {
            cb.canboard_bootloader();
        }
    }

    void robot::canboard_fdcan_reset()
    {
        ROS_INFO("canboard fdcan reset");
        for (canboard &cb : CANboards)
        {
            cb.canboard_fdcan_reset();
        }
        livelybot_serial_ros2::sleep_for_seconds(0.01);
    }
}
