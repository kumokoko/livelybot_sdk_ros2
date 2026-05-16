推荐使用鱼香ros安装ros：
```bash
wget http://fishros.com/install -O fishros && . fishros
```

## 依赖安装

本工作区按 ROS2 Humble 使用。建议先安装 ROS2，再安装下面这些编译和运行依赖：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-empy \
  libserialport0 \
  libserialport-dev \
  ros-humble-ament-cmake \
  ros-humble-ament-cmake-gtest \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  ros-humble-rclcpp \
  ros-humble-std-msgs \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-tf2 \
  ros-humble-launch \
  ros-humble-launch-ros
```

当前源码包之间的依赖关系：

```text
serial
livelybot_interfaces
livelybot_serial -> serial, livelybot_interfaces
livelybot_logger
livelybot_bringup -> livelybot_serial, livelybot_logger
```

已经删除的非电机包不再需要安装或启动：

```text
livelybot_power
yesense_imu
livelybot_oled
```

# livelybot_sdk_ros2 使用说明

这是面向高擎电机的 ROS2 迁移工作区。当前保留的核心包是：

- `serial`：串口通信库。
- `livelybot_interfaces`：电机控制消息。
- `livelybot_serial`：电机驱动节点。
- `livelybot_logger`：运行状态记录节点，可选启动。
- `livelybot_bringup`：电机启动文件和 YAML 配置。

常用启动方式仍然是：

1. 写好 YAML 配置文件
2. 用 launch 文件启动节点
3. 通过 topic 发送控制命令

当前 motor 驱动节点是 `livelybot_serial` 包里的 `motor_driver_node`。

## 编译

进入工作区：

```bash
cd ~/work/livelybot_sdk_ros2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

如果只想编译电机驱动和启动文件：

```bash
colcon build --packages-select livelybot_interfaces serial livelybot_serial livelybot_bringup
source install/setup.bash
```

## 启动

默认 launch 文件：

```bash
ros2 launch livelybot_bringup minimal_bringup.launch.py
```

这个 launch 默认会加载：

```text
src/livelybot_bringup/cfg/motor_params.minimal.yaml
```

默认只启动电机驱动。需要同时启动 logger 时：

```bash
ros2 launch livelybot_bringup minimal_bringup.launch.py \
  enable_motor:=true \
  enable_logger:=true
```

指定自己的电机配置文件：

```bash
ros2 launch livelybot_bringup minimal_bringup.launch.py \
  motor_params:=/absolute/path/to/motor_params.yaml
```

注意：ROS2 参数文件外面要有节点名和 `ros__parameters`，例如：

```yaml
motor_driver_node:
  ros__parameters:
    driver:
      period_ms: 5
      imu_topic: "/imu/data"
      joint_state_topic: "error_joint_states"
      command_topic: "low_cmd"
      broadcast_command_topic: "broadcast_command"
    robot:
      robot_name: "pai"
      Serial_Type: "/dev/ttyACM"
      Seial_baudrate: 4000000
      control_type: 9
      imu_limt_flag: false
      imu_dir: false
      imu_limt_num: 1.0
      motor_timeout_ms: 0
      CANboard_num: 1
      CANboard:
        No_1_CANboard:
          CANport_num: 1
          CANport:
            CANport_1:
              serial_id: 1
              motor_num: 6
              motor:
                motor1:
                  type: "5047_36_2"
                  id: 1
                  name: "L_low_foot"
                  num: 1
                  pos_limit_enable: false
                  pos_upper: 5.0
                  pos_lower: -5.0
                  tor_limit_enable: false
                  tor_upper: 5.0
                  tor_lower: -3.0
```

也就是说，原来 ROS1/SDK 风格的 `robot:` 内容仍然基本沿用，只是在 ROS2 参数文件里外面多包两层：

```yaml
motor_driver_node:
  ros__parameters:
    robot:
      ...
```

## 电机配置说明

`robot` 下面是机器人和通讯板配置。

常用字段：

```yaml
robot_name: "pai"
Serial_Type: "/dev/ttyACM"
Seial_baudrate: 4000000
control_type: 9
imu_limt_flag: false
imu_dir: false
imu_limt_num: 1.0
motor_timeout_ms: 0
CANboard_num: 1
```

字段说明：

- `robot_name`：机器人名字。
- `Serial_Type`：串口前缀，通常是 `/dev/ttyACM`，程序会查找 `/dev/ttyACM*`。
- `Seial_baudrate`：串口波特率，常用 `4000000`。
- `control_type`：`fresh_cmd_int16` 使用的控制模式。
- `imu_limt_flag`：角度限制开关。
- `imu_dir`：IMU 安装方向。
- `imu_limt_num`：角度限制值，范围 `[0, 1.57]`。
- `motor_timeout_ms`：电机超时时间。设为 `0` 表示关闭超时保护。
- `CANboard_num`：通讯板数量。

当前代码会读取这些核心字段。下面这些字段如果出现在旧配置里，目前不会影响运行：

```yaml
canport_error_output_flag: false
board_special_flag: false
```

## CAN 口和电机配置

示例：

```yaml
CANboard:
  No_1_CANboard:
    CANport_num: 2
    CANport:
      CANport_1:
        serial_id: 1
        motor_num: 6
        motor:
          motor1:
            type: "5047_36_2"
            id: 1
            name: "L_low_foot"
            num: 1
            pos_limit_enable: false
            pos_upper: 5.0
            pos_lower: -5.0
            tor_limit_enable: false
            tor_upper: 5.0
            tor_lower: -3.0
      CANport_2:
        serial_id: 2
        motor_num: 6
        motor:
          motor1:
            type: "5047_36_2"
            id: 1
            name: "R_low_foot"
            num: 1
            pos_limit_enable: false
            pos_upper: 5.0
            pos_lower: -5.0
            tor_limit_enable: false
            tor_upper: 5.0
            tor_lower: -3.0
```

注意：

- 当前配置支持最多 7 路 CAN。
- `serial_id` 要和实际连接顺序一致。
- 每个 CANport 下的 `motor_num` 要和 `motor:` 里写的电机数量一致。
- 每个 CANport 内电机 ID 不能重复。
- `motor1`, `motor2`, ... 要从 1 开始连续写。
- `num` 也要和顺序对应，例如 `motor3` 的 `num` 应为 `3`。
- `type` 填错不一定影响转动，但会影响力矩换算准确性；如果型号不在支持列表里，节点会启动失败并报 `Motor model error`。

电机顺序按照配置展开：

```text
CANport_1 motor1, motor2, ...
CANport_2 motor1, motor2, ...
CANport_3 motor1, motor2, ...
```

也就是说，如果 `CANport_1` 有 6 个电机，`CANport_2` 的第一个电机就是整体命令数组里的第 7 个。

## 控制模式

`control_type` 对应 `motor.fresh_cmd_int16(...)` 的模式：

```text
1  位置模式
2  速度模式
3  力矩模式
4  电压模式
5  电流模式
6  位置 + 速度 + 最大力矩模式
7  弃用
8  弃用
9  位置 + 速度 + 力矩 + Kp + Kd 运控模式
10 位置 + 速度 + Kp + Kd 模式
11 位置 + 速度 + 加速度模式
12 运控模式 2
```

常用配置一般是：

```yaml
control_type: 9
```

## ROS2 Topic

### 发送电机控制命令

默认命令 topic：

```text
low_cmd
```

消息类型：

```text
livelybot_interfaces/msg/LowCmd
```

`LowCmd` 内部是电机命令数组：

```text
MotorCmd[] motor_cmd
```

`MotorCmd` 字段：

```text
uint8 mode
float32 q
float32 dq
float32 tau
float32 kp
float32 kd
uint32[3] reserve
```

当前 `motor_driver_node` 使用这些字段：

```text
q    -> 目标位置
dq   -> 目标速度
tau  -> 目标力矩
kp   -> Kp
kd   -> Kd
```

命令数组数量必须不少于配置里的电机数量。多出来的命令会被忽略。

### Logger

默认不启动 logger：

```bash
ros2 launch livelybot_bringup minimal_bringup.launch.py enable_logger:=false
```

启动 logger：

```bash
ros2 launch livelybot_bringup minimal_bringup.launch.py enable_logger:=true
```

logger 会订阅电机状态、IMU 标准话题、BMS 标准话题和 `/logger/operation`。如果某些话题不存在，不影响电机驱动运行。

### 关节状态输出

默认状态 topic：

```text
error_joint_states
```

消息类型：

```text
sensor_msgs/msg/JointState
```

如果电机状态长时间没更新，对应位置会填为 `-999.0`。

### 广播命令

默认广播 topic：

```text
broadcast_command
```

消息类型：

```text
std_msgs/msg/UInt8
```

命令值：

```text
1  stop
2  reset
3  reset zero
```

示例：

```bash
ros2 topic pub --once /broadcast_command std_msgs/msg/UInt8 "{data: 1}"
```

## 常见终端报错

电机型号写错：

```text
Motor model error: xxx
```

串口打不开：

```text
Motor unable to open port
```

串口数量不够或 `serial_id` 写错：

```text
serial_id is outside the discovered serial port range
```

通讯板或 CAN 通道没识别到：

```text
CANboard(x) CANport(y) Connection disconnected!!!
```

配置里的电机没识别到：

```text
CANboard(x) CANport(y) id(z) Motor connection disconnected!!!
```

电机固件版本不一致：

```text
Inconsistent motor version
```

运行中串口异常：

```text
Serial error
Reconnect start
Reconnect end
```

位置或力矩超限：

```text
Motor x exceed position upper limit.
Motor x exceed torque upper limit.
robot pos limit, motor stop.
robot torque limit, motor stop.
```

## 当前状态

当前工作区已经验证：

```bash
colcon build
colcon test --packages-select livelybot_serial
```

已经用单电机做过基础硬件联调：节点可识别串口、通讯板、电机版本，`/error_joint_states` 能随手动转动变化，`/low_cmd` 可驱动电机动作。扩展到多路 CAN 和多电机时，建议先从小数量配置开始，确认串口、CAN 通道、电机 ID、型号都能被识别，再逐步扩展到完整配置。
