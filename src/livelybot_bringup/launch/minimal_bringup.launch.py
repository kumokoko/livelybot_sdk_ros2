from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motor_params = LaunchConfiguration("motor_params")
    power_params = LaunchConfiguration("power_params")
    imu_params = LaunchConfiguration("imu_params")
    oled_params = LaunchConfiguration("oled_params")
    logger_params = LaunchConfiguration("logger_params")
    enable_power = LaunchConfiguration("enable_power")
    enable_imu = LaunchConfiguration("enable_imu")
    enable_motor = LaunchConfiguration("enable_motor")
    enable_emergency_stop = LaunchConfiguration("enable_emergency_stop")
    enable_falldown_protect = LaunchConfiguration("enable_falldown_protect")
    enable_oled = LaunchConfiguration("enable_oled")
    enable_logger = LaunchConfiguration("enable_logger")
    falldown_config = LaunchConfiguration("falldown_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "motor_params",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("livelybot_bringup"), "cfg", "motor_params.minimal.yaml"]
                ),
                description="Motor driver parameter file",
            ),
            DeclareLaunchArgument(
                "power_params",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("livelybot_bringup"), "cfg", "power_params.yaml"]
                ),
                description="Power driver parameter file",
            ),
            DeclareLaunchArgument(
                "imu_params",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("livelybot_bringup"), "cfg", "yesense_params.yaml"]
                ),
                description="IMU driver parameter file",
            ),
            DeclareLaunchArgument(
                "oled_params",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("livelybot_bringup"), "cfg", "oled_params.yaml"]
                ),
                description="OLED bridge parameter file",
            ),
            DeclareLaunchArgument(
                "logger_params",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("livelybot_bringup"), "cfg", "logger_params.yaml"]
                ),
                description="Logger parameter file",
            ),
            DeclareLaunchArgument(
                "enable_power",
                default_value="true",
                description="Start the power driver node",
            ),
            DeclareLaunchArgument(
                "enable_imu",
                default_value="true",
                description="Start the IMU driver node",
            ),
            DeclareLaunchArgument(
                "enable_motor",
                default_value="true",
                description="Start the motor driver node",
            ),
            DeclareLaunchArgument(
                "enable_emergency_stop",
                default_value="false",
                description="Start the controller emergency stop node",
            ),
            DeclareLaunchArgument(
                "enable_falldown_protect",
                default_value="false",
                description="Start the IMU falldown protection node",
            ),
            DeclareLaunchArgument(
                "falldown_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("livelybot_bringup"), "cfg", "falldown_condition_pi.yaml"]
                ),
                description="Falldown protection threshold config file",
            ),
            DeclareLaunchArgument(
                "enable_oled",
                default_value="false",
                description="Start the OLED bridge node",
            ),
            DeclareLaunchArgument(
                "enable_logger",
                default_value="false",
                description="Start the logger node",
            ),
            Node(
                package="livelybot_power",
                executable="power_node",
                name="power_node",
                output="screen",
                parameters=[power_params],
                condition=IfCondition(enable_power),
            ),
            Node(
                package="yesense_imu",
                executable="yesense_imu_node",
                name="yesense_imu",
                output="screen",
                parameters=[imu_params],
                condition=IfCondition(enable_imu),
            ),
            Node(
                package="livelybot_serial",
                executable="motor_driver_node",
                name="motor_driver_node",
                output="screen",
                parameters=[motor_params],
                condition=IfCondition(enable_motor),
            ),
            Node(
                package="livelybot_bringup",
                executable="robot_emergency_stop_node",
                name="robot_emergency_stop",
                output="screen",
                condition=IfCondition(enable_emergency_stop),
            ),
            Node(
                package="livelybot_bringup",
                executable="robot_falldown_protect_node",
                name="robot_falldown_protect",
                output="screen",
                condition=IfCondition(enable_falldown_protect),
                parameters=[{"config_path": falldown_config}],
            ),
            Node(
                package="livelybot_oled",
                executable="livelybot_oled_node",
                name="livelybot_oled",
                output="screen",
                condition=IfCondition(enable_oled),
                parameters=[oled_params],
            ),
            Node(
                package="livelybot_logger",
                executable="livelybot_logger_node",
                name="livelybot_logger",
                output="screen",
                condition=IfCondition(enable_logger),
                parameters=[logger_params],
            ),
        ]
    )
