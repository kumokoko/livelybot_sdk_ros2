from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motor_params = LaunchConfiguration("motor_params")
    logger_params = LaunchConfiguration("logger_params")
    enable_motor = LaunchConfiguration("enable_motor")
    enable_logger = LaunchConfiguration("enable_logger")

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
                "logger_params",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("livelybot_bringup"), "cfg", "logger_params.yaml"]
                ),
                description="Logger parameter file",
            ),
            DeclareLaunchArgument(
                "enable_motor",
                default_value="true",
                description="Start the motor driver node",
            ),
            DeclareLaunchArgument(
                "enable_logger",
                default_value="false",
                description="Start the logger node",
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
                package="livelybot_logger",
                executable="livelybot_logger_node",
                name="livelybot_logger",
                output="screen",
                condition=IfCondition(enable_logger),
                parameters=[logger_params],
            ),
        ]
    )
