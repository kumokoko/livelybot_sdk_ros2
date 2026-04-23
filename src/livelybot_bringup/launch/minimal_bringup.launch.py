from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motor_params = LaunchConfiguration("motor_params")
    power_params = LaunchConfiguration("power_params")
    imu_params = LaunchConfiguration("imu_params")

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
            Node(
                package="livelybot_power",
                executable="power_node",
                name="power_node",
                output="screen",
                parameters=[power_params],
            ),
            Node(
                package="yesense_imu",
                executable="yesense_imu_node",
                name="yesense_imu",
                output="screen",
                parameters=[imu_params],
            ),
            Node(
                package="livelybot_serial",
                executable="motor_driver_node",
                name="motor_driver_node",
                output="screen",
                parameters=[motor_params],
            ),
        ]
    )
