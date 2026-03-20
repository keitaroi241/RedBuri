from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_bridge_share = FindPackageShare("serial_bridge")
    serial_bridge_param_default = PathJoinSubstitution(
        [serial_bridge_share, "config", "serial_bridge.yaml"]
    )

    serial_bridge_param = LaunchConfiguration("serial_bridge_param")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_bridge_param",
                default_value=serial_bridge_param_default,
                description="Path to serial_bridge parameter YAML",
            ),
            Node(
                package="serial_bridge",
                executable="serial_bridge_node",
                name="serial_bridge_node",
                output="screen",
                parameters=[serial_bridge_param],
            ),
        ]
    )
