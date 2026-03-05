from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "redburi_arm", package_name="redburi_moveit"
    ).to_moveit_configs()
    launch_package_path = moveit_config.package_path

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(launch_package_path / "launch/static_virtual_joint_tfs.launch.py")
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(launch_package_path / "launch/rsp.launch.py")
                ),
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                parameters=[moveit_config.robot_description],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(launch_package_path / "launch/move_group.launch.py")
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(launch_package_path / "launch/moveit_rviz.launch.py")
                ),
            ),
        ]
    )
