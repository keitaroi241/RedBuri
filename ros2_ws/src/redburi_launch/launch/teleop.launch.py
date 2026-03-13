from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    joy_dev = LaunchConfiguration("joy_dev")
    joy_deadzone = LaunchConfiguration("joy_deadzone")
    joy_autorepeat_rate = LaunchConfiguration("joy_autorepeat_rate")

    moveit_config = MoveItConfigsBuilder(
        "redburi_arm", package_name="redburi_moveit"
    ).to_moveit_configs()
    moveit_share = FindPackageShare("redburi_moveit")
    teleop_share = FindPackageShare("teleop")
    serial_bridge_share = FindPackageShare("serial_bridge")

    static_tf_launch = PathJoinSubstitution(
        [moveit_share, "launch", "static_virtual_joint_tfs.launch.py"]
    )
    rsp_launch = PathJoinSubstitution(
        [moveit_share, "launch", "rsp.launch.py"]
    )
    move_group_launch = PathJoinSubstitution(
        [moveit_share, "launch", "move_group.launch.py"]
    )
    moveit_rviz_launch = PathJoinSubstitution(
        [moveit_share, "launch", "moveit_rviz.launch.py"]
    )

    joy_base_param = PathJoinSubstitution(
        [teleop_share, "config", "joy_base_node.yaml"]
    )
    joy_arm_joint_param = PathJoinSubstitution(
        [teleop_share, "config", "joy_arm_joint_node.yaml"]
    )
    joy_arm_cartesian_param = PathJoinSubstitution(
        [teleop_share, "config", "joy_arm_cartesian_node.yaml"]
    )
    servo_arm_motor_param = PathJoinSubstitution(
        [teleop_share, "config", "servo_arm_motor_node.yaml"]
    )
    servo_param = PathJoinSubstitution(
        [teleop_share, "config", "servo.yaml"]
    )
    serial_bridge_param = PathJoinSubstitution(
        [serial_bridge_share, "config", "serial_bridge.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("joy_deadzone", default_value="0.1"),
            DeclareLaunchArgument("joy_autorepeat_rate", default_value="60.0"),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(static_tf_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(rsp_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(move_group_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(moveit_rviz_launch)),
            Node(
                package="moveit_servo",
                executable="servo_node_main",
                name="servo_node",
                output="screen",
                parameters=[
                    servo_param,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "service",
                            "call",
                            "/servo_node/start_servo",
                            "std_srvs/srv/Trigger",
                            "{}",
                        ],
                        output="screen",
                    )
                ],
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[
                    {
                        "dev": joy_dev,
                        "deadzone": joy_deadzone,
                        "autorepeat_rate": joy_autorepeat_rate,
                    }
                ],
            ),
            Node(
                package="teleop",
                executable="joy_mode_node",
                name="joy_mode_node",
                output="screen",
            ),
            Node(
                package="teleop",
                executable="joy_base_node",
                name="joy_base_node",
                output="screen",
                parameters=[joy_base_param],
            ),
            Node(
                package="teleop",
                executable="joy_arm_joint_node",
                name="joy_arm_joint_node",
                output="screen",
                parameters=[joy_arm_joint_param],
            ),
            Node(
                package="teleop",
                executable="joy_arm_cartesian_node",
                name="joy_arm_cartesian_node",
                output="screen",
                parameters=[joy_arm_cartesian_param],
            ),
            Node(
                package="teleop",
                executable="servo_arm_motor_node",
                name="servo_arm_motor_node",
                output="screen",
                parameters=[servo_arm_motor_param],
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
