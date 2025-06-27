import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    use_respawn = LaunchConfiguration("use_respawn")
    params_file = LaunchConfiguration("params_file")

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="True",
        description="Whether to respawn if a node crashes",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(get_package_share_directory("ultrasonic_sensor"), "params", "config.yaml"),
        description="Full path to the ROS2 parameters file to use for node",
    )

    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_params_file_cmd)

    node = Node(
        package="ultrasonic_sensor",
        executable="ultrasonic_sensor_node",
        parameters=[
            params_file,
        ],
        respawn=use_respawn,
        respawn_delay=3.0,
        output="screen",
    )

    ld.add_action(node)

    return ld