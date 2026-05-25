import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("simple_armor_tracker")

    config_file = os.path.join(
        pkg_dir,
        "config",
        "simple_tracker.yaml"
    )

    tracker_node = Node(
        package="simple_armor_tracker",
        executable="simple_tracker_node",
        name="simple_armor_tracker",
        output="screen",
        parameters=[config_file]
    )

    return LaunchDescription([
        tracker_node
    ])