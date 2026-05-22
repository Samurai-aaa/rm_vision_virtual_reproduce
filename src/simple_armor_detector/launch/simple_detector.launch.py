import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("simple_armor_detector")

    config_file = os.path.join(
        pkg_dir,
        "config",
        "simple_detector.yaml"
    )

    simple_detector_node = Node(
        package="simple_armor_detector",
        executable="simple_detector_node",
        name="simple_armor_detector",
        output="screen",
        parameters=[config_file]
    )

    return LaunchDescription([
        simple_detector_node
    ])
