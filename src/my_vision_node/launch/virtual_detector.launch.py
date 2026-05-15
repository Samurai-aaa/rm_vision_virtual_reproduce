from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    my_vision_share = get_package_share_directory('my_vision_node')
    config_file = os.path.join(
        my_vision_share,
        'config',
        'virtual_camera.yaml'
    )

    virtual_camera_node = Node(
        package='my_vision_node',
        executable='vision_node',
        name='virtual_camera_node',
        output='screen',
        parameters=[config_file]
    )

    camera_info_node = Node(
        package='my_vision_node',
        executable='camera_info_node',
        name='virtual_camera_info_node',
        output='screen',
        parameters=[config_file]
    )

    armor_detector_node = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='armor_detector',
        output='screen',
        parameters=[{'debug': True}]
    )

    return LaunchDescription([
        virtual_camera_node,
        camera_info_node,
        armor_detector_node
    ])
