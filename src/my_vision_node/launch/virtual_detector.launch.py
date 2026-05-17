# virtual_detector.launch.py
# 这个 launch 文件用于一键启动无硬件复现链路：
# 1. 虚拟相机节点：读取摄像头或视频，发布 /image_raw
# 2. camera_info 节点：发布 /camera_info，提供 PnP 所需相机内参
# 3. armor_detector 节点：订阅图像并进行装甲板检测

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # 获取 my_vision_node 包安装后的 share 目录
    my_vision_share = get_package_share_directory('my_vision_node')

    config_file = os.path.join(
        my_vision_share,
        'config',
        'virtual_camera.yaml'
    )

    # 启动虚拟相机节点
    #
    # 作用：
    # - 使用 OpenCV 读取摄像头或视频
    # - 使用 cv_bridge 转成 ROS 2 Image
    # - 发布 /image_raw
    virtual_camera_node = Node(
        package='my_vision_node',
        executable='vision_node',
        name='virtual_camera_node',
        output='screen',
        parameters=[config_file]
    )

    # 启动虚拟 camera_info 节点
    #
    # 作用：
    # - 发布 /camera_info
    # - 提供 fx, fy, cx, cy 等相机内参
    # - 供 armor_detector 初始化 PnP Solver
    camera_info_node = Node(
        package='my_vision_node',
        executable='camera_info_node',
        name='virtual_camera_info_node',
        output='screen',
        parameters=[config_file]
    )

    # 启动官方 armor_detector 节点
    #
    # 作用：
    # - 订阅 /image_raw
    # - 使用 /camera_info 初始化 PnP
    # - 检测灯条和装甲板
    # - 发布 /detector/result_img、/detector/binary_img、/detector/armors
    #
    # debug=True 表示开启调试图像输出
    armor_detector_node = Node(
        package='armor_detector', 
        executable='armor_detector_node',
        name='armor_detector',
        output='screen',
        parameters=[{'debug': True}]
    )

    # 返回需要启动的所有节点
    # ros2 launch 执行这个文件时，会按照这里的描述启动三个节点
    return LaunchDescription([
        virtual_camera_node,
        camera_info_node,
        armor_detector_node
    ])