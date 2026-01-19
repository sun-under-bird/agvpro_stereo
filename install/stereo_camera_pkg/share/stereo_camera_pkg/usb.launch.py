#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取usb_cam包的路径
    usb_cam_pkg = get_package_share_directory('usb_cam')
    
    # 左摄像头节点
    left_camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='left',
        output='screen',
        parameters=[
            {
                'video_device': '/dev/video1',
                'image_width': 320,
                'image_height': 240,
                'pixel_format': 'yuyv',
                'camera_frame_id': 'left',
                'camera_name': 'left',
                'io_method': 'mmap',
                'framerate': 30.0,
                'camera_info_url': '/home/yahboom/camera_ws/src/stereo_camera_pkg/config/left.yaml'
            }
        ],
        remappings=[
            ('/left/camera_info', '/stereo/left/camera_info'),
            ('/left/image_raw', '/stereo/left/image_raw')
        ]
    )
    
    # 右摄像头节点
    right_camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='right',
        output='screen',
        parameters=[
            {
                'video_device': '/dev/video2',
                'image_width': 320,
                'image_height': 240,
                'pixel_format': 'yuyv',
                'camera_frame_id': 'right',
                'camera_name': 'right',
                'io_method': 'mmap',
                'framerate': 30.0,
                'camera_info_url': '/home/yahboom/camera_ws/src/stereo_camera_pkg/config/right.yaml'
            }
        ],
        remappings=[
            ('/right/camera_info', '/stereo/right/camera_info'),
            ('/right/image_raw', '/stereo/right/image_raw')
        ]
    )
    
    # 可选的：如果需要image_view节点，可以取消注释并修改
    # image_view_node = Node(
    #     package='image_view',
    #     executable='image_view',
    #     name='image_view',
    #     parameters=[{'autosize': True}],
    #     remappings=[('image', '/usb_cam/image_raw')]
    # )
    
    return LaunchDescription([
        left_camera_node,
        right_camera_node,
        # image_view_node,  # 如果需要，取消注释
    ])