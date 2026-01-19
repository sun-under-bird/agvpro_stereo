# launch_stereo_rtabmap.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # # 1. 启动相机驱动（这里以模拟的相机驱动为例）
    # camera_driver = Node(
    #     package='usb_cam',
    #     executable='usb_cam_node_exe',
    #     name='stereo_camera',
    #     namespace='my_stereo',
    #     parameters=[{
    #         'video_device': '/dev/video0',
    #         'image_width': 640,
    #         'image_height': 480,
    #         'pixel_format': 'yuyv2rgb',
    #         'camera_name': 'stereo_camera',
    #         'camera_info_url': 'file:///path/to/your/calibration.yaml',
    #     }]
    # )
    
    # # 2. 启动图像校正节点
    # left_image_proc = Node(
    #     package='image_proc',
    #     executable='image_proc',
    #     name='left_image_proc',
    #     namespace='my_stereo/left',
    #     remappings=[
    #         ('image', 'image_raw'),
    #         ('image_rect', 'image_rect'),
    #     ]
    # )
    
    # right_image_proc = Node(
    #     package='image_proc',
    #     executable='image_proc',
    #     name='right_image_proc',
    #     namespace='my_stereo/right',
    #     remappings=[
    #         ('image', 'image_raw'),
    #         ('image_rect', 'image_rect'),
    #     ]
    # )
    
    # 3. 启动RTAB-Map
    rtabmap = Node(
        package='rtabmap',
        executable='rtabmap_slame',
        name='rtabmap',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_stereo': True,
            'approx_sync': True,
            'queue_size': 10,
            'Rtabmap/DetectionRate': '3',
            'Mem/STMSize': '30',
            'Kp/MaxFeatures': '400',
        }],
        remappings=[
            ('left/image_rect', '/left_camera/image_rect'),
            ('right/image_rect', '/right_camera/image_rect'),
            ('left/camera_info', '/left_camera/camera_info'),
            ('right/camera_info', '/right_camera/camera_info'),
            ('odom', '/odom'),
        ],
        arguments=['-d']
    )
    
    # 4. 启动可视化
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        parameters=[{
            'frame_id': 'base_link',
        }]
    )
    
    return LaunchDescription([
        # camera_driver,
        # left_image_proc,
        # right_image_proc,
        rtabmap,
        rtabmap_viz,
    ])