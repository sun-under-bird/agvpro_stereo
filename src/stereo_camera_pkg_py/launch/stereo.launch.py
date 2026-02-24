import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    
    # === 1. 配置路径 ===
    left_yaml = '/home/yahboom/camera_ws/src/stereo_camera_pkg_py/config/left2.yaml'
    right_yaml = '/home/yahboom/camera_ws/src/stereo_camera_pkg_py/config/right2.yaml'
    python_script = '/home/yahboom/camera_ws/src/stereo_camera_pkg_py/stereo_camera_pkg_py/stereo_info.py'

    # === 2. 硬件配置 (锁曝光) ===
    cmd_config_camera = [
        'v4l2-ctl', '-d', '/dev/video0', 
        '-c', 'auto_exposure=1',
        '-c', 'exposure_time_absolute=1000', 
        '-c', 'white_balance_automatic=0',
        '-c', 'white_balance_temperature=4500',
        '-c', 'brightness=70'
    ]
    config_action = ExecuteProcess(cmd=cmd_config_camera, output='screen')

    # === 3. 启动 USB Cam (只负责取图) ===
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        parameters=[{
            'video_device': '/dev/video0',
            'pixel_format': 'mjpeg2rgb',
            'image_width': 1280,
            'image_height': 480,
            'framerate': 30.0,
            'camera_name': 'default_cam',
        }]
    )

    # === 4. 启动 Python 处理节点 (裁剪+发布Info) ===
    stereo_driver = ExecuteProcess(
        cmd=['python3', python_script, 
             '--ros-args', 
             '-p', f'left_yaml_path:={left_yaml}', 
             '-p', f'right_yaml_path:={right_yaml}'],
        output='screen'
    )

    return LaunchDescription([
        config_action,
        TimerAction(period=1.0, actions=[usb_cam_node]),
        TimerAction(period=2.0, actions=[stereo_driver]),
    ])
