import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    
    # === 配置路径 ===
    python_script_path = '/home/yahboom/stereo_info_publisher.py'
    left_yaml_path = '/home/yahboom/camera_config/left.yaml'
    right_yaml_path = '/home/yahboom/camera_config/right.yaml'

    # 1. 硬件配置 (v4l2-ctl)
    cmd_config_camera = [
        'v4l2-ctl', '-d', '/dev/video0', 
        '-c', 'auto_exposure=1',
        '-c', 'exposure_time_absolute=150', 
        '-c', 'white_balance_automatic=0',
        '-c', 'white_balance_temperature=4600',
        '-c', 'brightness=50'
    ]
    config_cam_action = ExecuteProcess(cmd=cmd_config_camera, output='screen')

    # 2. 摄像头节点
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        parameters=[{
            'video_device': '/dev/video0',
            'pixel_format': 'mjpeg2rgb',
            'image_width': 640,
            'image_height': 240,
            'framerate': 30.0,
            'camera_name': 'default_cam',
        }]
    )

    # 3. 裁剪节点
    left_cropper = Node(
        package='image_proc',
        executable='crop_decimate_node',
        name='left_cropper',
        parameters=[{'width': 320, 'height': 240, 'offset_x': 0, 'offset_y': 0}],
        remappings=[
            ('in/image_raw', '/image_raw'),
            ('out/image_raw', '/image_left'),
            # 我们不需要crop节点输出camera_info了，因为我们有专门的节点发
            ('out/camera_info', '/camera_info_left_unused') 
        ]
    )

    right_cropper = Node(
        package='image_proc',
        executable='crop_decimate_node',
        name='right_cropper',
        parameters=[{'width': 320, 'height': 240, 'offset_x': 320, 'offset_y': 0}],
        remappings=[
            ('in/image_raw', '/image_raw'),
            ('out/image_raw', '/image_right'),
            ('out/camera_info', '/camera_info_right_unused')
        ]
    )

    # 4. 标定信息发布节点 (直接运行 Python 脚本)
    # 这是一个小技巧，可以直接在launch里跑python脚本作为节点
    stereo_info_node = Node(
        package='lib', executable=python_script_path, # 这里需要稍微改一下，通常不能直接exec .py
        # 如果是临时测试，建议直接用 ExecuteProcess 运行 python3 script.py
        # 或者把上面的python脚本放到一个ros包里。
        # 这里为了演示方便，我们用 ExecuteProcess 替代 Node
    )
    
    # 更简单的运行 Python 脚本的方法:
    run_calibration_publisher = ExecuteProcess(
        cmd=['python3', python_script_path, 
             '--ros-args', 
             '-p', f'left_yaml_path:={left_yaml_path}',
             '-p', f'right_yaml_path:={right_yaml_path}'],
        output='screen'
    )

    return LaunchDescription([
        config_cam_action,
        TimerAction(period=1.0, actions=[usb_cam_node]),
        left_cropper,
        right_cropper,
        run_calibration_publisher
    ])
