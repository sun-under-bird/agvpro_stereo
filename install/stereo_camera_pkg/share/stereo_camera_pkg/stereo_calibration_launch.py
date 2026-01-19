from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 启动camera_calibration工具，配置双目话题
    stereo_calibration_node = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        name='stereo_calibrator',
        output='screen',
        arguments=[
            '--size', '8x6',          # 标定板规格（内角点：行×列）
            '--square', '0.02',       # 标定板方格尺寸（单位：米，如20mm=0.02m）
            '--ros-args',
            '-r', '/left:=/left_camera',    # 左相机话题重映射（匹配你的话题前缀）
            '-r', '/right:=/right_camera',  # 右相机话题重映射
            '-p', 'right:=image_raw',       # 右相机图像话题名
            '-p', 'left:=image_raw'         # 左相机图像话题名
        ]
    )

    return LaunchDescription([stereo_calibration_node])