#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bag_path = LaunchConfiguration('bag_path')
    play_rate = LaunchConfiguration('play_rate')

    # 标定文件路径（传给 stereo_info.py 的 ROS 参数）
    left_yaml = LaunchConfiguration('left_yaml')
    right_yaml = LaunchConfiguration('right_yaml')

    # stereo_info.py 的路径
    stereo_script = LaunchConfiguration('stereo_script')

    # 1) 播放 bag（假设 bag 里有 /image_raw/compressed）
    play_bag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            bag_path,
            '--rate', play_rate,
            # '--loop',   # 需要循环就取消注释
        ],
        output='screen'
    )

    # 2) 解码 compressed -> raw
    # 输入：/image_raw/compressed  (sensor_msgs/CompressedImage)
    # 输出：/image_raw            (sensor_msgs/Image)
    decompress = Node(
        package='image_transport',
        executable='republish',
        name='republish_mjpeg_to_raw',
        output='screen',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/image_raw/compressed'),
            ('out', '/image_raw'),
        ],
    )

    # 3) 运行你的分割脚本 stereo_info.py
    # 注意：你的脚本用的是 ROS 参数 left_yaml_path/right_yaml_path
    # 所以必须加 --ros-args，并用 -p 传参
    split_and_info = ExecuteProcess(
        cmd=[
            'python3', stereo_script,
            '--ros-args',
            '-p', left_yaml.perform(None) if False else 'left_yaml_path:=' + str(left_yaml),
            '-p', right_yaml.perform(None) if False else 'right_yaml_path:=' + str(right_yaml),
        ],
        output='screen'
    )

    # 上面那种拼接在 Launch 里会导致 LaunchConfiguration 变成字符串对象（坑）
    # ✅ 正确方式：把 substitution 当作独立元素传递（见下面“最终正确 cmd”）
    split_and_info = ExecuteProcess(
        cmd=[
            'python3', stereo_script,
            '--ros-args',
            '-p', ['left_yaml_path:=', left_yaml],
            '-p', ['right_yaml_path:=', right_yaml],
        ],
        output='screen'
    )

    # 延迟启动，确保 bag play 和 republish 先起来
    delayed_split = TimerAction(period=1.0, actions=[split_and_info])

    return LaunchDescription([
        DeclareLaunchArgument('bag_path', default_value='/home/yahboom/rosbag2_2026_02_09-15_54_55/rosbag2_2026_02_09-15_54_55_0.db3'),
        DeclareLaunchArgument('play_rate', default_value='1.0'),

        DeclareLaunchArgument(
            'left_yaml',
            default_value='/home/yahboom/camera_ws/src/stereo_camera_pkg_py/config/left1.yaml'
        ),
        DeclareLaunchArgument(
            'right_yaml',
            default_value='/home/yahboom/camera_ws/src/stereo_camera_pkg_py/config/right1.yaml'
        ),
        DeclareLaunchArgument(
            'stereo_script',
            default_value='/home/yahboom/camera_ws/src/stereo_camera_pkg_py/stereo_camera_pkg_py/stereo_info.py'
        ),

        play_bag,
        decompress,
        delayed_split,
    ])

