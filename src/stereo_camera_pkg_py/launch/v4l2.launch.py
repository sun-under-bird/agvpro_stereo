from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    left_yaml  = '/home/yahboom/camera_ws/src/stereo_camera_pkg_py/config/left.yaml'
    right_yaml = '/home/yahboom/camera_ws/src/stereo_camera_pkg_py/config/right.yaml'

    return LaunchDescription([
        Node(
            package='stereo_camera_pkg_py',
            executable='v4l2',
            name='v4l2',
            output='screen',
            parameters=[{
                'left_device': '/dev/video10',
                'right_device': '/dev/video11',
                'width': 640,
                'height': 480,
                'fps': 30.0,

                # 如果你 v4l2-ctl --get-fmt-video 看到是 MJPG，就改成 'MJPG'
                'fourcc': 'YUYV',

                'left_calib_yaml': left_yaml,
                'right_calib_yaml': right_yaml,

                'left_frame_id': 'stereo_left',
                'right_frame_id': 'stereo_right',

                'left_image_topic': '/stereo/left/image_raw',
                'right_image_topic': '/stereo/right/image_raw',
                'left_info_topic': '/stereo/left/camera_info',
                'right_info_topic': '/stereo/right/camera_info',

                'drop_old_frames': True,
                'grab_warmup': 2,
            }],
        )
    ])

