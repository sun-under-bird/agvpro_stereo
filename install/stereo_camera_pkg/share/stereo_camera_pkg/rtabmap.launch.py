import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    left_camera_name = "left_camera"
    right_camera_name = "right_camera"

    return LaunchDescription([
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                # 'odom_frame_id': 'odom',
                'publish_tf': True,

                'approx_sync': True,
                'approx_sync_max_interval': 0.04,
                'sync_queue_size': 30,

                'wait_imu_to_init': False,

                'Odom/MinInliers': '12',
                'Vis/MinInliers': '12',
                'Stereo/MaxDisparity': '256',
            }],
            remappings=[
                ('left/image_rect', '/left_camera/image_rect'),
                ('left/camera_info', '/left_camera/camera_info'),
                ('right/image_rect', '/right_camera/image_rect'),
                ('right/camera_info', '/right_camera/camera_info')
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_cam_link_left',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'camera_link', 'left_camera']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_cam_link_right',
            arguments=['0.08192', '0', '0', '0', '0', '0', '1', 'camera_link', 'right_camera']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_left_frame_optical',
            arguments=['0', '0', '0', '-1.5707963', '0', '-1.5707963', 'left_camera', 'left_camera_optical']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_right_frame_optical',
            arguments=['0', '0', '0', '-1.5707963', '0', '-1.5707963', 'right_camera', 'right_camera_optical']
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'subscribe_stereo': True,
                'approx_sync': True,
                'approx_sync_max_interval': 0.04,
                'sync_queue_size': 30,
                'publish_tf': True,

                'Vis/MinInliers': '12',
            }],
            remappings=[
                ('left/image_rect', '/left_camera/image_rect'),
                ('left/camera_info', '/left_camera/camera_info'),
                ('right/image_rect', '/right_camera/image_rect'),
                ('right/camera_info', '/right_camera/camera_info'),
                # ('odom', '/stereo_odometry/odom'),
            ],
            arguments=['--delete_db_on_start']
        ),

        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'subscribe_stereo': True,
                'approx_sync': True,
                'approx_sync_max_interval': 0.04,
                'sync_queue_size': 30,

                'publish_tf': False,
            }],
            remappings=[
                ('left/image_rect', '/left_camera/image_rect'),
                ('left/camera_info', '/left_camera/camera_info'),
                ('right/image_rect', '/right_camera/image_rect'),
                ('right/camera_info', '/right_camera/camera_info'),
                # ('odom', '/stereo_odometry/odom'),
            ]
        ),
    ])
