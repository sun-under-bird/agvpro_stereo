from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    baseline = 0.08192

    left_image_proc = Node(
        package='image_proc',
        executable='image_proc',
        name='left_image_proc',
        output='screen',
        remappings=[
            ('image',       '/left_camera/image_raw'),
            ('camera_info', '/left_camera/camera_info'),
            ('image_rect',  '/left_camera/image_rect'),
        ],
        parameters=[{
            'use_transport_compressed': False,
        }]
    )

    right_image_proc = Node(
        package='image_proc',
        executable='image_proc',
        name='right_image_proc',
        output='screen',
        remappings=[
            ('image',       '/right_camera/image_raw'),
            ('camera_info', '/right_camera/camera_info'),
            ('image_rect',  '/right_camera/image_rect'),
        ],
        parameters=[{
            'use_transport_compressed': False,
        }]
    )

    disparity = Node(
        package='stereo_image_proc',
        executable='disparity_node',
        name='disparity_node',
        output='screen',
        remappings=[
            ('left/image_rect',   '/left_camera/image_rect'),
            ('left/camera_info',  '/left_camera/camera_info'),
            ('right/image_rect',  '/right_camera/image_rect'),
            ('right/camera_info', '/right_camera/camera_info'),
            ('disparity',         '/stereo/disparity'),
        ]
    )

    points2 = Node(
        package='stereo_image_proc',
        executable='point_cloud_node',
        name='point_cloud_node',
        output='screen',
        remappings=[
            ('left/camera_info',  '/left_camera/camera_info'),
            ('right/camera_info', '/right_camera/camera_info'),
            ('disparity',         '/stereo/disparity'),
            ('points2',           '/stereo/points2'),
        ]
    )

    tf_cam_link_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_cam_link_left',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'camera_link', 'left_camera']
    )

    tf_cam_link_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_cam_link_right',
        arguments=[str(baseline), '0', '0', '0', '0', '0', '1', 'camera_link', 'right_camera']
    )

    tf_left_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_left_optical',
        arguments=['0', '0', '0', '-1.5707963', '0', '-1.5707963', 'left_camera', 'left_camera_optical']
    )

    tf_right_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_right_optical',
        arguments=['0', '0', '0', '-1.5707963', '0', '-1.5707963', 'right_camera', 'right_camera_optical']
    )

    return LaunchDescription([
        tf_cam_link_left,
        tf_cam_link_right,
        tf_left_optical,
        tf_right_optical,
        left_image_proc,
        right_image_proc,
        disparity,
        points2,
    ])
