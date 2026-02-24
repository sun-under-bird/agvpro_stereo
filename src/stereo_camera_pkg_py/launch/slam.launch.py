from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    base_frame = LaunchConfiguration('base_frame')
    use_viz = LaunchConfiguration('use_viz')
    approx_sync = LaunchConfiguration('approx_sync')
    baseline = LaunchConfiguration('baseline')

    rtabmap_slam_params = {
        'frame_id': base_frame,
        'subscribe_rgbd': False,
        'subscribe_stereo': True,
        'subscribe_odom_info': True,
        'use_sim_time': False,
        'approx_sync': approx_sync,
        'approx_sync_max_interval': 0.02,
        'queue_size': 5,
        'sync_queue_size': 5,
        'wait_for_transform': 0.2,
        'tf_delay': 0.05,
        'Rtabmap/ImagesAlreadyRectified': 'true',
        'Rtabmap/DetectionRate': '2',
        'Reg/Force3DoF': 'false',
        'Kp/MaxFeatures': '700',
        'Kp/NndrRatio': '0.8',
        'GFTT/MinDistance': '10',
        'GFTT/QualityLevel': '0.05',
        'GFTT/MaxCorners': '700',
        'Stereo/MaxDisparity': '128',
        'Vis/MaxDepth': '4',
        'Kp/MaxDepth': '4',
        'Grid/RangeMax': '4',
        'Vis/MinInliers': '8',
    }

    rtabmap_odom_params = {
        'frame_id': base_frame,
        'subscribe_rgbd': False,
        'subscribe_stereo': True,
        'subscribe_odom_info': True,
        'use_sim_time': False,
        'approx_sync': approx_sync,
        'approx_sync_max_interval': 0.02,
        'sync_queue_size': 5,
        'wait_for_transform': 0.2,
        'tf_delay': 0.05,
        'Rtabmap/ImagesAlreadyRectified': 'true',
        'Odom/Strategy': '0',
        'Vis/EstimationType': '1',
        'Vis/MinInliers': '6',
        'Vis/MaxFeatures': '700',
        'OdomF2M/MaxSize': '700',
        'GFTT/MinDistance': '10',
        'GFTT/QualityLevel': '0.05',
        'GFTT/MaxCorners': '700',
        'Stereo/MaxDisparity': '128',
    }

    
    remaps = [
        ('left/image_rect',   '/stereo/left/camera/image_rect_color'),
        ('right/image_rect',  '/stereo/right/camera/image_rect_color'),
        ('left/camera_info',  '/stereo/left/camera/camera_info'),
        ('right/camera_info', '/stereo/right/camera/camera_info'),
        ('odom',              '/vo'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='camera_link'),
        DeclareLaunchArgument('use_viz', default_value='true'),
        DeclareLaunchArgument('approx_sync', default_value='true'),
        DeclareLaunchArgument('baseline', default_value='-0.0635'), 

        # TF
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
            arguments=['0', baseline, '0', '0', '0', '0', '1', 'camera_link', 'right_camera']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_left_optical',
            arguments=['0', '0', '0', '-1.570796', '0', '-1.570796',
                       'left_camera', 'camera_left_frame']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_right_optical',
            arguments=['0', '0', '0', '-1.570796', '0', '-1.570796',
                       'right_camera', 'camera_right_frame']
        ),

        # stereo_odometry
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='screen',
            parameters=[rtabmap_odom_params],
            remappings=remaps
        ),

        # rtabmap
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{"odometry_node_name": 'stereo_odometry'},rtabmap_slam_params],
            remappings=remaps,
            arguments=['-d']
        ),

        # rtabmap_viz
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            condition=IfCondition(use_viz),
            parameters=[rtabmap_slam_params, {'odometry_node_name': 'stereo_odometry'}],
            remappings=remaps
        ),
        
 
    ])

