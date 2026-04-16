import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        # 1. 静态 TF 发布 (请根据你相机的实际位置调整)
        # 左右相机基线约为 0.08192m。注意：在ROS中，右目通常在左目的 Y 轴负方向。
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
            arguments=['0', '-0.08192', '0', '0', '0', '0', '1', 'left_camera', 'right_camera']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_left_optical',
            arguments=['0', '0', '0', '-1.570796', '0', '-1.570796', 'left_camera', 'left_camera_optical']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_right_optical',
            arguments=['0', '0', '0', '-1.570796', '0', '-1.570796', 'right_camera', 'right_camera_optical']
        ),

        # 2. 立体里程计 (Stereo Odometry)
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'subscribe_rgbd': False,
                'subscribe_stereo': True,
                'subscribe_odom_info': True,
                'use_sim_time': False,
                'approx_sync': True,
                'approx_sync_max_interval': 0.1,
                'sync_queue_size': 30,
                'topic_queue_size': 30,
                'wait_for_transform': 0.2,
                'tf_delay': 0.05,
                'Rtabmap/ImagesAlreadyRectified': 'true',
            
                'Vis/EstimationType': '1', 
                'Vis/MinInliers': '8',
                'Vis/MaxFeatures': '1000',
                'Vis/MaxDepth':'3',
                'Vis/CorType': '0',

                'Odom/Strategy': '0', 
                'OdomF2M/MaxSize': '1000',
                'Odom/KeyFrameThr': '0.5',
                'Odom/ScanKeyFrameThr': '0.5',

                'GFTT/MinDistance': '10',
                'GFTT/QualityLevel': '0.00001',
                'GFTT/MaxCorners': '500',

                'Stereo/MaxDisparity':'128',
                'Stereo/MinDisparity':'3',
            }],
            remappings=[
                ('left/image_rect', '/left_camera/image_rect'),
                ('right/image_rect', '/right_camera/image_rect'),
                ('left/camera_info', '/left_camera/camera_info'),
                ('right/camera_info', '/right_camera/camera_info'),
            ]
        ),

        # 3. RTAB-Map 建图主节点
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'subscribe_rgbd': False,
                'subscribe_stereo': True,
                'subscribe_odom_info': True,
                'use_sim_time': False,
                'approx_sync': True,
                'approx_sync_max_interval': 0.1,
                'sync_queue_size': 30,
                'topic_queue_size': 30,
                'wait_for_transform': 0.2,
                'tf_delay': 0.05,

                'Rtabmap/ImagesAlreadyRectified': 'true',
                'Rtabmap/DetectionRate': '3',
                'Reg/Force3DoF': 'false',

                'Kp/MaxFeatures': '600',
                'Kp/NndrRatio': '0.75',

                'GFTT/MinDistance': '10',
                'GFTT/QualityLevel': '0.00001',
                'GFTT/MaxCorners': '500',

                'Stereo/MaxDisparity':'128',
            

                'Grid/CellSize':'0.05',
                'Grid/3D':'true',
                'Grid/GroundIsObstacle':'false',
                'Grid/RangeMax':'3',
                'Grid/MaxObstacleHeight':'1',

                'cloud_decimation':'4',
                'cloud_max_depth':'3',
                'cloud_min_depth':'0.3',
                'cloud_voxel_size':'0.05',

                'Vis/MaxDepth':'3',
            }],
            remappings=[
                ('left/image_rect', '/left_camera/image_rect'),
                ('right/image_rect', '/right_camera/image_rect'),
                ('left/camera_info', '/left_camera/camera_info'),
                ('right/camera_info', '/right_camera/camera_info'),
            ],
            arguments=['--delete_db_on_start']
        ),

        # 4. 可视化界面 (rtabmap_viz)
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'subscribe_stereo': True,
                'frame_id': 'camera_link',
                'approx_sync': True,
            }],
            remappings=[
                ('left/image_rect', '/left_camera/image_rect'),
                ('right/image_rect', '/right_camera/image_rect'),
                ('left/camera_info', '/left_camera/camera_info'),
                ('right/camera_info', '/right_camera/camera_info'),
            ]
        ),
    ])