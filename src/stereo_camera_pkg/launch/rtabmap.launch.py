import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 基础参数配置
    max_range = "3.5"  # 8cm基线+320分辨率，超过3.5米的点云完全不可信，强制滤除
    
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
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'approx_sync': True,             # 开启近似同步，容忍USB采集抖动
                'approx_sync_max_interval': 0.02,
                'wait_imu_to_init': False,
                
                # 里程计匹配优化
                'Vis/MinInliers': '15',          # 提高内点要求，防止乱跳
                'Vis/MaxDepth': max_range,       # 限制特征点深度
                'Vis/EstimationType': '1',       # 3D->3D
                'Vis/CorType': '0',              # Features matching
                'Stereo/OpticalFlow': 'True',    # 开启光流，极大稳定低分辨率下的特征跟踪
                'Odom/Strategy': '0',            # 0: Frame-to-Map
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
                'subscribe_stereo': True,
                'approx_sync': True,
                'approx_sync_max_interval': 0.02,
                
                # --- 深度过滤与点云清理 (解决散射的关键) ---
                'RangeMax': max_range,           # 全局截断，超过此距离的点直接丢弃
                'Grid/VoxelSize': '0.05',        # 体素化降采样，减小计算量并平滑点云
                'Grid/ClusterRadius': '0.1',     # 聚类半径
                'Grid/MinClusterSize': '20',     # 核心：孤立点簇少于20个点的全部删掉（解决散射点）
                'Grid/NoiseFilteringRadius': '0.1',
                'Grid/NoiseFilteringMinNeighbors': '5',
                
                # --- 立体匹配优化 ---
                'Stereo/Strategy': '0',          # 0: OpenCV Block Matching
                'Stereo/WindowSize': '21',       # 调大窗口。320分辨率下，大窗口能捕捉更多纹理，减少雪花噪点
                'Stereo/MaxDisparity': '64',     # 320x240分辨率下，视差通常不会很大
                
                # 地面分割
                'Grid/NormalsSegmentation': 'True',
                'Grid/MaxGroundHeight': '0.1',
                'Grid/MaxObstacleHeight': '2.0',
                
                'use_sim_time': False,
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