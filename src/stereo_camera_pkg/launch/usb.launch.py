from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter, SetRemap
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_stereo_image_proc = get_package_share_directory(
    'stereo_image_proc')

    # Paths
    stereo_image_proc_launch = PathJoinSubstitution(
        [pkg_stereo_image_proc, 'launch', 'stereo_image_proc.launch.py'])
    parameters={
          'frame_id':'camera_link',
          'subscribe_rgbd':True,
          'approx_sync':False, # odom is generated from images, so we can exactly sync all inputs
          'map_negative_poses_ignored':True,
          'subscribe_odom_info': True,
          # RTAB-Map's internal parameters should be strings
          'OdomF2M/MaxSize': '1000',
          'GFTT/MinDistance': '10',
          'GFTT/QualityLevel': '0.00001',
          'use_sim_time':False
          #'Kp/DetectorStrategy': '6', # Uncommment to match ros1 noetic results, but opencv should be built with xfeatures2d
          #'Vis/FeatureType': '6'      # Uncommment to match ros1 noetic results, but opencv should be built with xfeatures2d
    }

    remappings=[
         ('rgbd_image', '/stereo_camera/rgbd_image'),
         ('odom',       '/vo')]
    

    return LaunchDescription([

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
            arguments=['0', '-0.08192', '0', '0', '0', '0', '1', 'camera_link', 'right_camera']
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
        GroupAction(
            actions=[

                SetRemap(src='camera_info',dst='camera_info'),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([stereo_image_proc_launch]),
                    launch_arguments=[
                        ('left_namespace', '/left_camera'),
                        ('right_namespace', '/right_camera'),
                        ('disparity_range', '128'),
                    ]
                ),
            ]
        ),

        Node(
            package='rtabmap_sync', executable='stereo_sync', output='screen',
            namespace='stereo_camera',
            remappings=[
                ('left/image_rect',   '/left_camera/image_rect'),
                ('right/image_rect',  '/right_camera/image_rect'),
                ('left/camera_info',  '/left_camera/camera_info'),
                ('right/camera_info', '/right_camera/camera_info')]
        ),  
        # 2. 立体里程计 (Stereo Odometry)
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='screen',
            parameters=[parameters],
            remappings=remappings
        ),

        # 3. RTAB-Map 建图主节点
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']
        ),

        # 4. 可视化界面 (rtabmap_viz)
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[parameters,{'odometry_node_name':'stereo_odometry'}],
            remappings=remappings
        ),
    ])