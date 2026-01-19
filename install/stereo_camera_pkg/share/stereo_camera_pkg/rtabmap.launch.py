import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    parameters=[{
        'frame_id':'left_camera',          
        'subscribe_stereo':True,           
        'subscribe_odom_info':False,     
        'wait_imu_to_init':False,         
        'approx_sync':True,               
        'approx_sync_max_interval':0.2,    
        'subscribe_depth': False,
        'subscribe_rgb': False,   
        'use_sim_time': False,
        'publish_tf':True,
        # 'odom_frame_id':'odom',
        'min_inliers':1,               
        'max_disparity':256,
        'stereo':True,                     # 显式强制开启双目模式（优先级最高）
        'rgbd':False,            
    }]


    remappings=[
          ('left/image_rect', '/left_camera/image_rect'),  
          ('left/camera_info', '/left_camera/camera_info'), 
          ('right/image_rect', '/right_camera/image_rect'),  
          ('right/camera_info', '/right_camera/camera_info') 
    ]

    return LaunchDescription([

        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),
        

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_right_camera_tf',
            arguments=[
                '-0.0828', '0', '0', 
                '0', '0', '0',   
                'left_camera',     
                'right_camera'    
            ]
        ),

        #  Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='odom_to_base_link_tf',
        #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'left_camera']
        # ),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),
                

    ])