from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. 路径设置 (请核实你的 YAML 实际路径)
    pkg_stereo_image_proc = get_package_share_directory('stereo_image_proc')
    
    # 定义标定文件路径
    left_yaml_path = "/home/elephant/agvpro_stereo/src/stereo_camera_pkg/config/left5.yaml"
    right_yaml_path = "/home/elephant/agvpro_stereo/src/stereo_camera_pkg/config/right5.yaml"

    # 2. 启动参数
    base_frame = LaunchConfiguration('base_frame')
    use_viz = LaunchConfiguration('use_viz')
    approx_sync = LaunchConfiguration('approx_sync')
    baseline = LaunchConfiguration('baseline')

    # RTAB-Map 通用参数
    rtabmap_params = {
        'frame_id': base_frame,
        'subscribe_rgbd': False,
        'subscribe_stereo': True,
        'subscribe_odom_info': True,
        'approx_sync': approx_sync,
        'approx_sync_max_interval': 0.01,
        'sync_queue_size': 20,
        'topic_queue_size': 20,
        'Rtabmap/ImagesAlreadyRectified': 'true', 
        'Vis/EstimationType': '1', 
        'Vis/MinInliers': '15',
        'Reg/Force3DoF': 'true',
        'Stereo/MaxDisparity': '128',
    }

    # 话题映射
    remaps = [
        ('left/image_rect',   '/stereo/left/camera/image_rect'),
        ('right/image_rect',  '/stereo/right/camera/image_rect'),
        ('left/camera_info',  '/stereo/left/camera/camera_info'),
        ('right/camera_info', '/stereo/right/camera/camera_info'),
        ('odom',              '/vo'),
    ]

    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument('base_frame', default_value='camera_link'),
        DeclareLaunchArgument('use_viz', default_value='true'),
        DeclareLaunchArgument('approx_sync', default_value='false'),
        DeclareLaunchArgument('baseline', default_value='-0.0500'),

        # 3. 相机驱动节点
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': '/dev/video6',
                'image_width': 1280,
                'image_height': 480,
                'framerate': 20.0,
                'pixel_format': 'yuyv2rgb',
                'camera_name': 'stereo_camera'
            }]
        ),

        # 4. 图像拆分节点 (传入 YAML 路径，解决 Info 为 0 的问题)
        Node(
            package='stereo_camera_pkg',
            executable='stereo_split_node',
            name='stereo_split_node',
            output='screen',
            parameters=[{
                'left_info_url': left_yaml_path,
                'right_info_url': right_yaml_path,
                'input_topic': '/image_raw'
            }]
        ),

        # 5. 静态 TF 坐标变换
        # base_frame -> camera_link -> left_camera -> camera_left_frame (optical)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_left',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'left_camera']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_left_to_right',
            arguments=['0', baseline, '0', '0', '0', '0', 'left_camera', 'right_camera']
        ),
        # 旋转变换：将 ROS 坐标系 (X前Y左Z上) 转为 相机坐标系 (Z前X右Y下)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_left_optical',
            arguments=['0', '0', '0', '-1.570796', '0', '-1.570796', 'left_camera', 'camera_left_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_right_optical',
            arguments=['0', '0', '0', '-1.570796', '0', '-1.570796', 'right_camera', 'camera_right_frame']
        ),
        
        # 6. 图像校正节点 (将 raw 图转为 rect 图)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_stereo_image_proc, 'launch', 'stereo_image_proc.launch.py'])
            ),
            launch_arguments=[
                ('left_namespace',  '/stereo/left/camera'),
                ('right_namespace', '/stereo/right/camera'),
                ('approximate_sync', 'True'),
            ]
        ),

        # 7. RTAB-Map 里程计
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='screen',
            parameters=[rtabmap_params],
            remappings=remaps
        ),

        # 8. RTAB-Map SLAM 主节点
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_params],
            remappings=remaps,
            arguments=['-d']
        ),

        # 9. 可视化界面
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            condition=IfCondition(use_viz),
            parameters=[rtabmap_params],
            remappings=remaps
        ),
    ])