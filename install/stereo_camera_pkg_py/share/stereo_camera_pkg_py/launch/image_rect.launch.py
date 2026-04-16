from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    container = ComposableNodeContainer(
        name='rectify_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[

            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='left_rectify',
                namespace='stereo/left',
                remappings=[
                    ('image', '/image_raw'),
                    ('camera_info', '/camera_info'),
                    ('image_rect', '/image_rect')
                ]
            ),

            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='right_rectify',
                namespace='stereo/right',
                remappings=[
                    ('image', '/image_raw'),
                    ('camera_info', '/camera_info'),
                    ('image_rect', '/image_rect')
                ]
            ),

        ],
        output='screen',
    )

    return LaunchDescription([container])
