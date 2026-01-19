from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # ========== 1. 声明可配置参数（方便启动时调整） ==========
    # 棋盘格规格参数
    chessboard_size = DeclareLaunchArgument(
        'chessboard_size',
        default_value='10x7',
        description='棋盘格内角点数量（宽x高），如10x7'
    )
    square_size = DeclareLaunchArgument(
        'square_size',
        default_value='0.015',
        description='棋盘格单个方格的边长（单位：米），如0.015=1.5cm'
    )
    
    # 相机话题参数（适配你的双目节点话题）
    left_image_topic = DeclareLaunchArgument(
        'left_image_topic',
        default_value='/left_camera/image_raw',
        description='左相机图像话题名称'
    )
    right_image_topic = DeclareLaunchArgument(
        'right_image_topic',
        default_value='/right_camera/image_raw',
        description='右相机图像话题名称'
    )
    
    # 校准优化参数
    approximate_sync = DeclareLaunchArgument(
        'approximate_sync',
        default_value='0.1',
        description='左右相机图像时间同步误差容忍值（秒）'
    )
    queue_size = DeclareLaunchArgument(
        'queue_size',
        default_value='10',
        description='图像消息队列大小'
    )
    no_service_check = DeclareLaunchArgument(
        'no_service_check',
        default_value='true',
        description='是否跳过服务检查（避免校准节点卡死）'
    )

    # ========== 2. 配置双目标定节点 ==========
    stereo_calibrator_node = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        name='stereo_calibrator',
        output='screen',
        emulate_tty=True,  # 解决终端日志输出格式问题
        arguments=[
            # 棋盘格参数
            '--size', LaunchConfiguration('chessboard_size'),
            '--square', LaunchConfiguration('square_size'),
            # 同步/队列参数
            '--approximate', LaunchConfiguration('approximate_sync'),
            '--queue-size', LaunchConfiguration('queue_size'),
            # 话题映射
            'left:=' + LaunchConfiguration('left_image_topic'),
            'right:=' + LaunchConfiguration('right_image_topic'),
            # 条件参数：仅当no_service_check为true时添加
        ] + (['--no-service-check'] if LaunchConfiguration('no_service_check').value == 'true' else []),
        # 可选：设置日志级别
        arguments=['--ros-args', '--log-level', 'info']
    )

    # ========== 3. 组装Launch描述 ==========
    return LaunchDescription([
        # 声明参数
        chessboard_size,
        square_size,
        left_image_topic,
        right_image_topic,
        approximate_sync,
        queue_size,
        no_service_check,
        # 启动校准节点
        stereo_calibrator_node
    ])