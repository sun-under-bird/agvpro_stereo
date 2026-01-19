#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 左相机畸变校正（核心：生成image_rect）
    left_image_proc = Node(
        package='image_proc',
        executable='image_proc',
        name='left_image_proc',
        namespace='left_camera',
        remappings=[
            ('image', 'image_raw'),
            ('image_rect', 'image_rect'),
            ('camera_info','camera_info'),
        ]
    )

    # 2. 右相机畸变校正
    right_image_proc = Node(
        package='image_proc',
        executable='image_proc',
        name='right_image_proc',
        namespace='right_camera',
        remappings=[
            ('image', 'image_raw'),
            ('image_rect', 'image_rect'),
            ('camera_info','camera_info'),
        ]
    )

    # 3. 双目视差计算（stereo_image_proc的核心功能）
    disparity_node = Node(
        package='stereo_image_proc',
        executable='disparity_node',  # 视差计算节点
        name='disparity_node',
        remappings=[
            # 输入：校正后的左右图像+内参
            ('left/image_rect', '/left_camera/image_rect'),
            ('left/camera_info', '/left_camera/camera_info'),
            ('right/image_rect', '/right_camera/image_rect'),
            ('right/camera_info', '/right_camera/camera_info'),
            # 输出：视差图（可自定义话题）
            ('disparity', '/stereo/disparity'),
        ],
        parameters=[{
            'approximate_sync': True,  # 允许左右图像时间戳轻微不同步
        }]
    )

    # 4. 双目点云生成（可选，基于视差图）
    point_cloud_node = Node(
        package='stereo_image_proc',
        executable='point_cloud_node',  # 点云生成节点
        name='point_cloud_node',
        remappings=[
            # 输入：视差图+校正后图像/内参
            ('left/image_rect', '/left_camera/image_rect'),
            ('left/camera_info', '/left_camera/camera_info'),
            ('right/image_rect', '/right_camera/image_rect'),
            ('right/camera_info', '/right_camera/camera_info'),
            ('disparity', '/stereo/disparity'),
            # 输出：3D点云（可用于SLAM/可视化）
            ('points2', '/stereo/points2'),
        ]
    )

    return LaunchDescription([
        left_image_proc,
        right_image_proc,
        disparity_node,
        point_cloud_node
    ])