#!/usr/bin/env python3
"""apriltag.launch.py – start only the AprilTag localizer + pose relay"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='apriltag_detection_pkg',
            executable='apriltag_localizer',
            name='apriltag_localizer',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'camera_topic': '/overhead_camera/image_raw',
                'camera_info_topic': '/overhead_camera/camera_info',
                'tag_id': 0,
            }]
        ),
        Node(
            package='camera_localization_pkg',
            executable='pose_relay',
            name='pose_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
