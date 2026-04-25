#!/usr/bin/env python3
"""
nav2.launch.py  –  launch Nav2 stack only (expects sim + localization already running)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_cfg_dir = get_package_share_directory('nav2_config')
    sim_pkg_dir  = get_package_share_directory('simulation_pkg')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml     = LaunchConfiguration('map', default=os.path.join(
        sim_pkg_dir, 'maps', 'shopfloor.yaml'))
    params_file  = LaunchConfiguration('params_file', default=os.path.join(
        nav2_cfg_dir, 'config', 'nav2_params.yaml'))

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'use_lifecycle_mgr': 'True',
            'map_subscribe_transient_local': 'True',
        }.items()
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'yaml_filename': map_yaml}]
    )

    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'node_names': ['map_server']}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.join(
            sim_pkg_dir, 'maps', 'shopfloor.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            nav2_cfg_dir, 'config', 'nav2_params.yaml')),

        map_server,
        map_lifecycle,
        nav2,
    ])
