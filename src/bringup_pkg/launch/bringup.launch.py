#!/usr/bin/env python3
"""
bringup.launch.py
-----------------
Master launch file. Starts:
  1. Simulation (Gazebo + robot)
  2. AprilTag localizer  (overhead camera → pose)
  3. Pose relay          (pose → TF + /odom)
  4. Map server
  5. Nav2 navigation stack (no AMCL)
  6. RViz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                             IncludeLaunchDescription, TimerAction,
                             SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Package paths ──
    sim_pkg_dir = get_package_share_directory('simulation_pkg')
    nav2_cfg_dir = get_package_share_directory('nav2_config')
    bringup_dir = get_package_share_directory('bringup_pkg')

    # ── Launch args ──
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration(
        'map',
        default=os.path.join(sim_pkg_dir, 'maps', 'shopfloor.yaml'))
    nav2_params = LaunchConfiguration(
        'params_file',
        default=os.path.join(nav2_cfg_dir, 'config', 'nav2_params.yaml'))
    rviz_config = LaunchConfiguration(
        'rviz_config',
        default=os.path.join(bringup_dir, 'rviz', 'nav2_view.rviz'))
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')

    # ── 1. Simulation ──
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_dir, 'launch', 'simulation.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ── 2. AprilTag Localizer ──
    apriltag_node = Node(
        package='apriltag_detection_pkg',
        executable='apriltag_localizer',
        name='apriltag_localizer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'camera_topic': '/overhead_camera/image_raw',
                     'camera_info_topic': '/overhead_camera/camera_info',
                     'tag_id': 0}]
    )

    # ── 3. Pose Relay (TF + /odom publisher) ──
    pose_relay_node = Node(
        package='camera_localization_pkg',
        executable='pose_relay',
        name='pose_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── 4. Map Server ──
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'yaml_filename': map_yaml}]
    )

    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'node_names': ['map_server']}]
    )

    # ── 5. Nav2 Navigation Stack ──
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'use_lifecycle_mgr': 'True',
            'map_subscribe_transient_local': 'True',
        }.items()
    )

    # ── 6. RViz ──
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.join(
            sim_pkg_dir, 'maps', 'shopfloor.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            nav2_cfg_dir, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            bringup_dir, 'rviz', 'nav2_view.rviz')),
        DeclareLaunchArgument('launch_rviz', default_value='true'),

        # Start simulation first
        sim_launch,

        # Give Gazebo + robot time to initialise, then start perception+nav
        TimerAction(period=5.0, actions=[
            apriltag_node,
            pose_relay_node,
            map_server_node,
            map_server_lifecycle,
        ]),

        # Nav2 starts after map server
        TimerAction(period=8.0, actions=[
            nav2_launch,
        ]),

        # RViz after everything
        TimerAction(period=10.0, actions=[
            rviz_node,
        ]),
    ])
