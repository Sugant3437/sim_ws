#!/usr/bin/env python3
"""
simulation.launch.py
--------------------
Launches:
  1. Gazebo with shopfloor world
  2. Robot description / robot_state_publisher
  3. Spawn robot in Gazebo
  4. Overhead camera (part of world file)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    # ── Package paths ──
    robot_desc_pkg = get_package_share_directory('robot_description')
    sim_pkg = get_package_share_directory('simulation_pkg')

    # ── Launch args ──
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_x = LaunchConfiguration('robot_x', default='1.0')
    robot_y = LaunchConfiguration('robot_y', default='1.0')
    robot_yaw = LaunchConfiguration('robot_yaw', default='0.0')

    # ── Process URDF via xacro ──
    xacro_file = os.path.join(robot_desc_pkg, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # ── World file ──
    world_file = os.path.join(sim_pkg, 'worlds', 'shopfloor.world')

    # ── Gazebo launch ──
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',
             '-s', 'libgazebo_ros_init.so', world_file],
        output='screen'
    )

    # ── Robot state publisher ──
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # ── Spawn robot entity in Gazebo ──
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'amr_robot',
            '-x', robot_x,
            '-y', robot_y,
            '-z', '0.05',
            '-Y', robot_yaw,
        ],
        output='screen'
    )

    # ── Joint state publisher (for wheel joints) ──
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_x', default_value='1.0'),
        DeclareLaunchArgument('robot_y', default_value='1.0'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),

        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        # Delay spawn to let Gazebo fully start
        TimerAction(period=3.0, actions=[spawn_robot]),
    ])
