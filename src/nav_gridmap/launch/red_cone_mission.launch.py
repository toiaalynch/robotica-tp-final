#!/usr/bin/env python3
"""
Launch Parte C: detector de conos rojos.

Uso con navegacion ya levantada:
  ros2 launch nav_gridmap red_cone_mission.launch.py

Para publicar automaticamente goals al navegador:
  ros2 launch nav_gridmap red_cone_mission.launch.py auto_goal:=true

Los topicos de camara se pueden sobreescribir desde linea de comandos para
adaptarse al rosbag o al TurtleBot4 real.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('nav_gridmap')
    params_file = os.path.join(pkg, 'config', 'perception.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='tb4'),
        DeclareLaunchArgument('image_topic', default_value='/tb4_0/color/image'),
        DeclareLaunchArgument('depth_topic', default_value=''),
        DeclareLaunchArgument('camera_info_topic', default_value=''),
        DeclareLaunchArgument('camera_frame', default_value=''),
        DeclareLaunchArgument('target_frame', default_value='map'),
        DeclareLaunchArgument('auto_goal', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('tf_timeout', default_value='0.5'),
        DeclareLaunchArgument('tf_cache_time', default_value='60.0'),

        Node(
            package='nav_gridmap',
            executable='red_cone_mission',
            name='red_cone_mission',
            output='screen',
            parameters=[
                params_file,
                {
                    'robot_type': LaunchConfiguration('robot_type'),
                    'image_topic': LaunchConfiguration('image_topic'),
                    'depth_topic': LaunchConfiguration('depth_topic'),
                    'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                    'camera_frame': LaunchConfiguration('camera_frame'),
                    'target_frame': LaunchConfiguration('target_frame'),
                    'auto_goal': LaunchConfiguration('auto_goal'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'tf_timeout': LaunchConfiguration('tf_timeout'),
                    'tf_cache_time': LaunchConfiguration('tf_cache_time'),
                },
            ],
        ),
    ])
