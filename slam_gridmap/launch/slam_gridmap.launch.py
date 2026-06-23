#!/usr/bin/env python3
"""
Launch del Grid-Based FastSLAM (Parte A, Opcion 1).
===================================================

Este launch arranca SOLO el SLAM + RViz. La simulacion de Gazebo se lanza
aparte (en otra terminal), segun la consigna:

  Terminal 1:  ros2 launch turtlebot3_custom_simulation custom_casa.launch.py
  Terminal 2:  ros2 run turtlebot3_teleop teleop_keyboard
  Terminal 3:  ros2 launch slam_gridmap slam_gridmap.launch.py

Argumentos utiles:
  num_particles:=50      cambia la cantidad de particulas
  use_rviz:=false        no abrir RViz
  params_file:=/ruta.yaml  usar otro archivo de parametros
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('slam_gridmap')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam_gridmap.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    num_particles = LaunchConfiguration('num_particles')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('num_particles', default_value='30'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        # Nodo de SLAM. Los parametros salen del YAML; ademas sobre-escribimos
        # use_sim_time y num_particles desde los argumentos del launch.
        Node(
            package='slam_gridmap',
            executable='grid_fastslam',
            name='grid_fastslam',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time,
                 'num_particles': num_particles},
            ],
        ),

        # RViz con la configuracion preparada (mapa, particulas, rayos, etc.)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
