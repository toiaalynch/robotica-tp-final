#!/usr/bin/env python3
"""
Launch de Localizacion MCL (Parte B, Sistema 1).
================================================

Arranca:
  - el nodo mcl_localization (filtro de particulas contra el mapa de la Parte A),
  - RViz con una vista preparada (mapa + nube + pose estimada).

El mapa se resuelve solo (el que viene instalado en el paquete). Se puede
sobreescribir con argumentos:

  ros2 launch nav_gridmap localization.launch.py
  ros2 launch nav_gridmap localization.launch.py robot_type:=tb4
  ros2 launch nav_gridmap localization.launch.py map:=/ruta/a/otro_mapa.yaml
  ros2 launch nav_gridmap localization.launch.py rviz:=false

Pasos para localizar (en RViz):
  1) Click en "2D Pose Estimate".
  2) Click + arrastrar en el mapa donde esta (de verdad) el robot, en su sentido.
  3) Mové el robot (teleop): la nube debe converger sobre la pose real.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('nav_gridmap')
    default_map = os.path.join(pkg, 'maps', 'mapa_fastslam_final_v2.yaml')
    params_file = os.path.join(pkg, 'config', 'localization.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'localization.rviz')

    robot_type = LaunchConfiguration('robot_type')
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='tb3',
                              description='tb3 (Gazebo) o tb4 (real)'),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Ruta al .yaml del mapa estatico'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Lanzar RViz (true/false)'),

        Node(
            package='nav_gridmap',
            executable='mcl_localization',
            name='mcl_localization',
            output='screen',
            parameters=[
                params_file,
                {'robot_type': robot_type, 'map_yaml': map_yaml},
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_localization',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_rviz),
        ),
    ])
