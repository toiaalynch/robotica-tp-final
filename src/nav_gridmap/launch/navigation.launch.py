#!/usr/bin/env python3
"""
Launch de NAVEGACION COMPLETA (Parte B, Sistema 1).
===================================================

Levanta de una sola vez todo el stack de navegacion:
  - mcl_localization : localizacion por filtro de particulas (TF map->odom).
  - navigator        : planificacion + seguimiento + evasion + maquina de estados.
  - RViz             : vista con mapa, nube, plan, pose y herramientas
                       2D Pose Estimate / 2D Goal Pose.

Uso tipico (4 terminales):

  # 1) simulacion (entorno estandar o con obstaculos)
  ros2 launch turtlebot3_custom_simulation custom_casa.launch.py
  #    o:  custom_casa_obs.launch.py   (con obstaculos)

  # 2) teleop SOLO para la localizacion inicial (mover un poco y converger)
  ros2 run turtlebot3_teleop teleop_keyboard

  # 3) navegacion completa
  ros2 launch nav_gridmap navigation.launch.py

Luego, en RViz:
  a) "2D Pose Estimate" -> marcar donde esta el robot (localizacion).
  b) Mover un poco con teleop hasta que la nube converja; cerrar el teleop.
  c) "2D Goal Pose" -> marcar destino y orientacion. El robot va solo.

Argumentos: robot_type:=tb3|tb4 , map:=/ruta.yaml , rviz:=true|false
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
    default_map = os.path.join(pkg, 'maps', 'mapa_fastslam_final_v2_nav.yaml')
    params_file = os.path.join(pkg, 'config', 'navigation.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'navigation.rviz')

    robot_type = LaunchConfiguration('robot_type')
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('rviz')

    common = {'robot_type': robot_type, 'map_yaml': map_yaml}

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='tb3'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(
            package='nav_gridmap', executable='mcl_localization',
            name='mcl_localization', output='screen',
            parameters=[params_file, common],
        ),
        Node(
            package='nav_gridmap', executable='navigator',
            name='navigator', output='screen',
            parameters=[params_file, common],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2_navigation',
            output='screen', arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_rviz),
        ),
    ])
