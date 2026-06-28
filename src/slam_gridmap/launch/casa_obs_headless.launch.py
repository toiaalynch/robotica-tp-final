#!/usr/bin/env python3
"""
Simulacion de la casa con obstaculos simples, sin ventana 3D de Gazebo.

Es la variante headless de:
  ros2 launch turtlebot3_custom_simulation custom_casa_obs.launch.py

Sirve para probar la Parte B en el entorno con obstaculos que pide la consigna,
pero mirando todo desde RViz o desde scripts automaticos.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    sim_share = get_package_share_directory('turtlebot3_custom_simulation')
    tb3_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    models_path = os.path.join(sim_share, 'worlds')
    os.environ["GAZEBO_MODEL_PATH"] = (
        models_path + ":" + os.environ.get("GAZEBO_MODEL_PATH", ""))

    world = os.path.join(sim_share, 'worlds', 'casa_o.world')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items()
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([gzserver, spawn_robot, robot_state_publisher])
