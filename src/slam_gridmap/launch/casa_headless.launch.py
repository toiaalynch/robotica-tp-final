#!/usr/bin/env python3
"""
Simulacion de la casa SIN la ventana 3D de Gazebo (headless).
=============================================================

En Mac con Apple Silicon, la ventana 3D de Gazebo Classic (gzclient) suele
crashear por un bug del renderer OpenGL/Metal. Pero esa ventana NO hace falta
para el SLAM: alcanza con gzserver (fisica + sensores) y mirar todo en RViz.

Este launch arranca lo mismo que custom_casa.launch.py PERO sin gzclient:
  - gzserver con el mundo casa.world
  - spawn del TurtleBot3
  - robot_state_publisher (publica el TF del robot: base_footprint -> base_scan)

Uso (reemplaza a la Terminal 1):
  ros2 launch slam_gridmap casa_headless.launch.py

Despues, igual que siempre:
  Terminal 2:  ros2 run turtlebot3_teleop teleop_keyboard
  Terminal 3:  ros2 launch slam_gridmap slam_gridmap.launch.py
Manejas el robot mirando el mapa y el LIDAR en RViz (no necesitas ver Gazebo).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    sim_share = get_package_share_directory('turtlebot3_custom_simulation')
    tb3_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # para que Gazebo encuentre los modelos del mundo
    models_path = os.path.join(sim_share, 'worlds')
    os.environ["GAZEBO_MODEL_PATH"] = (
        models_path + ":" + os.environ.get("GAZEBO_MODEL_PATH", ""))

    world = os.path.join(sim_share, 'worlds', 'casa.world')

    # SOLO gzserver (sin gzclient -> sin ventana 3D -> sin crash)
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

    calc_odom = Node(
        package='turtlebot3_custom_simulation',
        executable='turtlebot3_custom_simulation',
        name='turtlebot3_custom_simulation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'calc_odom',
            'base_frame': 'calc_base_footprint',
            'joint_states_frame': 'base_footprint',
            'wheels.separation': 0.160,
            'wheels.radius': 0.033,
        }]
    )

    return LaunchDescription([gzserver, spawn_robot, robot_state_publisher, calc_odom])
