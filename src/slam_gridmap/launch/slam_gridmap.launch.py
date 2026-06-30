#!/usr/bin/env python3
"""
Launch del Grid-Based FastSLAM (Parte A, Opcion 1).
===================================================

Este launch arranca SOLO el SLAM + RViz. La simulacion de Gazebo (tb3) o el
stack del robot real (tb4) se lanzan APARTE, en otra terminal.

  --- Simulacion (TurtleBot3 + Gazebo) ---
  Terminal 1:  ros2 launch turtlebot3_custom_simulation custom_casa.launch.py
  Terminal 2:  ros2 run turtlebot3_teleop teleop_keyboard
  Terminal 3:  ros2 launch slam_gridmap slam_gridmap.launch.py

  --- Robot real (TurtleBot4) ---
  (con el stack del TB4 ya corriendo y publicando /tb4_0/scan, /tb4_0/odom, ...)
  ros2 launch slam_gridmap slam_gridmap.launch.py robot_type:=tb4

Argumentos utiles:
  robot_type:=tb4        usar el robot real (default: tb3 = Gazebo)
  num_particles:=50      cambia la cantidad de particulas
  use_rviz:=false        no abrir RViz
  use_sim_time:=auto     'auto' decide solo: true en sim (tb3), false en real (tb4).
                         Podes forzarlo con use_sim_time:=true|false.
  params_file:=/ruta.yaml  usar otro archivo de parametros

Notas sobre el TurtleBot4 real:
  - El TB4 publica TODO bajo el namespace /tb4_0/, incluido el arbol de
    transformadas (/tb4_0/tf y /tb4_0/tf_static). Por eso, en tb4 remapeamos
    /tf y /tf_static al namespace del robot, asi nuestra correccion map->odom
    entra en el MISMO arbol de TF que el robot (si no, RViz veria dos arboles
    desconectados y descartaria el LaserScan).
  - El robot real NO usa el reloj de Gazebo: use_sim_time debe ser false. Con
    'auto' no hace falta acordarse: se pone solo en false cuando robot_type=tb4.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('slam_gridmap')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam_gridmap.rviz')

    # Resolvemos los argumentos a texto para poder decidir con logica Python.
    robot_type = LaunchConfiguration('robot_type').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    if not params_file:                       # vacio -> usar el params.yaml del paquete
        params_file = default_params
    num_particles = LaunchConfiguration('num_particles').perform(context)
    use_sim_time_arg = LaunchConfiguration('use_sim_time').perform(context)

    # --- use_sim_time automatico ---
    # 'auto' (default): true en simulacion (tb3), false en el robot real (tb4).
    # Asi, en la vida real NO hay que pasar nada: se desactiva solo.
    if use_sim_time_arg.lower() == 'auto':
        use_sim_time = (robot_type == 'tb3')
    else:
        use_sim_time = use_sim_time_arg.lower() in ('true', '1', 'yes')

    # --- TF namespaceado en el robot real ---
    # El TB4 publica su arbol de transformadas bajo /tb4_0/tf y /tb4_0/tf_static.
    # Remapeamos los topicos de TF de nuestro nodo a ese namespace para que la
    # transformada map->odom quede en el mismo arbol que el robot.
    remappings = []
    if robot_type == 'tb4':
        remappings = [
            ('/tf', '/tb4_0/tf'),
            ('/tf_static', '/tb4_0/tf_static'),
        ]

    # Nodo de SLAM. Los parametros salen del YAML; ademas sobre-escribimos
    # robot_type, use_sim_time y num_particles (el override gana sobre el YAML).
    slam_node = Node(
        package='slam_gridmap',
        executable='grid_fastslam',
        name='grid_fastslam',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time,
             'robot_type': robot_type,
             'num_particles': int(num_particles)},
        ],
        remappings=remappings,
    )

    # RViz con la configuracion preparada (mapa, particulas, rayos, etc.).
    # Tambien remapeamos su TF en tb4 para que vea el arbol del robot real.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return [slam_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='tb3',
                              description='tb3 (Gazebo/simulacion) o tb4 (robot real)'),
        DeclareLaunchArgument('use_sim_time', default_value='auto',
                              description="auto -> true si tb3, false si tb4. Forzable: true|false"),
        DeclareLaunchArgument('num_particles', default_value='30'),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
