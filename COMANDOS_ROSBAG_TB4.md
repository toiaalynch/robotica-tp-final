# Comandos para probar SLAM con rosbag de TurtleBot4

Esta guia permite validar la Parte A con el rosbag del laberinto real. El
rosbag debe estar disponible en:

```bash
~/tp_final/rosbags/laberinto
```

El rosbag publica topicos namespaceados como `/tb4_0/scan` y `/tb4_0/odom`.
Por eso se usa `robot_type:=tb4`.

## 1. Preparar workspace

```bash
cd ~/tp_final/robotica-tp-final
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2. Limpiar procesos previos

Si se reinicia el rosbag sin cerrar nodos, RViz puede mostrar errores de tiempo
como `TF_OLD_DATA` o `Detected jump back in time`. Para arrancar limpio:

```bash
pkill -f "ros2 bag play"
pkill -f "grid_fastslam"
pkill -f "rviz2"
```

## 3. Terminal 1: correr SLAM

```bash
cd ~/tp_final/robotica-tp-final
source /opt/ros/humble/setup.bash
source install/setup.bash

rm -rf ~/tp_final/resultados_rosbag
mkdir -p ~/tp_final/resultados_rosbag

ros2 run slam_gridmap grid_fastslam --ros-args \
  -p use_sim_time:=true \
  -p robot_type:=tb4 \
  -p map_save_path:=/home/usuario/tp_final/resultados_rosbag/mapa_laberinto_bag \
  -p autosave_period:=15.0
```

Con `robot_type:=tb4` el nodo configura automaticamente:

- `/tb4_0/scan`
- `/tb4_0/odom`
- QoS `best_effort` para odometria
- rotacion del LIDAR `+90 deg`
- offset del LIDAR `x=-0.04 m`
- descarte de lecturas con intensidad `0.0`

## 4. Terminal 2: abrir RViz

```bash
cd ~/tp_final/robotica-tp-final
source /opt/ros/humble/setup.bash
source install/setup.bash

LIBGL_ALWAYS_SOFTWARE=1 QT_OPENGL=software rviz2 -d rviz_rosbag_laberinto.rviz \
  --ros-args -p use_sim_time:=true
```

Si RViz muestra muchos avisos de `Message Filter dropping message`, se puede
desactivar temporalmente el display `LaserScan`. El mapa `/map` sigue
generandose igual.

## 5. Terminal 3: reproducir rosbag

```bash
cd ~/tp_final/robotica-tp-final
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag play ~/tp_final/rosbags/laberinto --clock --rate 1.0 \
  --remap /tb4_0/tf:=/tf /tb4_0/tf_static:=/tf_static
```

Para acelerar una prueba corta, se puede usar `--rate 4.0` u `--rate 8.0`.
Para observar RViz con menos carga, usar `--rate 0.5` o `--rate 1.0`.

## 6. Verificar que funciona

En otra terminal:

```bash
ros2 topic echo /belief --once
ros2 topic echo /map --once
```

Tambien se puede revisar que se hayan guardado los archivos:

```bash
ls -lh ~/tp_final/resultados_rosbag
```

Se esperan:

```bash
mapa_laberinto_bag.pgm
mapa_laberinto_bag.yaml
mapa_laberinto_bag.png
```

## 7. Criterio rapido

Si el mapa sale como una mancha pegada al robot, revisar:

1. Que SLAM y RViz esten con `use_sim_time:=true`.
2. Que el rosbag se haya lanzado con `--clock`.
3. Que el nodo diga en consola `robot=tb4`, `scan=/tb4_0/scan` y
   `odom=/tb4_0/odom`.
4. Que no se haya reiniciado el rosbag sin limpiar procesos previos.

## 8. Parte C con rosbag de conos

### Opcion automatica recomendada

Esta prueba levanta SLAM, detector de cono rojo y rosbag desde el inicio. Es
mas lenta, pero evita perder `/tf_static` y guarda todo en una carpeta de
resultados:

```bash
cd ~/tp_final/robotica-tp-final
source /opt/ros/humble/setup.bash
source install/setup.bash

OUT=$HOME/tp_final/resultados_rosbag_cones_check \
RATE=0.5 \
START_OFFSET=0 \
DURATION=200 \
scripts/run_rosbag_cones_check.sh
```

Al terminar, revisar:

```bash
ls -lh ~/tp_final/resultados_rosbag_cones_check
tail -30 ~/tp_final/resultados_rosbag_cones_check/red_cone_status.log
tail -40 ~/tp_final/resultados_rosbag_cones_check/red_cone_goal_pose.log
```

Se esperan archivos `mapa_laberinto_conos_check.pgm/.yaml/.png` y mensajes
`cono rojo area=... goal=(x,y)`.

### Opcion manual por terminales

Para probar la deteccion de conos con el rosbag `laberinto_conos`, conviene
levantar primero SLAM, luego reproducir el bag, esperar unos segundos a que
exista `map -> odom`, y recien despues lanzar el detector.

```bash
ros2 launch nav_gridmap red_cone_mission.launch.py \
  use_sim_time:=true \
  robot_type:=tb4 \
  image_topic:=/tb4_0/oakd/rgb/preview/image_raw \
  camera_info_topic:=/tb4_0/oakd/rgb/preview/camera_info \
  target_frame:=map \
  auto_goal:=false \
  tf_timeout:=1.0 \
  tf_cache_time:=120.0
```

Para verificar:

```bash
ros2 topic echo /red_cone/status
ros2 topic echo /red_cone/goal_pose --once
```

Si solo se quiere validar la deteccion sin depender del mapa SLAM, usar
`target_frame:=odom`.

Con rosbags pesados puede aparecer el aviso `Lookup would require extrapolation
into the past`. En ese caso bajar la velocidad del bag (`--rate 0.5` o
`--rate 1.0`) y mantener `tf_cache_time:=120.0` para que el detector conserve
mas historial de transformaciones.
