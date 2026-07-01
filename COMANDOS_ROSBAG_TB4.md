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
