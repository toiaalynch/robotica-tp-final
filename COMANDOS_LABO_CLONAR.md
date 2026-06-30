# Comandos para clonar el TP Final en Ubuntu del laboratorio

## 1. Abrir una terminal de Ubuntu

Actualizar paquetes e instalar `git` si hace falta:

```bash
sudo apt update
sudo apt install -y git
```

## 2. Crear carpeta de trabajo

```bash
mkdir -p ~/tp_final
cd ~/tp_final
```

## 3. Clonar el repositorio

```bash
git clone https://github.com/toiaalynch/robotica-tp-final.git
cd robotica-tp-final
```

## 4. Cargar ROS 2 Humble

```bash
source /opt/ros/humble/setup.bash
```

## 5. Compilar

```bash
colcon build --symlink-install
```

## 6. Cargar el workspace compilado

```bash
source install/setup.bash
```

## 7. Verificar paquetes

```bash
ros2 pkg list | grep -E "slam_gridmap|nav_gridmap|turtlebot3_custom_simulation"
```

Si aparecen esos paquetes, el repo quedo clonado y compilado correctamente.

## 8. Comandos utiles para la prueba

SLAM con robot real:

```bash
ros2 run slam_gridmap grid_fastslam --ros-args \
  --params-file install/slam_gridmap/share/slam_gridmap/config/params.yaml \
  -p use_sim_time:=false \
  -p robot_type:=tb4 \
  -p scan_topic:=/tb4_0/scan \
  -p odom_topic:=/tb4_0/odom \
  -p odom_qos:=best_effort \
  -p map_save_path:=/home/$USER/maps/mapa_real \
  -p autosave_period:=15.0
```

Navegacion con mapa generado:

```bash
ros2 launch nav_gridmap navigation.launch.py \
  robot_type:=tb4 \
  rviz:=true \
  map:=/home/$USER/maps/mapa_real.yaml
```

Detector de cono en modo seguro:

```bash
ros2 launch nav_gridmap red_cone_mission.launch.py \
  image_topic:=/tb4_0/oakd/rgb/preview/image_raw \
  camera_info_topic:=/tb4_0/oakd/rgb/preview/camera_info \
  target_frame:=map \
  auto_goal:=false \
  use_sim_time:=false
```

Detector de cono automatico:

```bash
ros2 launch nav_gridmap red_cone_mission.launch.py \
  image_topic:=/tb4_0/oakd/rgb/preview/image_raw \
  camera_info_topic:=/tb4_0/oakd/rgb/preview/camera_info \
  target_frame:=map \
  auto_goal:=true \
  use_sim_time:=false
```

