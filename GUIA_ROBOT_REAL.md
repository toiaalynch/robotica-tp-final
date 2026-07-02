# Guia de laboratorio - Parte A con TurtleBot4 fisico

Esta guia es para correr el **Grid-Based FastSLAM de la Parte A** con el robot
fisico del laboratorio. La idea es no depender de memoria: cada paso incluye
chequeos para confirmar que el robot publica lo que esperamos y que nuestro nodo
esta efectivamente suscripto.

> Importante: en robot real se usa `use_sim_time:=false`. No hay reloj de
> Gazebo ni `/clock` de rosbag. El launch ya lo pone automaticamente en `false`
> cuando se usa `robot_type:=tb4`.

---

## 0. Robot y red

1. Prender el TurtleBot4.
2. En el robot / app / botonera del laboratorio, ponerlo en **mode 0** para
   conectarlo y dejar activos los sensores.
3. Conectar la PC al mismo Wi-Fi/red que el robot.
4. Confirmar con el docente o con el grupo el `ROS_DOMAIN_ID` del robot.

En cada terminal de Ubuntu que uses:

```bash
source /opt/ros/humble/setup.bash
cd ~/tp_final/robotica-tp-final
source install/setup.bash
```

Si en el laboratorio usan un dominio ROS especifico:

```bash
export ROS_DOMAIN_ID=<numero_del_laboratorio>
```

Ejemplo:

```bash
export ROS_DOMAIN_ID=0
```

---

## 1. Actualizar y compilar el repo

```bash
cd ~/tp_final/robotica-tp-final
git pull
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Chequeo rapido:

```bash
ros2 pkg list | grep -E "slam_gridmap|nav_gridmap"
```

Tienen que aparecer:

```text
nav_gridmap
slam_gridmap
```

---

## 2. Verificar que la PC ve al robot

Primero listar nodos y topicos:

```bash
ros2 node list
ros2 topic list | sort
```

Para nuestro perfil `tb4`, esperamos ver como minimo:

```text
/tb4_0/scan
/tb4_0/odom
/tb4_0/tf
/tb4_0/tf_static
```

Tambien puede haber camara:

```text
/tb4_0/oakd/rgb/preview/image_raw
/tb4_0/oakd/rgb/preview/camera_info
```

Si no aparecen topicos con `/tb4_0/`, probar:

```bash
ros2 topic list | grep -E "scan|odom|tf|oakd|camera"
```

Si aparecen como `/scan`, `/odom`, `/tf`, etc., anotar esos nombres porque hay
que lanzar el SLAM con parametros manuales o ajustar el perfil.

---

## 3. Verificar que los sensores publican datos

Laser:

```bash
ros2 topic hz /tb4_0/scan
```

Deberia mostrar frecuencia. Cortar con `Ctrl+C`.

Odometria:

```bash
ros2 topic echo /tb4_0/odom --once
```

Tiene que imprimir una `pose` y una `twist`.

TF:

```bash
ros2 topic echo /tb4_0/tf --once
ros2 topic echo /tb4_0/tf_static --once
```

Tiene que salir al menos algun frame del robot. Si `/tb4_0/tf_static` tarda,
esperar unos segundos y repetir.

Chequeo de QoS de odometria:

```bash
ros2 topic info /tb4_0/odom --verbose
```

En TurtleBot4 real suele ser `BEST_EFFORT`. Nuestro perfil `tb4` ya se suscribe
con `best_effort`, por eso este punto es importante.

---

## 4. Chequeo antes de lanzar SLAM: nadie suscripto todavia

Antes de correr nuestro nodo, mirar:

```bash
ros2 topic info /tb4_0/odom
ros2 topic info /tb4_0/scan
```

Es normal que diga algo parecido a:

```text
Publisher count: 1
Subscription count: 0
```

Lo importante es que **Publisher count sea 1 o mas**. Si es 0, la PC no esta
viendo el robot o el topico tiene otro nombre.

---

## 5. Lanzar Parte A - SLAM en robot real

Terminal SLAM:

```bash
cd ~/tp_final/robotica-tp-final
source /opt/ros/humble/setup.bash
source install/setup.bash

mkdir -p ~/tp_final/resultados_robot_real

ros2 launch slam_gridmap slam_gridmap.launch.py \
  robot_type:=tb4 \
  use_sim_time:=false \
  use_rviz:=true \
  num_particles:=30
```

Con `robot_type:=tb4`, el nodo usa:

| Parametro | Valor esperado |
|---|---|
| `use_sim_time` | `false` |
| `scan_topic` | `/tb4_0/scan` |
| `odom_topic` | `/tb4_0/odom` |
| QoS odom | `best_effort` |
| LIDAR | offset angular `+90 deg`, offset fisico `x=-0.04 m` |
| TF | remapeado a `/tb4_0/tf` y `/tb4_0/tf_static` |

En la consola del SLAM tiene que aparecer una linea similar:

```text
Grid-Based FastSLAM iniciado [robot=tb4]: ... scan=/tb4_0/scan  odom=/tb4_0/odom (best_effort).
```

---

## 6. Chequeo despues de lanzar SLAM: confirmar suscripciones

En otra terminal:

```bash
ros2 topic info /tb4_0/odom
ros2 topic info /tb4_0/scan
```

Ahora deberia cambiar a algo parecido a:

```text
Publisher count: 1
Subscription count: 1
```

Si `/tb4_0/odom` sigue con `Subscription count: 0`, el SLAM **no esta
escuchando la odometria**. No sigas mapeando: hay que corregir los topicos.

Chequeo de que nuestro nodo existe:

```bash
ros2 node list | grep grid_fastslam
```

Chequeo de parametros reales del nodo:

```bash
ros2 param get /grid_fastslam use_sim_time
ros2 param get /grid_fastslam robot_type
ros2 param get /grid_fastslam scan_topic
ros2 param get /grid_fastslam odom_topic
ros2 param get /grid_fastslam odom_qos
```

Esperado:

```text
use_sim_time: false
robot_type: tb4
scan_topic: /tb4_0/scan
odom_topic: /tb4_0/odom
odom_qos: best_effort
```

---

## 7. Verificar que se esta generando mapa

Sin mover el robot todavia:

```bash
ros2 topic list | grep -E "^/map$|^/belief$|slam"
```

Luego:

```bash
ros2 topic echo /belief --once
ros2 topic echo /map --once
```

Si `/map --once` devuelve un mensaje, el mapa se esta publicando.

En RViz:

- `Fixed Frame`: `map`
- Display `Map`: topic `/map`
- Display `LaserScan`: topic `/tb4_0/scan`
- Display `Pose`: `/belief`
- Display `Path`: `/slam/path`

Si RViz no muestra nada pero `/map --once` si devuelve datos, el problema es de
visualizacion/TF, no necesariamente del SLAM.

---

## 8. Mover el robot para mapear

Si el laboratorio les permite teleoperar:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/tb4_0/cmd_vel
```

Si ese paquete no existe, probar el teleop del TurtleBot:

```bash
ros2 run turtlebot3_teleop teleop_keyboard \
  --ros-args -r cmd_vel:=/tb4_0/cmd_vel
```

Mover lento:

- avanzar de a poco;
- giros suaves;
- evitar empujar paredes/conos;
- volver a pasar por zonas conocidas para cerrar bien el mapa;
- si el mapa se deforma mucho, frenar, guardar lo que haya y reiniciar limpio.

---

## 9. Guardar mapa final

El nodo autoguarda cada 15 s y tambien al cerrar con `Ctrl+C`.

Por defecto guarda en:

```text
~/maps/mapa_slam.pgm
~/maps/mapa_slam.yaml
~/maps/mapa_slam.png
```

Para guardar en una ruta de entrega, lanzar SLAM asi:

```bash
ros2 launch slam_gridmap slam_gridmap.launch.py \
  robot_type:=tb4 \
  use_sim_time:=false \
  use_rviz:=true \
  num_particles:=30 \
  params_file:=install/slam_gridmap/share/slam_gridmap/config/params.yaml
```

Si quieren forzar ruta exacta, usar `ros2 run`:

```bash
ros2 run slam_gridmap grid_fastslam --ros-args \
  --params-file install/slam_gridmap/share/slam_gridmap/config/params.yaml \
  -p use_sim_time:=false \
  -p robot_type:=tb4 \
  -p map_save_path:=/home/$USER/tp_final/resultados_robot_real/mapa_robot_real \
  -p autosave_period:=15.0
```

Despues revisar:

```bash
ls -lh ~/tp_final/resultados_robot_real
```

---

## 10. Plan B si los topicos no son `/tb4_0/...`

Si `ros2 topic list` muestra otros nombres, por ejemplo `/odom` y `/scan`, no
uses el launch automatico. Lanzar el nodo con parametros explicitos:

```bash
ros2 run slam_gridmap grid_fastslam --ros-args \
  --params-file install/slam_gridmap/share/slam_gridmap/config/params.yaml \
  -p use_sim_time:=false \
  -p robot_type:=tb4 \
  -p scan_topic:=/scan \
  -p odom_topic:=/odom \
  -p odom_qos:=best_effort \
  -p map_save_path:=/home/$USER/tp_final/resultados_robot_real/mapa_robot_real \
  -p autosave_period:=15.0
```

Y abrir RViz aparte:

```bash
rviz2 -d install/slam_gridmap/share/slam_gridmap/rviz/slam_gridmap.rviz
```

Despues repetir los chequeos:

```bash
ros2 topic info /odom
ros2 topic info /scan
ros2 param get /grid_fastslam odom_topic
ros2 param get /grid_fastslam scan_topic
```

---

## 11. Checklist corto para mañana

```bash
# 1) Robot en mode 0 y conectado
ros2 topic list | grep -E "tb4_0|scan|odom|tf"

# 2) Sensores vivos
ros2 topic hz /tb4_0/scan
ros2 topic echo /tb4_0/odom --once

# 3) Antes de SLAM: debe haber publisher
ros2 topic info /tb4_0/odom

# 4) Lanzar SLAM real
ros2 launch slam_gridmap slam_gridmap.launch.py robot_type:=tb4 use_sim_time:=false

# 5) Despues de SLAM: debe haber suscriptor
ros2 topic info /tb4_0/odom
ros2 topic info /tb4_0/scan

# 6) Confirmar parametros
ros2 param get /grid_fastslam use_sim_time
ros2 param get /grid_fastslam odom_topic

# 7) Confirmar mapa
ros2 topic echo /map --once
ros2 topic echo /belief --once
```

Si falla el punto 5, parar y corregir nombres de topicos antes de seguir.
