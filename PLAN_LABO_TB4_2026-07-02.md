# Plan de labo TB4 - Mapeo en vivo + rosbag de rescate

Fecha objetivo: 2026-07-02  
Repo esperado en labo: `~/tp_final/robotica-tp-final`  
Robot esperado: TurtleBot4 publicando bajo namespace `/tb4_0`

Este documento es el plan operativo para ir al labo con poco tiempo y no
improvisar. La prioridad es:

1. Hacer Parte A en vivo con el TurtleBot4 y guardar un mapa usable.
2. Si no sale rapido, grabar un rosbag bueno para traer a casa y debuggear.
3. Dejar evidencia minima: topics, comando usado, mapa o bag, errores vistos.

Contexto clave: Alan corrigio el perfil `tb4`. Ahora `robot_type:=tb4` usa
`/tb4_0/scan`, `/tb4_0/odom`, QoS `best_effort`, LIDAR rotado `+90 deg`, offset
del sensor `x=-0.04 m` y menor ruido de odometria. Si el mapa sale rotado o como
una mancha rara, primero confirmar que estamos en `main` actualizado.

---

## 0. Regla de tiempo

No clavarse una hora con RViz. Si en 20-25 minutos no hay mapa razonable:

1. Parar SLAM/RViz si hace falta.
2. Grabar rosbag de rescate.
3. Recorrer el laberinto despacio y completo.
4. Traer el bag a casa.

El rosbag no es derrota: es la caja negra del vuelo. Si queda bien grabado, se
puede reproducir offline todas las veces que haga falta.

---

## 1. Terminal 0 - Preparar repo y compilar

Usar Ubuntu/ROS 2 Humble en la compu del labo.

```bash
mkdir -p ~/tp_final
cd ~/tp_final

git clone https://github.com/toiaalynch/robotica-tp-final.git || true
cd robotica-tp-final
git pull

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

git log -1 --oneline
ros2 pkg list | grep -E "slam_gridmap|nav_gridmap"
```

Si `slam_gridmap` no aparece, el build o el `source install/setup.bash` no
quedo bien.

---

## 2. Terminal 1 - Verificar sensores del robot

Antes de correr nuestro SLAM, confirmar que el robot publica lo que necesitamos.

```bash
cd ~/tp_final/robotica-tp-final
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic list | sort | tee ~/tp_final/topicos_labo.txt
```

Tienen que existir, como minimo:

```text
/tb4_0/scan
/tb4_0/odom
/tb4_0/tf
/tb4_0/tf_static
```

Ahora confirmar que no son topics fantasmas:

```bash
ros2 topic hz /tb4_0/scan
```

Deberia mostrar una frecuencia, tipicamente alrededor de 10 Hz. Cortar con
`Ctrl+C` cuando ya se vea frecuencia.

```bash
ros2 topic echo /tb4_0/odom --once
ros2 topic info /tb4_0/scan --verbose
ros2 topic info /tb4_0/odom --verbose
```

Si no existen `/tb4_0/scan` o `/tb4_0/odom`, buscar el namespace real:

```bash
ros2 topic list | grep -E "scan|odom|tf"
```

Si el namespace no es `/tb4_0`, no insistir a ciegas. Hay que reemplazar los
topics en el comando de SLAM o ajustar el perfil `tb4`.

---

## 3. Terminal 2 - Correr SLAM Parte A en vivo

Este comando guarda el mapa en una ruta clara. Importante: robot real usa
`use_sim_time:=false`.

```bash
cd ~/tp_final/robotica-tp-final
source /opt/ros/humble/setup.bash
source install/setup.bash

mkdir -p ~/tp_final/resultados_labo
export MAP_BASE="$HOME/tp_final/resultados_labo/mapa_labo_$(date +%Y%m%d_%H%M)"
echo "Mapa se guardara como: $MAP_BASE"

ros2 run slam_gridmap grid_fastslam --ros-args \
  -p use_sim_time:=false \
  -p robot_type:=tb4 \
  -p map_save_path:=$MAP_BASE \
  -p autosave_period:=15.0 \
  -r /tf:=/tb4_0/tf \
  -r /tf_static:=/tb4_0/tf_static
```

En la consola del nodo buscar una linea parecida a:

```text
Grid-Based FastSLAM iniciado [robot=tb4] ... scan=/tb4_0/scan odom=/tb4_0/odom
```

Si dice `robot=tb3`, `/scan` o `/calc_odom`, estamos corriendo mal el comando.

---

## 4. Terminal 3 - Abrir RViz

```bash
cd ~/tp_final/robotica-tp-final
source /opt/ros/humble/setup.bash
source install/setup.bash

LIBGL_ALWAYS_SOFTWARE=1 QT_OPENGL=software rviz2 -d rviz_rosbag_laberinto.rviz \
  --ros-args \
  -p use_sim_time:=false \
  -r /tf:=/tb4_0/tf \
  -r /tf_static:=/tb4_0/tf_static
```

En RViz mirar:

- `Fixed Frame = map`
- `Map` en `/map`
- `LaserScan TB4` en `/tb4_0/scan`
- `Belief` en `/belief`
- `PathSLAM` en `/slam/path`
- `Particles` en `/slam/particles`

Nota: si el trazo rojo de odometria queda corrido, no bloquearse. Es un overlay
de diagnostico y no deberia impedir que el mapa funcione. El mapa y el camino
azul importan mas.

---

## 5. Terminal 4 - Teleop para recorrer y mapear

```bash
source /opt/ros/humble/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/tb4_0/cmd_vel
```

Manejo recomendado:

- Ir lento.
- Girar suave, no trompos bruscos.
- Recorrer paredes exteriores primero.
- Despues pasillos/interiores.
- Volver por zonas ya vistas para cerrar el mapa.
- Si una zona se deforma al girar, repetir el paso mas lento y con giros mas
  amplios.

El LIDAR mira 360 grados, pero el SLAM igual sufre si la odometria y el giro
son muy bruscos. Como regla simple: manejar como si llevaras una taza de cafe.

---

## 6. Terminal 1 - Verificaciones mientras corre

Estas pruebas sirven para saber si el problema es nuestro nodo, RViz o el robot.

```bash
ros2 node list | grep grid_fastslam
ros2 param get /grid_fastslam robot_type
ros2 param get /grid_fastslam scan_topic
ros2 param get /grid_fastslam odom_topic
ros2 topic info /map --verbose
ros2 topic echo /belief --once
ros2 topic echo /map --once
```

Interpretacion rapida:

- `/belief` sale: el filtro esta estimando pose.
- `/map` sale por consola pero RViz no lo muestra: problema visual/RViz/TF, no
  necesariamente del SLAM.
- `/map` no sale y `/belief` tampoco: el nodo probablemente no esta recibiendo
  scan u odom.
- `/tb4_0/scan` tiene frecuencia pero el mapa no arranca: revisar que tambien
  llegue `/tb4_0/odom`.

---

## 7. Guardar y verificar el mapa

Cuando el mapa se vea razonable:

1. Frenar el robot.
2. Hacer `Ctrl+C` en Terminal 2, la del SLAM.
3. Esperar que imprima que guardo el mapa.

Verificar archivos:

```bash
ls -lh ~/tp_final/resultados_labo
ls -lh ~/tp_final/resultados_labo/*.pgm ~/tp_final/resultados_labo/*.yaml ~/tp_final/resultados_labo/*.png
```

Esperado:

```text
mapa_labo_YYYYMMDD_HHMM.pgm
mapa_labo_YYYYMMDD_HHMM.yaml
mapa_labo_YYYYMMDD_HHMM.png
```

El `.yaml` + `.pgm` son los importantes para Partes B/C. El `.png` es para mirar
rapido o poner en informe.

---

## 8. Si falla: rosbag de rescate para Parte A

Si el SLAM no funciona en vivo, grabar datos crudos. Para esto conviene cerrar
SLAM/RViz si estan molestando, pero dejar el stack del robot andando.

Limpiar procesos nuestros si hace falta:

```bash
pkill -f "grid_fastslam"
pkill -f "rviz2"
pkill -f "ros2 bag"
```

Abrir una terminal para grabar:

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/tp_final/rosbags
export BAG_DIR="$HOME/tp_final/rosbags/labo_A_$(date +%Y%m%d_%H%M)"
echo "Grabando rosbag en: $BAG_DIR"

ros2 bag record -o "$BAG_DIR" \
  /tb4_0/scan /tb4_0/odom \
  /tb4_0/tf /tb4_0/tf_static \
  /tf /tf_static
```

Mientras graba, manejar el robot con teleop y recorrer el laberinto despacio.
Si solo hay 4 terminales, reutilizar la terminal de RViz para grabar el bag y
dejar teleop en otra.

Al terminar:

1. Frenar el robot.
2. `Ctrl+C` en la terminal de `ros2 bag record`.
3. Verificar:

```bash
ros2 bag info "$BAG_DIR"
du -sh "$BAG_DIR"
```

El `ros2 bag info` debe listar al menos `/tb4_0/scan` y `/tb4_0/odom`. Si no
estan, el bag no sirve para Parte A.

---

## 9. Rosbag recomendado si tambien queremos Parte C

Si hay cono/camara o queremos dejar material para probar deteccion visual,
grabar tambien RGB y CameraInfo.

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/tp_final/rosbags
export BAG_DIR="$HOME/tp_final/rosbags/labo_A_C_$(date +%Y%m%d_%H%M)"
echo "Grabando rosbag A+C en: $BAG_DIR"

ros2 bag record -o "$BAG_DIR" \
  /tb4_0/scan /tb4_0/odom \
  /tb4_0/tf /tb4_0/tf_static \
  /tf /tf_static \
  /tb4_0/oakd/rgb/preview/image_raw \
  /tb4_0/oakd/rgb/preview/camera_info \
  /tb4_0/oakd/rgb/image_raw \
  /tb4_0/oakd/rgb/camera_info
```

Si alguno de esos topics de camara no existe, buscar los reales:

```bash
ros2 topic list | grep -E "oakd|rgb|image|camera_info"
```

No hace falta grabar todos si pesan demasiado. Minimo util para C: imagen RGB +
`camera_info`, idealmente del mismo stream.

---

## 10. Debug rapido por sintoma

| Sintoma | Causa probable | Accion |
|---|---|---|
| No existe `/tb4_0/scan` | Namespace distinto o robot stack incompleto | `ros2 topic list \| grep -E "scan|odom"` |
| `/tb4_0/scan` existe pero `hz` no responde | Sensor/stack no publica | Revisar robot, no nuestro codigo |
| `/tb4_0/odom` no existe | Topic de odom distinto | Buscar con `ros2 topic list \| grep odom` |
| Nodo dice `scan=/scan` | No se paso `robot_type:=tb4` | Relanzar Terminal 2 |
| Nodo dice `odom=/odom` o `/calc_odom` | Repo viejo o parametro mal | `git pull`, rebuild, relanzar |
| `/map` no aparece, `/belief` tampoco | SLAM no recibe scan/odom | Ver Terminal 1 |
| `/map` aparece por consola pero no en RViz | Problema de visualizacion/TF/QoS | No bloquearse; guardar mapa igual |
| Mapa rotado/manchado | Falta fix TB4 LIDAR/odom o repo viejo | Confirmar commit actual y `robot_type:=tb4` |
| `TF_OLD_DATA` o `jump back in time` | Procesos viejos o reloj mezclado | `pkill` y relanzar limpio |
| RViz muy pesado | GPU/OpenGL o LaserScan saturando | Usar `LIBGL_ALWAYS_SOFTWARE=1`; apagar LaserScan |

Fallback si el mapa no inicializa y scan/odom si llegan:

```bash
ros2 run slam_gridmap grid_fastslam --ros-args \
  -p use_sim_time:=false \
  -p robot_type:=tb4 \
  -p discard_zero_intensity:=false \
  -p map_save_path:=$MAP_BASE \
  -p autosave_period:=15.0 \
  -r /tf:=/tb4_0/tf \
  -r /tf_static:=/tb4_0/tf_static
```

---

## 11. Como probar en casa el rosbag grabado

Esto no es para correr en el labo salvo que sobre tiempo. Es para cuando el bag
ya esta copiado en una maquina con el repo.

Terminal 1 - SLAM offline:

```bash
cd ~/tp_final/robotica-tp-final
source /opt/ros/humble/setup.bash
source install/setup.bash

mkdir -p ~/tp_final/resultados_rosbag_labo
export MAP_BASE="$HOME/tp_final/resultados_rosbag_labo/mapa_labo_offline"

ros2 run slam_gridmap grid_fastslam --ros-args \
  -p use_sim_time:=true \
  -p robot_type:=tb4 \
  -p map_save_path:=$MAP_BASE \
  -p autosave_period:=15.0
```

Terminal 2 - RViz offline:

```bash
cd ~/tp_final/robotica-tp-final
source /opt/ros/humble/setup.bash
source install/setup.bash

LIBGL_ALWAYS_SOFTWARE=1 QT_OPENGL=software rviz2 -d rviz_rosbag_laberinto.rviz \
  --ros-args -p use_sim_time:=true
```

Terminal 3 - Reproducir bag:

```bash
source /opt/ros/humble/setup.bash

ros2 bag play ~/tp_final/rosbags/NOMBRE_DEL_BAG --clock --rate 1.0 \
  --remap /tb4_0/tf:=/tf /tb4_0/tf_static:=/tf_static
```

Diferencia central:

- Robot real en vivo: `use_sim_time:=false`.
- Rosbag offline: `use_sim_time:=true` y `ros2 bag play --clock`.

Mezclar eso es receta para fantasmas de TF.

---

## 12. Que traer a casa si todo sale mal

Copiar o subir:

- Carpeta del rosbag: `~/tp_final/rosbags/labo_...`
- `~/tp_final/topicos_labo.txt`
- Foto o captura de errores en Terminal 2.
- Si se genero algo de mapa: `~/tp_final/resultados_labo/`
- El ultimo commit usado:

```bash
cd ~/tp_final/robotica-tp-final
git log -1 --oneline
```

Con eso alcanza para reconstruir la escena en casa: que vio el robot, que odom
tenia, que TF habia, y con que codigo se intento mapear.
