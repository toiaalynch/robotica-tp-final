# Guía de ejecución — SLAM en el robot real (TurtleBot4, sin simulación)

Esta guía es para correr el **Grid-Based FastSLAM (Parte A)** sobre el
**TurtleBot4 físico**, sin Gazebo. La diferencia con la simulación es solo de
**dónde salen los datos de entrada** (`/scan` y `/odom`): el mapa lo sigue
construyendo y publicando **tu nodo**, igual que en la sim.

> Todo se controla con un único parámetro: `robot_type:=tb4`. Eso ajusta solo
> los tópicos, el QoS, el TF namespaceado y `use_sim_time`. No tenés que tocar
> el `params.yaml`.

---

## 0. Antes de empezar

- El TurtleBot4 prendido, en la misma red Wi-Fi que tu PC.
- ROS 2 **Humble** instalado y la misma `ROS_DOMAIN_ID` que el robot.
- El repo compilado en tu PC (ver paso 1).

---

## 1. Compilar (en tu PC)

```bash
cd ~/robotica-tp-final          # tu workspace (donde está src/)
colcon build --packages-select slam_gridmap
source install/setup.bash       # en Mac/zsh: install/setup.zsh
```

> Rebuildeá siempre después de cambiar el código: la carpeta `install/` es una
> copia, no un symlink.

---

## 2. Verificar que llegan los datos del robot

Con el robot encendido, en una terminal (ya con `source install/setup.bash`):

```bash
ros2 topic list
```

Tenés que ver, como mínimo, estos tópicos namespaceados bajo `/tb4_0/`:

```
/tb4_0/scan
/tb4_0/odom
/tb4_0/tf
/tb4_0/tf_static
```

Comprobá que **realmente publican** (que no estén vacíos):

```bash
ros2 topic hz /tb4_0/scan      # debería tirar una frecuencia (~10 Hz)
ros2 topic echo /tb4_0/odom --once
```

> ⚠️ Si los nombres NO son exactamente esos (por ejemplo el namespace no es
> `tb4_0`), avisá: hay que ajustar el perfil `tb4` en `grid_fastslam_node.py`.
> El SLAM no arranca si se suscribe a un tópico que no existe.

---

## 3. Lanzar el SLAM

Una sola terminal:

```bash
ros2 launch slam_gridmap slam_gridmap.launch.py robot_type:=tb4
```

Con `robot_type:=tb4` el launch hace **automáticamente**:

| Cosa | Valor en real (tb4) |
|---|---|
| `scan_topic` | `/tb4_0/scan` |
| `odom_topic` | `/tb4_0/odom` |
| QoS de odom | `best_effort` |
| TF / tf_static | remapeados a `/tb4_0/tf` y `/tb4_0/tf_static` |
| `use_sim_time` | **`false`** (no usa reloj de simulación) |
| ground truth | desactivado (en el robot real no existe) |

No hace falta pasar `use_sim_time`: se pone solo en `false`.

---

## 4. Mover el robot y mapear

En **otra terminal**, teleoperá el robot para que recorra el entorno:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/tb4_0/cmd_vel
```

Manejá **despacio** y haciendo giros suaves. El SLAM corre un paso cada ~10 cm
de avance o ~6° de giro (keyframes).

---

## 5. Ver el mapa en RViz

RViz se abre solo con el launch. Si no, abrilo aparte:

```bash
ros2 run rviz2 rviz2 -d install/slam_gridmap/share/slam_gridmap/rviz/slam_gridmap.rviz
```

Lo que tenés que ver:

- **Map** (`/map`): la grilla de ocupación creciendo a medida que recorrés.
- **Fixed Frame** = `map`.
- La nube de partículas (`/slam/particles`) y la trayectoria (`/slam/path`).

> El mapa se publica en `/map` cada 1 s y queda "latcheado" (QoS
> `transient_local`), así que aunque abras RViz tarde, vas a ver el último mapa.

Para confirmar por consola que el mapa se está publicando:

```bash
ros2 topic hz /map
```

---

## 6. Guardar el mapa

El nodo **autoguarda** el mapa cada 15 s y también al cerrarlo con `Ctrl+C`.
Por defecto queda en `~/maps/`:

```
~/maps/mapa_slam.pgm     # imagen del mapa
~/maps/mapa_slam.yaml    # metadatos (para map_server / Partes B y C)
~/maps/mapa_slam.png     # versión presentable para el informe
```

Para guardarlo en otra ruta, pasá el parámetro al lanzar:

```bash
ros2 launch slam_gridmap slam_gridmap.launch.py robot_type:=tb4 \
  --ros-args -p map_save_path:=/ruta/a/mi_mapa
```

---

## Resumen rápido (3 terminales)

```bash
# Terminal 1 — robot real ya publicando /tb4_0/...  (su propio stack)

# Terminal 2 — SLAM
source install/setup.bash
ros2 launch slam_gridmap slam_gridmap.launch.py robot_type:=tb4

# Terminal 3 — teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/tb4_0/cmd_vel
```

---

## Problemas comunes

| Síntoma | Causa probable | Solución |
|---|---|---|
| El mapa nunca aparece / `/map` no publica | No llega `/tb4_0/odom` o `/tb4_0/scan` | Paso 2: `ros2 topic hz`. Revisar nombres/namespace. |
| RViz: "timestamp earlier than transform cache" o descarta el LIDAR | TF en árbol distinto o `use_sim_time` mal | Confirmar `robot_type:=tb4` (remapea TF y pone `use_sim_time=false`). |
| El mapa sale **rotado** | LIDAR del TB4 montado girado | Medir y setear `lidar_angle_offset` (en rad) en el perfil tb4. |
| Nada se mueve con el teleop | `cmd_vel` sin namespace | Usar `-r cmd_vel:=/tb4_0/cmd_vel`. |
