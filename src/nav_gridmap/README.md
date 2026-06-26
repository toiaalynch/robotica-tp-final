# nav_gridmap — Parte B (Navegación Autónoma, Sistema 1: grilla pura)

Navegación autónoma del TP Final I-402 sobre el mapa de ocupación generado en la
Parte A (`slam_gridmap`). Como en la Parte A se eligió **Opción 1 (Grid-Based
FastSLAM)**, esta parte usa el **Sistema 1: navegación basada en grilla pura**.

El paquete se construye **por componentes**. Estado actual:

| Componente | Estado | Archivo |
|---|---|---|
| **Localización (MCL)** | 🟩 Listo | `mcl.py`, `static_map.py`, `mcl_node.py` |
| **Planificación de ruta (A\*)** | 🟩 Listo | `planner.py` |
| **Seguimiento (pure pursuit)** | 🟩 Listo | `controller.py` |
| **Evasión de obstáculos** | 🟩 Listo | `obstacle.py` |
| **Máquina de estados** | 🟩 Listo | `navigator_node.py` |

> Reutiliza por dependencia los módulos matemáticos de la Parte A
> (`motion_model`, `likelihood_field`, `resampling`), sin duplicar código.

---

## Cómo correr la navegación completa

```bash
# 0) compilar el workspace (desde la raíz del repo)
colcon build --symlink-install && source install/setup.bash

# 1) simulación: entorno estándar  o  con obstáculos
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py
#   o:  ros2 launch turtlebot3_custom_simulation custom_casa_obs.launch.py

# 2) teleop (solo para la localización inicial)
ros2 run turtlebot3_teleop teleop_keyboard

# 3) navegación completa (localización + planner + control + FSM + RViz)
ros2 launch nav_gridmap navigation.launch.py
```

En **RViz**:

1. **2D Pose Estimate** → marcá dónde está el robot (siembra la localización).
2. Mové un poco con el teleop hasta que la nube azul **converja**; cerrá el teleop.
3. **2D Goal Pose** → marcá destino y orientación. El robot planifica (línea
   magenta `/plan`) y va solo, frenando y replanificando si aparece un obstáculo.
4. Para ir a otro lado, marcá un **nuevo goal** (incluso a mitad de camino):
   replanifica automáticamente.

### Máquina de estados

`WAIT_GOAL → PLANNING → FOLLOWING → REACHED → WAIT_GOAL`, con re-planificación
ante obstáculos no mapeados o un nuevo goal. El estado se publica en `/nav_state`.

### Tópicos del navegador

| Dirección | Tópico | Tipo |
|---|---|---|
| sub | `/goal_pose` | `geometry_msgs/PoseStamped` (2D Goal Pose) |
| sub | `/scan` | `sensor_msgs/LaserScan` |
| sub (TF) | `map → base_footprint` | pose del robot (la da MCL) |
| pub | `/cmd_vel` | `geometry_msgs/Twist` |
| pub | `/plan` | `nav_msgs/Path` |
| pub | `/nav_state` | `std_msgs/String` |

Parámetros en `config/navigation.yaml` (radio del robot, inflado, lookahead,
velocidades, tolerancias, distancias de evasión, etc.).

---

## Componente 1 — Localización por Monte Carlo (MCL)

Filtro de partículas que estima la pose del robot dentro del **mapa ya conocido**.
A diferencia del SLAM (donde cada partícula construía su propio mapa), acá el mapa
es **fijo y compartido**: las partículas solo estiman la pose `(x, y, θ)`.

**Ciclo:** predicción (mover la nube con la odometría + ruido) → corrección
(pesar cada partícula comparando el LIDAR contra el mapa fijo) → resampleo (si el
filtro degenera). Incluye **MCL aumentado**: si el robot "no se reconoce" (mala
pose inicial, deriva), reinyecta partículas para **re-localizarse**.

### Tópicos

| Dirección | Tópico | Tipo |
|---|---|---|
| sub | `/scan` | `sensor_msgs/LaserScan` |
| sub | `/calc_odom` | `nav_msgs/Odometry` |
| sub | `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` |
| pub | `/map` | `nav_msgs/OccupancyGrid` (latched) |
| pub | `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| pub | `/particlecloud` | `geometry_msgs/PoseArray` |
| TF | `map → odom` | corrección de la localización |

### Cómo correrlo

```bash
# 0) compilar el workspace (desde la raíz del repo)
colcon build --symlink-install
source install/setup.bash

# 1) simulación (entorno estándar)
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py

# 2) teleoperación (para mover el robot y que la nube converja)
ros2 run turtlebot3_teleop teleop_keyboard

# 3) localización + RViz
ros2 launch nav_gridmap localization.launch.py
```

En **RViz**: click en **“2D Pose Estimate”** y luego click + arrastrar sobre el
mapa, en la posición y sentido reales del robot. La nube azul (`/particlecloud`)
se siembra ahí; al mover el robot debe **converger** sobre su pose real, y la
flecha verde (`/amcl_pose`) seguirlo.

### Argumentos útiles

```bash
ros2 launch nav_gridmap localization.launch.py robot_type:=tb4   # robot real
ros2 launch nav_gridmap localization.launch.py map:=/ruta/mapa.yaml
ros2 launch nav_gridmap localization.launch.py rviz:=false
```

Parámetros en `config/localization.yaml` (partículas, ruido, modelo del sensor,
keyframes, etc.).

### Test offline (sin ROS)

```bash
python3 -m pytest src/nav_gridmap/test -v
```

Verifica, sobre un mapa sintético, que el filtro converge a la pose verdadera.
