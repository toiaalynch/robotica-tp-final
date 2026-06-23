# slam_gridmap — Grid-Based FastSLAM (TP Final I-402, Parte A · Opción 1)

Paquete de ROS 2 que implementa **Grid-Based FastSLAM** desde cero: un filtro de
partículas donde **cada partícula mantiene su propio mapa de ocupación** y se
localiza usando **likelihood fields**. Construye el mapa de un laberinto mientras
estima la pose del robot, usando el **TurtleBot3** en **Gazebo** (ROS 2 Humble).

El mapa generado es el entregable de la Parte A y el insumo de localización de
las Partes B y C.

---

## 1. Requisitos

- **ROS 2 Humble** (Ubuntu 22.04 o Robostack).
- **Gazebo Classic** + paquetes de **TurtleBot3** (`turtlebot3_gazebo`,
  `turtlebot3_custom_simulation`, `turtlebot3_teleop`).
- Python: **numpy** y **scipy** (`sudo apt install python3-numpy python3-scipy`).
- `nav2_map_server` (opcional, para guardar el mapa con la herramienta oficial).

---

## 2. Estructura del paquete

```
slam_gridmap/
├── package.xml / setup.py / setup.cfg     # metadata y build (ament_python)
├── config/params.yaml                     # todos los parámetros, comentados
├── launch/slam_gridmap.launch.py          # arranca el SLAM + RViz
├── rviz/slam_gridmap.rviz                  # vista preconfigurada de RViz
├── test/test_grid_fastslam_sim.py         # test offline (valida la matemática)
└── slam_gridmap/
    ├── occupancy_grid.py     # el mapa: grilla en log-odds + likelihood field
    ├── motion_model.py       # modelo de odometría (dr1, dt, dr2) con ruido
    ├── likelihood_field.py   # peso de cada partícula (scan vs. su mapa)
    ├── resampling.py         # N_eff + resampleo low-variance
    ├── grid_fastslam_core.py # el filtro: junta todo (predict/update/resample)
    └── grid_fastslam_node.py # nodo ROS: suscribe, corre el filtro, publica
```

**Idea clave de organización:** el nodo (`grid_fastslam_node.py`) solo coordina
ROS; toda la matemática vive en los módulos de `slam_gridmap/`, que son numpy
puro sin ROS y por eso se pueden testear solos.

---

## 3. Compilar

Copiá la carpeta `slam_gridmap/` a tu workspace de ROS 2 (junto a `custom_msgs`,
`tp5`, etc.) y compilá:

```bash
cd ~/ros2_ws
colcon build --packages-select slam_gridmap --symlink-install
source install/setup.bash
```

---

## 4. Ejecutar (3 terminales)

En cada terminal, primero: `source install/setup.bash` y
`export TURTLEBOT3_MODEL=burger`.

```bash
# Terminal 1 — simulación de Gazebo (laberinto + robot)
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py

# Terminal 2 — teleoperación (manejás el robot para explorar)
ros2 run turtlebot3_teleop teleop_keyboard

# Terminal 3 — el SLAM + RViz
ros2 launch slam_gridmap slam_gridmap.launch.py
```

Manejá el robot lento por todo el laberinto. El mapa se va dibujando en RViz.
Cuando terminaste, hacé **Ctrl+C** en la Terminal 3: el nodo **guarda el mapa
automáticamente** (ver sección 7).

Opciones del launch:

```bash
ros2 launch slam_gridmap slam_gridmap.launch.py num_particles:=50 use_rviz:=true
```

### Portabilidad TurtleBot3 / TurtleBot4

El parámetro **`robot_type`** (`tb3` por defecto) ajusta de una sola vez los
tópicos, el QoS y las particularidades del LIDAR:

| | `tb3` (Gazebo, Parte A) | `tb4` (real, Parte C) |
|---|---|---|
| scan | `/scan` | `/tb4_0/scan` |
| odometría | `/calc_odom` | `/odom` |
| QoS odometría | reliable | **best_effort** |
| intensidad 0 | se conserva | se descarta |
| LIDAR rotado | no | sí (`lidar_angle_offset`) |

```bash
# Para el robot real:
ros2 run slam_gridmap grid_fastslam --ros-args -p robot_type:=tb4
```

Para el TB4 hay que **medir y cargar** `lidar_angle_offset` (rotación real del
LIDAR). El resto se ajusta solo.

---

## 5. Qué se ve en RViz

La configuración (`rviz/slam_gridmap.rviz`) ya trae todo cargado, con
`Fixed Frame = map`:

| Display | Tópico | Qué es |
|---|---|---|
| Map | `/map` | mapa de ocupación de la mejor partícula (en vivo) |
| LikelihoodField | `/likelihoodfield` | distancia a la pared (debug, viene apagado) |
| LaserScan | `/scan` | rayos del LIDAR (puntos amarillos) |
| Particles | `/slam/particles` | la nube de partículas (flechas naranjas) |
| Belief | `/belief` | pose corregida por SLAM (flecha azul) |
| PathSLAM | `/slam/path` | trayectoria estimada por SLAM (azul) |
| PathOdom | `/slam/odom_path` | trayectoria de `/calc_odom` (roja, deriva) |
| PathGroundTruth | `/slam/gt_path` | trayectoria real `/odom` (verde) |

La comparación importante: la trayectoria **azul (SLAM)** debe pegarse a la
**verde (real)**, mientras la **roja (odometría pura)** se va desviando. Eso
demuestra que el filtro corrige la deriva de la odometría.

---

## 6. Parámetros principales (`config/params.yaml`)

| Parámetro | Default | Qué hace |
|---|---|---|
| `num_particles` | 30 | más partículas = mejor mapa pero más lento |
| `alpha` | [0.02]×4 | ruido del modelo de odometría (a1..a4) |
| `map_resolution` | 0.05 | metros por celda |
| `sigma_hit` | 0.20 | incertidumbre del LIDAR en el likelihood field |
| `scan_subsample` | 4 | usa 1 de cada k rayos (clave para tiempo real) |
| `keyframe_dist` / `keyframe_angle` | 0.10 / 0.10 | cada cuánto se corre un paso de SLAM |
| `odom_topic` | `/calc_odom` | odometría estimada (con deriva), según la consigna |

> **Tiempo real (la dificultad de la Opción 1):** el costo crece con
> `num_particles` y baja con `scan_subsample` y los umbrales de keyframe. Si va
> lento, subí `scan_subsample` a 6–8 o bajá `num_particles`. Si el mapa sale
> pobre, hacé lo contrario.

---

## 7. Guardar el mapa

**Automático:** al cerrar el nodo con Ctrl+C, guarda `mapa_slam.pgm` +
`mapa_slam.yaml` en `~/maps/` (formato `map_server`, listo para B y C). Cambiá
la ruta con el parámetro `map_save_path`.

**Manual (herramienta oficial de Nav2), con el nodo corriendo:**

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/mapa_slam
```

---

## 8. Test offline (sin ROS)

Valida que la matemática del filtro converge: simula un robot en una habitación,
genera escaneos por ray-casting y comprueba que la pose estimada sigue a la real
pese al error de odometría.

```bash
cd slam_gridmap
python3 test/test_grid_fastslam_sim.py
# RESULTADO: OK - el filtro localiza y mapea   (error de pose ~0.03 m)
```

---

## 9. Cómo funciona el algoritmo (resumen)

Por cada **keyframe** (cuando el robot se movió lo suficiente):

1. **Predicción** — se mueven todas las partículas con el modelo de odometría
   `(dr1, dt, dr2)` más ruido (cada una imagina un movimiento distinto).
2. **Corrección** — a cada partícula se le da un peso con el **likelihood field**:
   se proyectan los puntos del LIDAR según su pose y se premia a las que caen
   sobre las paredes de *su* mapa.
3. **Mapeo** — se integra el escaneo al mapa de cada partícula (modelo inverso
   del sensor en log-odds, con trazado de rayos de Bresenham).
4. **Resampleo** — si `N_eff` cae por debajo del umbral, se conservan (con
   repetición) las partículas de mayor peso. La nube converge a la pose real.

La mejor partícula da el **mapa final** y la **trayectoria estimada**.

---

## 10. Troubleshooting

- **El LaserScan no aparece en RViz:** el display `/scan` debe tener
  *Reliability = Best Effort* (ya viene así en el `.rviz`).
- **El mapa no aparece:** el display Map usa *Durability = Transient Local*. Si
  abriste RViz antes que el nodo, esperá unos segundos o recargá el display.
- **Todo va muy lento:** subí `scan_subsample` o bajá `num_particles`
  (ver sección 6).
- **El mapa sale “doblado” o con paredes dobles:** la odometría derivó mucho
  entre correcciones. Bajá `keyframe_dist`/`keyframe_angle` o subí partículas.
- **`use_sim_time`:** con Gazebo tiene que estar en `true` (ya viene así).

---

## 11. Notas para el informe (Sim-to-Real / decisiones de diseño)

- Se eligió la **Opción 1** por ser la de menor complejidad conceptual (solo
  LIDAR, en Gazebo), a costa de exigir optimización para correr en tiempo real.
- Decisiones a documentar: nº de partículas vs. velocidad, resolución del mapa,
  parámetros del likelihood field (`sigma_hit`, `z_hit/z_rand`), y la estrategia
  de **keyframes** para hacer el filtro viable en tiempo real.
- El parámetro `alpha` del modelo de odometría se ajusta según cuánta deriva
  muestre `/calc_odom` frente a `/odom`.
