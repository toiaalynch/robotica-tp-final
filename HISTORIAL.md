# Historial de cambios — TP Final I-402 (Robótica)

Bitacora compartida del grupo. Sirve para que cualquier integrante pueda ver
de un vistazo **que se hizo hasta el momento** sin leer todo el codigo.

---

## Cómo usar este archivo

**Para las personas:** leé el "Estado actual" para el panorama, y el "Historial"
para el detalle cronológico.

**Para mantener la bitacora:** cada vez que se haga un cambio relevante en el
proyecto, actualizar este archivo:

1. Agregá una entrada **arriba de todo** en la sección "Historial" (orden
   cronológico inverso: lo más nuevo primero), con el formato:
   `### AAAA-MM-DD — <autor> — <parte>` y una lista breve de los cambios.
2. Actualizá la sección "Estado actual" si cambió el panorama general.
3. Sé conciso: qué se hizo y por qué, no pegues código. Una persona tiene que
   poder entender el avance en 30 segundos.
4. Si te piden "resumime lo hecho hasta ahora", respondé a partir de este archivo.

---

## Estado actual

| Parte | Estado | Detalle |
|---|---|---|
| **A** — SLAM | 🟩 Implementada y verificada (falta probar en Gazebo real) | Paquete `slam_gridmap`: Grid-Based FastSLAM (Opción 1). |
| **B** — Navegación | 🟩 Implementada y verificada en Gazebo headless | Paquete `nav_gridmap` (Sistema 1, grilla pura). Localización MCL + planificación A* + seguimiento pure pursuit + evasión + máquina de estados. Tests offline OK y smoke test en Gazebo OK. |
| **C** — Hardware real | 🟨 En progreso | Percepción de conos rojos inicial agregada; falta validar con rosbag/cámara real. |
| Informe técnico (PDF) | ⬜ Pendiente | — |
| Defensa (diapositivas) | ⬜ Pendiente | — |

**Decisión principal:** para la Parte A se eligió la **Opción 1 (Grid-Based
FastSLAM)** por ser la de menor complejidad conceptual.

**Entorno:** ROS 2 Humble + Gazebo Classic + TurtleBot3 (real: TurtleBot4).

---

## Historial

### 2026-06-29 — Alan — Parte C

- Agregado `red_cone_mission`, nodo de percepción que detecta conos rojos en
  imagen RGB, estima una coordenada en `map` y publica `/red_cone/goal_pose` o
  `/goal_pose` si se activa `auto_goal`.
- Agregado `red_cone_vision.py` con segmentación offline testeable para ajustar
  umbrales con rosbags antes de ir al laboratorio.
- Incorporados `red_cone_mission.launch.py` y `config/perception.yaml` para
  configurar tópicos de cámara, depth, CameraInfo y modo automático.
- Corregida la portabilidad de odometría: TB3 simulado usa `/calc_odom` y
  `calc_base_footprint`; TB4 real usa `/odom` con QoS `best_effort`.
- Los launch de Gazebo/headless ahora levantan el nodo de `calc_odom` para que
  la simulación no dependa de correrlo manualmente en otra terminal.
- Pendiente: la Parte C todavía no fue validada con rosbag de visión ni con
  cámara real; por ahora se verificó compilación y tests offline de segmentación.

### 2026-06-28 — Alan — Parte B

- Generado el mapa de navegación **`mapa_fastslam_final_v2_nav`** a partir del
  V2 original. Se conserva `mapa_fastslam_final_v2` como resultado de SLAM de
  Parte A, y la copia `v2_nav` agrega cuatro puntos ocupados en las patas de la
  mesa del entorno simulado.
- Actualizados `localization.launch.py` y `navigation.launch.py` para usar por
  defecto el mapa navegable `v2_nav`, manteniendo la opción `map:=...` para
  cargar otro mapa si se desea.
- Mejorada la máquina de estados del navegador con estado **`RECOVERY`** ante
  atasco: si no hay progreso hacia el objetivo, gira brevemente, releva
  obstáculos con LIDAR y re-planifica. También se limpia la capa dinámica al
  recibir un goal nuevo.
- Agregada suite `scripts/run_nav_goal_suite.sh` para validar navegación
  headless con goals repetidos, salida por `/nav_state`, `/cmd_vel`, `/plan` y
  logs por objetivo. Puede activarse el stress de mesa con `RUN_STRESS=true`.
- **Verificado:** build OK, tests offline `6/6 OK`; en Gazebo headless con
  `v2_nav`, la suite estándar llega a 3/3 goals y la corrida con
  `RUN_STRESS=true` llega a 4/4 goals, incluyendo el objetivo cercano a la mesa.

### 2026-06-27 — Alan — Parte B

- Revisada e integrada la subida de Parte B (`nav_gridmap`) realizada el
  2026-06-26: localizacion MCL, planificacion A*, seguimiento pure pursuit,
  evasion de obstaculos y maquina de estados.
- En ese momento se había probado MCL con `/odom` en Gazebo; el 2026-06-29 se
  corrigió para cumplir la consigna: TB3 simulado usa `/calc_odom`.
- Corregida la deteccion de obstaculos dinamicos: ahora solo marca obstaculos
  nuevos sobre celdas que el mapa conoce como libres. Las celdas desconocidas
  del mapa ya no generan falsos positivos de evasion/replanificacion.
- Agregado `scripts/run_nav_gridmap_smoke.sh` para probar automaticamente la
  integracion: levanta Gazebo headless, navegacion sin RViz, publica
  `/initialpose` y `/goal_pose`, y verifica `/amcl_pose`, `/plan`,
  `/nav_state` y `/cmd_vel`.
- Verificado localmente: `colcon build --packages-select nav_gridmap`
  correcto; `python3 -m pytest src/nav_gridmap/test -q` con 6/6 tests OK; smoke
  test en Gazebo headless OK (`PLAN_OK`, estado `FOLLOWING` y objetivo
  alcanzado).

### 2026-06-26 — Toia — Parte B

- Creado el paquete ROS 2 **`nav_gridmap`** (carpeta `src/nav_gridmap`) para la
  navegación autónoma. Como la Parte A usó Opción 1, se trabaja con el
  **Sistema 1: navegación basada en grilla pura**. Se construye por componentes.
- **Componente 1 — Localización (MCL) listo y verificado.** Filtro de partículas
  de localización pura contra el **mapa fijo** de la Parte A
  (`mapa_fastslam_final_v2`). A diferencia del SLAM, el mapa es compartido y solo
  se estima la pose. Reutiliza por dependencia los módulos de la Parte A
  (`motion_model`, `likelihood_field`, `resampling`), sin duplicar código.
- Arquitectura modular (numpy puro, sin ROS, testeable sola): `static_map.py`
  (carga el `.pgm`/`.yaml` y precalcula el likelihood field), `mcl.py` (filtro +
  **MCL aumentado** para re-localizarse si el robot se pierde); la plomería ROS
  en `mcl_node.py`.
- El nodo publica el mapa en `/map` (latched), escucha `/initialpose` (botón *2D
  Pose Estimate* de RViz), corre el filtro por keyframes con `/scan` + `/odom`,
  y publica `/amcl_pose`, `/particlecloud` y el TF **`map→odom`** (la corrección
  que usarán el planificador y el control). Nombres de tópicos al estilo Nav2/AMCL
  para que RViz enchufe sin configuración extra.
- Agregados: `launch/localization.launch.py` (resuelve solo la ruta del mapa),
  `config/localization.yaml`, `rviz/localization.rviz`, `README.md` y un test
  offline. Copia del mapa en `maps/` para que el launch lo encuentre tras instalar.
- **Verificado:** sintaxis OK; el test sintético converge a la pose verdadera
  (error ~0.015 m / 0.2°) partiendo de una pose inicial desplazada; el cargador
  lee el mapa real (320×320, 16×16 m) con la orientación correcta.
- **Componentes 2-5 — Planificación + Seguimiento + Evasión + FSM listos y
  verificados.** Se completó todo el stack de navegación:
  - `planner.py`: **A\*** sobre la grilla con **costmap inflado** (transformada
    de distancia): celdas a menos del radio del robot son letales, gradiente de
    costo cerca de paredes (rutas centradas), desconocido intransitable;
    suavizado por línea de visión. Re-planificación con obstáculos dinámicos.
  - `controller.py`: **pure pursuit** suave (lookahead + curvatura), frenado en
    curvas y al llegar, y control del **ángulo final** (gira en el lugar).
    Límites de TB3 burger.
  - `obstacle.py`: detecta obstáculos **no mapeados** (separa lo nuevo de las
    paredes conocidas), parada de seguridad + marcado para re-planificar.
  - `navigator_node.py`: **máquina de estados**
    `WAIT_GOAL→PLANNING→FOLLOWING→REACHED`, escucha `/goal_pose`, pose por TF
    `map→base`, publica `/cmd_vel`, `/plan` y `/nav_state`. Soporta **nuevo goal
    a mitad de camino** (re-planifica) y **evasión** (frena, marca, re-planifica).
- Integración: `launch/navigation.launch.py` (localización + navegador + RViz),
  `config/navigation.yaml`, `rviz/navigation.rviz`, entry points `navigator`.
- **Verificado (tests offline, 6/6 OK):** A\* halla rutas válidas que no pisan
  paredes; pure pursuit lleva el robot al goal y al ángulo final en sim
  cinemática (error ~0.10 m / ~4°); la evasión distingue obstáculo nuevo de
  pared; **end-to-end en el mapa real** (esquina a esquina de la casa, 5
  waypoints) completa el recorrido. Falta probar en Gazebo real.

### 2026-06-25 — Alan — Parte A

- Generado y seleccionado el mapa final de SLAM **`mapa_fastslam_final_v2`**
  como insumo para la Parte B. Se guarda en `src/slam_gridmap/maps/` en formato
  `map_server` (`.pgm` + `.yaml`) y con una vista `.png` para informe/defensa.
- Se compararon varias corridas alternativas (refuerzos locales, ajuste de
  `p_free`/`p_occ` y movimiento continuo por `/cmd_vel` con waypoints). Aunque
  algunas reducian una incertidumbre local en la esquina inferior izquierda,
  empeoraban la consistencia global del mapa.
- Criterio de seleccion: V2 mantiene mejor equilibrio entre paredes exteriores
  rectas, pared central sin duplicaciones severas, objetos principales
  reconocibles y topologia apta para planificacion.
- Ajustado `free_thresh` del `.yaml` a `0.196` para conservar las celdas
  grises del `.pgm` como desconocidas al cargar el mapa, evitando que zonas no
  exploradas sean interpretadas como libres por el planificador.
- Queda documentado que la mesa/sofa puede verse parcialmente por el LIDAR 2D:
  el sensor detecta las partes que intersectan su plano de escaneo (por ejemplo,
  patas/bordes), no necesariamente la tapa completa del objeto.

### 2026-06-23 — Toia — Parte A

- Creado el paquete ROS 2 **`slam_gridmap`** (carpeta `TPF/slam_gridmap`) que
  implementa **Grid-Based FastSLAM** desde cero (Opción 1).
- Arquitectura modular: la matemática (numpy puro, sin ROS) va en
  `occupancy_grid.py`, `motion_model.py`, `likelihood_field.py`,
  `resampling.py` y `grid_fastslam_core.py`; la "plomería" de ROS en
  `grid_fastslam_node.py`.
- El nodo consume `/scan` y `/calc_odom`, corre el filtro por *keyframes* (para
  tiempo real) y publica `/map`, `/likelihoodfield`, `/belief`,
  `/slam/particles`, las trayectorias y el TF `map→odom`. Guarda el mapa
  automáticamente al cerrar (formato `map_server`, insumo de B y C).
- Agregados: `launch/slam_gridmap.launch.py`, `rviz/slam_gridmap.rviz`,
  `config/params.yaml`, `README.md` y un test offline.
- **Verificado:** sintaxis OK; el test sintético converge (error de pose
  ~0.03 m) y el mapa generado reproduce el entorno.
- Agregado parámetro **`robot_type` (tb3/tb4)**: ajusta automáticamente tópicos,
  QoS (best_effort para la odometría del TB4), offset angular del LIDAR y
  descarte de lecturas con intensidad 0. Deja el código portable al robot real.
- Creado este `HISTORIAL.md`.
- El nodo ahora también guarda al cerrar un **PNG presentable** del mapa
  (`save_png`, default true): paredes negras / libre blanco / desconocido gris,
  con las trayectorias real, odometría pura y SLAM superpuestas — listo para el
  informe.
- Reorganización del repo a estructura de **workspace** (`src/`): los paquetes
  ROS van bajo `src/`, con README de portada y `consignas/` para los PDFs.
- Robustez: agregado **autoguardado periódico** del mapa (`autosave_period`,
  default 15 s) para no depender de cerrar con Ctrl+C. Además, launch
  `casa_headless.launch.py` para correr Gazebo sin la ventana 3D (que crashea en
  Mac M-series por OpenGL/Metal).
