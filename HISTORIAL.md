# Historial de cambios — TP Final I-402 (Robótica)

Bitácora compartida del grupo. Sirve para que cualquier integrante (o su agente
de IA) pueda ver de un vistazo **qué se hizo hasta el momento** sin leer todo el
código.

---

## Cómo usar este archivo

**Para las personas:** leé el "Estado actual" para el panorama, y el "Historial"
para el detalle cronológico.

**Para la IA (instrucciones):** cada vez que hagas un cambio relevante en el
proyecto, actualizá este archivo:

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
| **0** — Control básico | ⬜ Pendiente | Nodo reactivo (avanzar/rotar). Lo arma el grupo aparte. |
| **A** — SLAM | 🟩 Implementada y verificada (falta probar en Gazebo real) | Paquete `slam_gridmap`: Grid-Based FastSLAM (Opción 1). |
| **B** — Navegación | ⬜ Pendiente | — |
| **C** — Hardware real | ⬜ Pendiente | Se usa TurtleBot4 real. |
| Informe técnico (PDF) | ⬜ Pendiente | — |
| Defensa (diapositivas) | ⬜ Pendiente | — |

**Decisión principal:** para la Parte A se eligió la **Opción 1 (Grid-Based
FastSLAM)** por ser la de menor complejidad conceptual.

**Entorno:** ROS 2 Humble + Gazebo Classic + TurtleBot3 (real: TurtleBot4).

---

## Historial

### 2026-06-23 — Toia (con IA) — Parte A

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
