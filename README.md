# robotica-tp-final

Trabajo Práctico Final de **I-402 — Principios de la Robótica Autónoma** (UdeSA).

Sistema robótico móvil autónomo desarrollado sobre **ROS 2 Humble** con un
**TurtleBot3** en **Gazebo** (y **TurtleBot4** en el robot real). El proyecto
integra percepción, estimación probabilística, mapeo, navegación y control, en
tres etapas evolutivas.

---

## Estado del proyecto

| Parte | Descripción | Estado |
|---|---|---|
| **0** | Control básico del robot (avanzar / rotar ante obstáculos) | ⬜ Pendiente |
| **A** | **SLAM** — mapeo + localización (Grid-Based FastSLAM) | 🟩 Implementada y verificada |
| **B** | Navegación autónoma, planificación y control | ⬜ Pendiente |
| **C** | Despliegue en hardware real (TurtleBot4, misión de conos) | ⬜ Pendiente |
| — | Informe técnico (PDF) | ⬜ Pendiente |
| — | Defensa oral (diapositivas) | ⬜ Pendiente |

> El avance detallado se registra en **[HISTORIAL.md](HISTORIAL.md)**.

---

## Estructura del repositorio

```
robotica-tp-final/
├── README.md           <- este archivo
├── HISTORIAL.md        <- bitácora de cambios del grupo
├── consignas/          <- enunciados (PDF) + diagrama de flujo
└── slam_gridmap/       <- paquete ROS 2 de la Parte A (SLAM)
```

---

## Parte A — Grid-Based FastSLAM (Opción 1)

El paquete `slam_gridmap` implementa, desde cero, un **filtro de partículas donde
cada partícula mantiene su propio mapa de ocupación**, localizándose con
**likelihood fields**. Construye el mapa de un laberinto mientras estima la pose
del robot. El mapa resultante es el insumo de localización de las Partes B y C.

Ejecución rápida (3 terminales; ver detalle en
**[slam_gridmap/README.md](slam_gridmap/README.md)**):

```bash
# Terminal 1 — simulación
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py
# Terminal 2 — teleoperación
ros2 run turtlebot3_teleop teleop_keyboard
# Terminal 3 — SLAM + RViz
ros2 launch slam_gridmap slam_gridmap.launch.py
```

El código es portable entre **TurtleBot3** (simulado) y **TurtleBot4** (real)
mediante el parámetro `robot_type`.

---

## Entorno

- ROS 2 Humble · Gazebo Classic · TurtleBot3 / TurtleBot4
- Python: numpy, scipy, matplotlib

## Autores

Grupo de I-402 — UdeSA.
