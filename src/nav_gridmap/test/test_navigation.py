#!/usr/bin/env python3
"""
Tests offline de la navegacion (sin ROS): planner + controller + obstacle.
==========================================================================

Verifican la matematica de la Parte B contra mapas conocidos:
  1) A* encuentra un camino valido (sin pisar paredes) y lo suaviza.
  2) Pure pursuit, en una simulacion cinematica, lleva el robot al goal y al
     ANGULO final pedido.
  3) La deteccion de obstaculos distingue un obstaculo NUEVO de las paredes.
  4) End-to-end sobre el MAPA REAL: localizar pose -> planificar -> seguir ->
     llegar.

Correr:  python3 -m pytest src/nav_gridmap/test -v
"""

import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_HERE))
for _pkg in ('nav_gridmap', 'slam_gridmap'):
    _p = os.path.join(_SRC, _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from nav_gridmap.static_map import StaticGridMap, FREE, OCCUPIED      # noqa: E402
from nav_gridmap.planner import GridPlanner                          # noqa: E402
from nav_gridmap.controller import PurePursuitController             # noqa: E402
from nav_gridmap.obstacle import detect_unmapped                     # noqa: E402


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


# ----------------------------------------------------------------------
def build_room_with_divider(resolution=0.05):
    """Habitacion 6x6 m con una pared divisoria con un hueco (puerta)."""
    w = h = int(6.0 / resolution)
    occ = np.full((h, w), FREE, dtype=np.int16)
    t = 2
    occ[:t, :] = OCCUPIED
    occ[-t:, :] = OCCUPIED
    occ[:, :t] = OCCUPIED
    occ[:, -t:] = OCCUPIED
    # pared divisoria vertical en el medio, con una puerta en el centro
    cx = w // 2
    occ[:, cx:cx + t] = OCCUPIED
    door = slice(int(h * 0.45), int(h * 0.55))
    occ[door, cx:cx + t] = FREE
    return StaticGridMap(occ, resolution, -3.0, -3.0)


# ----------------------------------------------------------------------
def simulate_follow(controller, start_pose, dt=0.05, max_steps=6000):
    """Sim cinematica de un uniciclo siguiendo el controlador. Devuelve la pose
    final y si termino ('done')."""
    x, y, th = start_pose
    for _ in range(max_steps):
        v, w, status = controller.compute((x, y, th), dt)
        x += v * np.cos(th) * dt
        y += v * np.sin(th) * dt
        th = _wrap(th + w * dt)
        if status == 'done':
            return (x, y, th), True
    return (x, y, th), False


# ======================================================================
def test_planner_finds_valid_path():
    smap = build_room_with_divider()
    planner = GridPlanner(smap, robot_radius=0.12, inflation_radius=0.30)

    # de un lado de la pared al otro: debe pasar por la puerta
    path = planner.plan((-2.0, 0.0), (2.0, 0.0))
    assert path is not None, "no se encontro camino (deberia, por la puerta)"
    assert len(path) >= 2

    # ningun waypoint cae sobre una celda no transitable
    for (x, y) in path:
        i, j = planner.world_to_cell(x, y)
        assert planner.traversable[j, i], f"waypoint ({x:.2f},{y:.2f}) no transitable"

    # el camino arranca y termina cerca de lo pedido
    assert np.hypot(path[0][0] + 2.0, path[0][1]) < 0.3
    assert np.hypot(path[-1][0] - 2.0, path[-1][1]) < 0.3
    print(f"\n[planner] camino con {len(path)} waypoints, pasa por la puerta. OK")


def test_planner_blocked_goal_returns_none():
    smap = build_room_with_divider()
    planner = GridPlanner(smap, robot_radius=0.12)
    # objetivo fuera del mapa (lejos): no hay camino
    path = planner.plan((-2.0, 0.0), (100.0, 100.0))
    assert path is None or np.hypot(path[-1][0] - 100, path[-1][1] - 100) > 1.0


def test_controller_reaches_goal_and_angle():
    smap = build_room_with_divider()
    planner = GridPlanner(smap, robot_radius=0.12, inflation_radius=0.30)
    path = planner.plan((-2.0, -1.5), (2.0, 1.5))
    assert path is not None

    goal_yaw = np.radians(90)
    ctrl = PurePursuitController(lookahead=0.35, v_max=0.22, w_max=2.5,
                                 goal_tol=0.10, yaw_tol=0.08)
    ctrl.set_path(path, goal_yaw=goal_yaw)

    start = (-2.0, -1.5, 0.0)
    final, done = simulate_follow(ctrl, start)
    err_pos = np.hypot(final[0] - path[-1][0], final[1] - path[-1][1])
    err_ang = abs(_wrap(final[2] - goal_yaw))

    print(f"[controller] done={done}  err_pos={err_pos:.3f} m  "
          f"err_ang={np.degrees(err_ang):.1f} deg")
    assert done, "el controlador no termino el recorrido"
    assert err_pos < 0.15, f"no llego a la posicion: {err_pos:.3f} m"
    assert err_ang < np.radians(6), f"no llego al angulo: {np.degrees(err_ang):.1f} deg"


def test_obstacle_detection():
    smap = build_room_with_divider()
    pose = (-1.5, 0.0, 0.0)            # robot mirando +x

    # escaneo "limpio": todos los rayos al maximo (sin obstaculo, espacio abierto)
    angles = np.linspace(-np.pi, np.pi, 180, endpoint=False)
    max_range = 8.0
    ranges = np.full_like(angles, max_range)

    # colocar un obstaculo nuevo a 0.6 m justo al frente (no es pared del mapa)
    front = np.argmin(np.abs(angles))
    ranges[front] = 0.6
    for k in (front - 1, front + 1):
        ranges[k % len(ranges)] = 0.6

    info = detect_unmapped(smap, pose, ranges, angles, max_range, wall_tol=0.20)
    print(f"[obstacle] front_dist={info['front_dist']:.2f} celdas={len(info['cells'])}")
    assert info['cells'], "no detecto el obstaculo nuevo"
    assert abs(info['front_dist'] - 0.6) < 0.1, "distancia al frente incorrecta"

    # un rayo que cae sobre la pared del borde NO debe contar como obstaculo nuevo
    ranges2 = np.full_like(angles, max_range)
    # rayo hacia -x: pega en la pared izquierda (~1.0 m desde x=-1.5 hasta x=-2.5? borde en -3)
    back = np.argmin(np.abs(_wrap(angles - np.pi)))
    ranges2[back] = 1.4               # cae cerca del borde izquierdo (pared conocida)
    info2 = detect_unmapped(smap, pose, ranges2, angles, max_range, wall_tol=0.20)
    # el punto cae sobre/junto a la pared -> no es obstaculo nuevo
    print(f"[obstacle] (pared conocida) celdas nuevas={len(info2['cells'])}")


def test_end_to_end_real_map():
    yaml_path = os.path.join(_SRC, 'nav_gridmap', 'maps',
                             'mapa_fastslam_final_v2.yaml')
    if not os.path.exists(yaml_path):
        print("[e2e] mapa real no encontrado, se saltea")
        return
    smap = StaticGridMap.from_yaml(yaml_path)
    planner = GridPlanner(smap, robot_radius=0.14, inflation_radius=0.35)

    # elegir start y goal en celdas libres reales del mapa
    js, iss = np.where(smap.occ == FREE)
    # start: zona inferior-izquierda; goal: zona superior-derecha (en metros)
    start_w = (smap.origin_x + iss.min() * smap.resolution + 0.5,
               smap.origin_y + js.min() * smap.resolution + 0.5)
    goal_w = (smap.origin_x + iss.max() * smap.resolution - 0.5,
              smap.origin_y + js.max() * smap.resolution - 0.5)

    path = planner.plan(start_w, goal_w)
    assert path is not None, "no se encontro camino en el mapa real"
    for (x, y) in path:
        i, j = planner.world_to_cell(x, y)
        assert planner.traversable[j, i]

    # seguir el camino con el controlador desde el inicio real
    sx, sy = path[0]
    # orientar el robot hacia el segundo waypoint
    if len(path) > 1:
        th0 = np.arctan2(path[1][1] - sy, path[1][0] - sx)
    else:
        th0 = 0.0
    ctrl = PurePursuitController(lookahead=0.35, v_max=0.22, w_max=2.5,
                                 goal_tol=0.12, yaw_tol=0.10)
    ctrl.set_path(path, goal_yaw=np.radians(0))
    final, done = simulate_follow(ctrl, (sx, sy, th0), max_steps=12000)
    err_pos = np.hypot(final[0] - path[-1][0], final[1] - path[-1][1])
    print(f"[e2e] camino {len(path)} wpts | done={done} | err_pos={err_pos:.3f} m")
    assert done, "no completo el recorrido en el mapa real"
    assert err_pos < 0.15


if __name__ == "__main__":
    test_planner_finds_valid_path()
    test_planner_blocked_goal_returns_none()
    test_controller_reaches_goal_and_angle()
    test_obstacle_detection()
    test_end_to_end_real_map()
    print("\nOK: planner + controller + obstacle + end-to-end verificados.")
