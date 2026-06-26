#!/usr/bin/env python3
"""
Test offline de la Localizacion MCL (sin ROS).
==============================================

Verifica la MATEMATICA del filtro contra un mapa sintetico conocido:
construimos una habitacion (con una pared interna que rompe la simetria para
evitar ambiguedad de 180 grados), simulamos el LIDAR por trazado de rayos, y
comprobamos que:

  1) Partiendo de una pose inicial DESPLAZADA respecto de la real, el filtro
     CONVERGE a la pose verdadera a medida que el robot se mueve.
  2) La estimacion final tiene error de posicion < 0.20 m y de angulo < 10 deg.

Correr:  python3 -m pytest src/nav_gridmap/test -v
"""

import os
import sys

import numpy as np

# Permitir importar los paquetes sin ROS (colcon no necesario para este test).
_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_HERE))            # .../src
for _pkg in ('nav_gridmap', 'slam_gridmap'):
    _p = os.path.join(_SRC, _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from nav_gridmap.static_map import StaticGridMap, FREE, OCCUPIED   # noqa: E402
from nav_gridmap.mcl import MonteCarloLocalization                 # noqa: E402


# ----------------------------------------------------------------------
# Construir un mapa sintetico: habitacion 6x6 m con borde + pared interna.
# ----------------------------------------------------------------------
def build_synthetic_map(resolution=0.05):
    w = h = int(6.0 / resolution)                # 120 x 120 celdas
    occ = np.full((h, w), FREE, dtype=np.int16)

    t = 2                                        # grosor de pared (celdas)
    occ[:t, :] = OCCUPIED                        # borde inferior
    occ[-t:, :] = OCCUPIED                       # borde superior
    occ[:, :t] = OCCUPIED                        # borde izquierdo
    occ[:, -t:] = OCCUPIED                       # borde derecho

    # pared interna asimetrica (rompe la simetria del rectangulo)
    occ[h // 2:h // 2 + t, : int(w * 0.6)] = OCCUPIED

    origin = -3.0                                # centro del mapa en (0,0)
    return StaticGridMap(occ, resolution, origin, origin)


# ----------------------------------------------------------------------
# Simular el LIDAR: trazar rayos contra el mapa y devolver (ranges, angles).
# ----------------------------------------------------------------------
def simulate_scan(smap, pose, n_beams=120, max_range=8.0):
    x, y, th = pose
    angles = np.linspace(-np.pi, np.pi, n_beams, endpoint=False)
    ranges = np.full(n_beams, max_range, dtype=np.float64)
    step = smap.resolution * 0.5
    for k, a in enumerate(angles):
        ca, sa = np.cos(th + a), np.sin(th + a)
        d = 0.0
        while d < max_range:
            d += step
            i, j = smap.world_to_map(x + d * ca, y + d * sa)
            if not smap.in_bounds(i, j) or smap.occ[j, i] == OCCUPIED:
                ranges[k] = d
                break
    return ranges, angles


def odom_delta(prev, cur):
    """Delta (dr1, dt, dr2) entre dos poses verdaderas."""
    x0, y0, th0 = prev
    x1, y1, th1 = cur
    dx, dy = x1 - x0, y1 - y0
    dt = np.hypot(dx, dy)
    if dt > 1e-6:
        dr1 = np.arctan2(np.sin(np.arctan2(dy, dx) - th0),
                         np.cos(np.arctan2(dy, dx) - th0))
        dr2 = np.arctan2(np.sin(th1 - th0 - dr1), np.cos(th1 - th0 - dr1))
    else:
        dr1 = 0.0
        dr2 = np.arctan2(np.sin(th1 - th0), np.cos(th1 - th0))
    return dr1, dt, dr2


def test_mcl_converges():
    rng = np.random.default_rng(0)
    smap = build_synthetic_map()
    max_range = 8.0

    # trayectoria verdadera del robot (un recorrido en L por la habitacion)
    waypoints = [(-1.5, -1.5, 0.0)]
    for _ in range(8):
        x, y, th = waypoints[-1]
        waypoints.append((x + 0.4, y, 0.0))
    for _ in range(8):
        x, y, th = waypoints[-1]
        waypoints.append((x, y + 0.35, np.pi / 2))

    mcl = MonteCarloLocalization(smap, num_particles=600,
                                 alpha=(0.02, 0.02, 0.02, 0.02),
                                 sigma_hit=0.20, seed=1)

    # pose inicial DESPLAZADA del valor real (lo que pasaria con un click impreciso)
    tx, ty, tth = waypoints[0]
    mcl.initialize_gaussian(tx + 0.4, ty - 0.3, tth + 0.25,
                            std_xy=0.3, std_theta=0.3)

    # primer scan (correccion sin movimiento)
    ranges, angles = simulate_scan(smap, waypoints[0], max_range=max_range)
    mcl.update(ranges, angles, max_range)

    for prev, cur in zip(waypoints[:-1], waypoints[1:]):
        dr1, dt, dr2 = odom_delta(prev, cur)
        # odometria ruidosa (deriva): el filtro NO recibe la verdad exacta
        mcl.predict(dr1 + rng.normal(0, 0.01),
                    dt + rng.normal(0, 0.01),
                    dr2 + rng.normal(0, 0.01))
        ranges, angles = simulate_scan(smap, cur, max_range=max_range)
        mcl.update(ranges, angles, max_range)

    ex, ey, eth = mcl.estimate()
    fx, fy, fth = waypoints[-1]
    err_pos = np.hypot(ex - fx, ey - fy)
    err_ang = abs(np.arctan2(np.sin(eth - fth), np.cos(eth - fth)))

    print(f"\nPose verdadera : ({fx:.2f}, {fy:.2f}, {np.degrees(fth):.0f} deg)")
    print(f"Pose estimada  : ({ex:.2f}, {ey:.2f}, {np.degrees(eth):.0f} deg)")
    print(f"Error posicion : {err_pos:.3f} m")
    print(f"Error angulo   : {np.degrees(err_ang):.1f} deg")

    assert err_pos < 0.20, f"error de posicion alto: {err_pos:.3f} m"
    assert err_ang < np.radians(10), f"error de angulo alto: {np.degrees(err_ang):.1f} deg"


if __name__ == "__main__":
    test_mcl_converges()
    print("\nOK: el filtro MCL converge a la pose verdadera.")
