#!/usr/bin/env python3
"""
Deteccion de obstaculos NO mapeados (capa reactiva).
====================================================

MATEMATICA PURA (solo numpy), SIN ROS. Se testea sola.

El mapa de la Parte A es estatico: si aparece algo que no estaba (una caja, una
persona, otro robot), el LIDAR lo ve pero el mapa no lo tiene. Este modulo:

  1) Proyecta los puntos del LIDAR al mundo segun la pose estimada.
  2) Descarta los que caen sobre paredes YA conocidas del mapa (con tolerancia):
     esos son el entorno normal, no un obstaculo nuevo.
  3) Lo que queda y cae en una celda que el mapa cree LIBRE es un obstaculo
     DINAMICO -> devuelve esas celdas (para re-planificar marcandolas) y la
     distancia al obstaculo mas cercano AL FRENTE (para frenar por seguridad).

Asi el navegador puede: (a) frenar de inmediato si algo esta muy cerca adelante,
y (b) marcar el obstaculo en la grilla y pedir una ruta nueva que lo rodee.
"""

import numpy as np

from .static_map import FREE, OCCUPIED


def project_scan(pose, ranges, angles, max_range):
    """Devuelve los puntos finales (x, y) del LIDAR en el mundo (solo los que
    chocaron, es decir range < max_range)."""
    x, y, th = pose
    mask = np.isfinite(ranges) & (ranges < max_range)
    r = ranges[mask]
    a = angles[mask]
    ex = x + r * np.cos(th + a)
    ey = y + r * np.sin(th + a)
    return ex, ey, a, r


def detect_unmapped(static_map, pose, ranges, angles, max_range,
                    wall_tol=0.20, front_fov=np.pi / 3.0):
    """
    static_map : StaticGridMap (mapa conocido).
    pose       : (x, y, theta) estimada del robot (sensor).
    ranges, angles : escaneo del LIDAR (limpio).
    wall_tol   : si un punto cae a <= wall_tol [m] de una pared conocida, se
                 considera parte del mapa (no es obstaculo nuevo).
    front_fov  : apertura del cono frontal para medir la distancia de seguridad.

    Devuelve: dict con
      'cells'        : lista de celdas (i, j) con obstaculo dinamico.
      'front_dist'   : distancia [m] al obstaculo dinamico mas cercano al frente
                       (np.inf si no hay).
      'min_dist'     : distancia [m] al obstaculo dinamico mas cercano en 360.
    """
    ex, ey, a, r = project_scan(pose, ranges, angles, max_range)
    if r.size == 0:
        return {'cells': [], 'front_dist': np.inf, 'min_dist': np.inf}

    lf = static_map.likelihood_field()        # distancia a la pared conocida [m]
    ii = ((ex - static_map.origin_x) / static_map.resolution).astype(np.int32)
    jj = ((ey - static_map.origin_y) / static_map.resolution).astype(np.int32)

    inside = ((ii >= 0) & (ii < static_map.width)
              & (jj >= 0) & (jj < static_map.height))

    # distancia de cada punto a la pared conocida mas cercana
    dwall = np.full(r.shape, 1e3, dtype=np.float32)
    dwall[inside] = lf[jj[inside], ii[inside]]

    # Un punto es obstaculo NUEVO si cae dentro de una celda que el mapa creia
    # LIBRE. No marcamos UNKNOWN como obstaculo dinamico: en este TP el mapa de
    # SLAM conserva zonas desconocidas y tratarlas como obstaculos nuevos genera
    # falsos positivos/re-planificaciones espurias.
    is_new = inside & (dwall > wall_tol)
    if np.any(inside):
        known_occ = np.zeros(r.shape, dtype=bool)
        known_occ[inside] = (static_map.occ[jj[inside], ii[inside]] == OCCUPIED)
        known_free = np.zeros(r.shape, dtype=bool)
        known_free[inside] = (static_map.occ[jj[inside], ii[inside]] == FREE)
        is_new &= known_free & ~known_occ

    if not np.any(is_new):
        return {'cells': [], 'front_dist': np.inf, 'min_dist': np.inf}

    cells = list(zip(ii[is_new].tolist(), jj[is_new].tolist()))

    # distancia al obstaculo nuevo mas cercano (en 360 y al frente)
    rn = r[is_new]
    an = a[is_new]                            # angulo relativo al robot
    min_dist = float(np.min(rn))
    front = np.abs(np.arctan2(np.sin(an), np.cos(an))) <= (front_fov / 2.0)
    front_dist = float(np.min(rn[front])) if np.any(front) else np.inf

    # quitar celdas duplicadas conservando orden
    cells = list(dict.fromkeys(cells))
    return {'cells': cells, 'front_dist': front_dist, 'min_dist': min_dist}
