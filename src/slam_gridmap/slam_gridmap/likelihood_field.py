#!/usr/bin/env python3
"""
Likelihood Field: pesar una particula comparando el LIDAR contra su mapa.
=========================================================================

MATEMATICA PURA (solo numpy), SIN ROS.

Esta es la pieza que responde la pregunta central del filtro:
  "Si el robot estuviera EXACTAMENTE en la pose de esta particula, ¿que tan
   bien coinciden las mediciones del LIDAR con el mapa que esa particula
   construyo hasta ahora?"

Modelo "Likelihood Field" (Thrun, cap. 6.4) — por que se usa en Opcion 1:
  En vez de simular cada rayo celda por celda (lento), se precalcula UNA sola
  vez la distancia de cada celda a la pared mas cercana (eso lo hace el
  occupancy_grid con la transformada de distancia). Despues, pesar una
  particula es solo: proyectar los puntos finales del LIDAR segun su pose,
  mirar a que distancia 'd' caen de una pared, y premiar a la particula cuando
  esos puntos caen JUSTO sobre paredes (d ~ 0).

Probabilidad de un punto final que cae a distancia 'd' de la pared mas cercana:
        p(d) = z_hit * N(d; 0, sigma_hit^2)  +  z_rand
  - N(...) es una gaussiana centrada en 0: vale mucho si d~0 (cae en pared),
    poco si d es grande (cae en el aire) -> esa particula explica mal el scan.
  - z_rand es un piso de probabilidad para mediciones espurias (robustez).

Trabajamos con la SUMA de log(p) de todos los rayos (no el producto), por
estabilidad numerica. El resultado es el log-peso de la particula.
"""

import numpy as np

# distancia que se le asigna a un rayo cuyo extremo cae fuera del mapa
DIST_FUERA_DEL_MAPA = 2.0   # metros (penaliza, pero no mata a la particula)


def scan_log_likelihood(grid, pose, ranges, angles, max_range,
                        sigma_hit=0.20, z_hit=0.85, z_rand=0.15):
    """
    Devuelve el LOG-peso de una particula (escalar) para un escaneo dado.

    grid      : OccupancyGrid2D de ESA particula (su mapa propio).
    pose      : (x, y, theta) del sensor segun la hipotesis de la particula.
    ranges, angles : escaneo del LIDAR ya submuestreado y limpio (sin inf/nan).
    max_range : alcance maximo del LIDAR (los rayos a >= max_range no aportan).
    sigma_hit : ancho de la gaussiana (incertidumbre del sensor), en metros.
    z_hit, z_rand : pesos de la mezcla (hit gaussiano + ruido uniforme).
    """
    lf = grid.likelihood_field()          # distancia a la pared mas cercana [m]
    x, y, theta = pose
    cos_t, sin_t = np.cos(theta), np.sin(theta)

    # solo rayos que efectivamente chocaron contra algo (dentro de alcance)
    mask = ranges < max_range
    r = ranges[mask]
    a = angles[mask]
    if r.size == 0:
        return 0.0                        # sin info util -> no cambia el peso

    # puntos finales de los rayos en coordenadas del mundo (vectorizado)
    ex = x + r * (cos_t * np.cos(a) - sin_t * np.sin(a))
    ey = y + r * (sin_t * np.cos(a) + cos_t * np.sin(a))

    # a indices de grilla
    ii = ((ex - grid.origin_x) / grid.resolution).astype(np.int32)
    jj = ((ey - grid.origin_y) / grid.resolution).astype(np.int32)

    # distancia a la pared mas cercana para cada punto (fuera del mapa -> penaliza)
    dentro = (ii >= 0) & (ii < grid.width) & (jj >= 0) & (jj < grid.height)
    d = np.full(r.shape, DIST_FUERA_DEL_MAPA, dtype=np.float32)
    d[dentro] = lf[jj[dentro], ii[dentro]]

    # p(d) = z_hit * gaussiana(d) + z_rand
    p_hit = np.exp(-(d * d) / (2.0 * sigma_hit * sigma_hit))
    q = z_hit * p_hit + z_rand

    # log-peso = suma de log(p) de todos los rayos
    return float(np.sum(np.log(q)))
