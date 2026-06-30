#!/usr/bin/env python3
"""
Grilla de ocupacion en log-odds (el "mapa" de UNA particula).
=============================================================

MATEMATICA PURA (solo numpy/scipy), SIN ROS. Asi se testea sola.

En FastSLAM grid-based (Opcion 1) CADA particula lleva SU PROPIO mapa. Este
modulo define ese mapa: una grilla 2D donde cada celda guarda la creencia
(belief) de estar ocupada, en formato log-odds.

Por que log-odds (igual que en el TP5 de mapeo 1D):
  l = log( p / (1-p) )
La actualizacion bayesiana de cada celda se vuelve una SUMA en vez de una
multiplicacion, lo cual es numericamente estable y rapido:
    l_t = l_{t-1} + (aporte del sensor)
  - celda que el rayo ATRAVESO  -> sumamos L_FREE (< 0)  -> mas libre
  - celda donde el rayo CHOCO    -> sumamos L_OCC  (> 0)  -> mas ocupada
Saturamos l en [L_MIN, L_MAX] para que el mapa pueda "olvidar" y no se clave.

Convencion de indices (estandar nav_msgs/OccupancyGrid):
  - La grilla es data[fila, columna] = data[j, i].
  - i (columna) crece con x del mundo; j (fila) crece con y del mundo.
  - La celda (0,0) corresponde al punto del mundo (origin_x, origin_y).
  - resolution = metros por celda.
"""

import numpy as np
from scipy.ndimage import distance_transform_edt


# Conversiones probabilidad <-> log-odds -------------------------------
def prob_to_logodds(p):
    return np.log(p / (1.0 - p))


def logodds_to_prob(l):
    return 1.0 - 1.0 / (1.0 + np.exp(l))


class OccupancyGrid2D:
    """Mapa de ocupacion en log-odds. Es el 'mapa' que lleva cada particula."""

    def __init__(self,
                 width_m=12.0, height_m=12.0, resolution=0.05,
                 origin_x=-6.0, origin_y=-6.0,
                 p_occ=0.70, p_free=0.35, p_prior=0.50,
                 l_clamp=4.0, occ_threshold=0.65):
        """
        width_m, height_m : tamano del mapa en metros.
        resolution        : metros por celda (0.05 = 5 cm, tipico de Gazebo).
        origin_x, origin_y: posicion del mundo de la celda (0,0) (esquina inferior-izq).
        p_occ, p_free     : prob. del modelo inverso del sensor (ocupado / libre).
        l_clamp           : saturacion de log-odds en +/- l_clamp.
        occ_threshold     : prob. a partir de la cual una celda se considera "pared"
                            (se usa para el likelihood field).
        """
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.width = int(round(width_m / resolution))    # columnas (i, eje x)
        self.height = int(round(height_m / resolution))  # filas    (j, eje y)

        # aportes de log-odds del modelo inverso del sensor
        self.L_OCC = prob_to_logodds(p_occ)
        self.L_FREE = prob_to_logodds(p_free)
        self.L_PRIOR = prob_to_logodds(p_prior)   # = 0 si p_prior = 0.5
        self.L_CLAMP = float(l_clamp)
        self.l_occ_threshold = prob_to_logodds(occ_threshold)

        # estado del mapa: log-odds de cada celda, arranca en el prior
        self.log_odds = np.full((self.height, self.width),
                                self.L_PRIOR, dtype=np.float32)

        # cache del likelihood field (distancia a la pared mas cercana, en metros)
        self._lf_cache = None
        self._lf_dirty = True

    # ------------------------------------------------------------------
    # Copia profunda: imprescindible al resamplear para que dos particulas
    # NO compartan el mismo array de mapa.
    # ------------------------------------------------------------------
    def copy(self):
        g = OccupancyGrid2D.__new__(OccupancyGrid2D)
        g.resolution = self.resolution
        g.origin_x = self.origin_x
        g.origin_y = self.origin_y
        g.width = self.width
        g.height = self.height
        g.L_OCC = self.L_OCC
        g.L_FREE = self.L_FREE
        g.L_PRIOR = self.L_PRIOR
        g.L_CLAMP = self.L_CLAMP
        g.l_occ_threshold = self.l_occ_threshold
        g.log_odds = self.log_odds.copy()
        g._lf_cache = None          # el cache no se comparte; se recalcula
        g._lf_dirty = True
        return g

    # ------------------------------------------------------------------
    # Conversiones mundo <-> grilla
    # ------------------------------------------------------------------
    def world_to_map(self, x, y):
        """(x,y) en metros -> (i, j) en celdas (columna, fila). Sin validar limites."""
        i = int((x - self.origin_x) / self.resolution)
        j = int((y - self.origin_y) / self.resolution)
        return i, j

    def in_bounds(self, i, j):
        return 0 <= i < self.width and 0 <= j < self.height

    # ------------------------------------------------------------------
    # Integrar un escaneo del LIDAR al mapa (actualizacion bayesiana).
    # pose  = (x, y, theta) del SENSOR en el mundo.
    # ranges, angles = arrays del LaserScan ya filtrados (sin inf/nan).
    # ------------------------------------------------------------------
    def integrate_scan(self, pose, ranges, angles, max_range,
                       hit_free_margin_m=0.0,
                       occ_update_radius_cells=0,
                       free_update_scale=1.0,
                       occ_update_scale=1.0):
        x, y, theta = pose
        i0, j0 = self.world_to_map(x, y)   # celda del robot (origen de los rayos)
        hit_free_margin_m = max(0.0, float(hit_free_margin_m))
        occ_update_radius_cells = max(0, int(occ_update_radius_cells))
        free_update = self.L_FREE * max(0.0, float(free_update_scale))
        occ_update = self.L_OCC * max(0.0, float(occ_update_scale))

        cos_t, sin_t = np.cos(theta), np.sin(theta)
        for r, a in zip(ranges, angles):
            hit = r < max_range            # False -> el rayo no choco (alcance maximo)
            r_use = min(r, max_range)
            # Si hubo hit, no marcamos como libre hasta la celda pegada al
            # obstaculo. Ese margen evita que rangos apenas ruidosos "borren"
            # paredes ya observadas.
            free_r = max(0.0, r_use - hit_free_margin_m) if hit else r_use
            # punto final del rayo en el mundo
            ca = cos_t * np.cos(a) - sin_t * np.sin(a)
            sa = sin_t * np.cos(a) + cos_t * np.sin(a)
            ex = x + r_use * ca
            ey = y + r_use * sa
            fx = x + free_r * ca
            fy = y + free_r * sa
            i1, j1 = self.world_to_map(ex, ey)
            fi, fj = self.world_to_map(fx, fy)

            # 1) celdas LIBRES: todas las que el rayo atraviesa (Bresenham)
            if free_update != 0.0:
                for (ci, cj) in self._bresenham(i0, j0, fi, fj):
                    self._add_logodds(ci, cj, free_update)

            # 2) celda OCUPADA: solo el extremo, y solo si el rayo realmente choco
            if hit and occ_update != 0.0:
                self._mark_occupied(i1, j1, occ_update, occ_update_radius_cells)

        self._lf_dirty = True          # el mapa cambio -> hay que recalcular el LF

    def _add_logodds(self, i, j, delta):
        if self.in_bounds(i, j):
            self.log_odds[j, i] = np.clip(
                self.log_odds[j, i] + delta,
                -self.L_CLAMP, self.L_CLAMP)

    def _mark_occupied(self, i, j, delta, radius):
        if radius <= 0:
            self._add_logodds(i, j, delta)
            return
        r2 = radius * radius
        for dj in range(-radius, radius + 1):
            for di in range(-radius, radius + 1):
                if di * di + dj * dj <= r2:
                    self._add_logodds(i + di, j + dj, delta)

    # ------------------------------------------------------------------
    # Likelihood field: para cada celda, distancia (en metros) a la pared
    # mas cercana. Se calcula UNA vez con la transformada de distancia
    # (mucho mas rapido que medir rayo por rayo) y se cachea.
    # ------------------------------------------------------------------
    def likelihood_field(self):
        if self._lf_dirty or self._lf_cache is None:
            occupied = self.log_odds > self.l_occ_threshold     # mascara de paredes
            if not occupied.any():
                # mapa todavia vacio: distancia "infinita" en todos lados
                self._lf_cache = np.full(self.log_odds.shape, 1e3, dtype=np.float32)
            else:
                # distancia (en celdas) de cada celda libre a la pared mas cercana
                dist_celdas = distance_transform_edt(~occupied)
                self._lf_cache = (dist_celdas * self.resolution).astype(np.float32)
            self._lf_dirty = False
        return self._lf_cache

    def has_obstacles(self):
        return bool((self.log_odds > self.l_occ_threshold).any())

    # ------------------------------------------------------------------
    # Salida para publicar como nav_msgs/OccupancyGrid:
    #   prob -> entero [0,100], y -1 (desconocido) donde nunca se midio.
    # ------------------------------------------------------------------
    def to_occupancy_int8(self):
        p = logodds_to_prob(self.log_odds)
        data = np.round(p * 100.0).astype(np.int8)
        # celdas que siguen en el prior exacto -> desconocidas (-1)
        desconocido = np.abs(self.log_odds - self.L_PRIOR) < 1e-6
        data[desconocido] = -1
        return data.flatten().tolist()    # row-major (fila 0 primero), como pide ROS

    # ------------------------------------------------------------------
    # Bresenham: lista de celdas de la recta (i0,j0)->(i1,j1) SIN incluir el
    # extremo final (ese se marca como ocupado aparte). Es entero y rapido.
    # ------------------------------------------------------------------
    @staticmethod
    def _bresenham(i0, j0, i1, j1):
        celdas = []
        di = abs(i1 - i0)
        dj = abs(j1 - j0)
        si = 1 if i0 < i1 else -1
        sj = 1 if j0 < j1 else -1
        err = di - dj
        i, j = i0, j0
        while True:
            if i == i1 and j == j1:
                break                      # no incluimos el extremo (es el "hit")
            celdas.append((i, j))
            e2 = 2 * err
            if e2 > -dj:
                err -= dj
                i += si
            if e2 < di:
                err += di
                j += sj
        return celdas
