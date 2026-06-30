#!/usr/bin/env python3
"""
Nucleo de Grid-Based FastSLAM (Opcion 1 del TP Final).
======================================================

MATEMATICA PURA (solo numpy/scipy), SIN ROS. Se testea sola (ver test/).

Idea (filtro de particulas Rao-Blackwellized, estilo gmapping simplificado):
  - Mantenemos N particulas. CADA particula es una hipotesis COMPLETA:
        * la pose del robot (x, y, theta)
        * SU PROPIO mapa de ocupacion (occupancy grid en log-odds)
    Esto es lo que distingue al grid-based FastSLAM: no hay un mapa unico
    compartido; cada hipotesis de trayectoria construye su propio mapa.

  El ciclo, en cada paso de SLAM (keyframe):
    1) PREDICCION  : movemos todas las particulas con el modelo de odometria
                     + ruido (cada una imagina un movimiento un poco distinto).
    2) CORRECCION  : pesamos cada particula con el likelihood field, comparando
                     el LIDAR contra SU mapa (mejor encaje -> mayor peso).
    3) MAPEO       : integramos el escaneo al mapa de cada particula.
    4) RESAMPLEO   : si el filtro degenero (N_eff bajo), nos quedamos con las
                     particulas de mayor peso (copiando sus mapas).

  Al final, la particula de mayor peso da el mapa + la trayectoria estimados.
"""

import numpy as np

from .occupancy_grid import OccupancyGrid2D
from .motion_model import sample_motion, normalize_angle
from .likelihood_field import scan_log_likelihood
from .resampling import effective_sample_size, low_variance_resample


class GridFastSLAM:
    def __init__(self,
                 num_particles=30,
                 alpha=(0.02, 0.02, 0.02, 0.02),
                 map_args=None,
                 sigma_hit=0.20, z_hit=0.85, z_rand=0.15,
                 neff_ratio=0.5,
                 seed=None):
        """
        num_particles : cantidad de hipotesis (mas = mejor mapa pero mas lento).
        alpha         : (a1..a4) ruido del modelo de movimiento por odometria.
        map_args      : dict con los parametros de OccupancyGrid2D (tamano,
                        resolucion, origen, etc.). Si es None, usa los defaults.
        sigma_hit, z_hit, z_rand : parametros del likelihood field.
        neff_ratio    : resamplea cuando N_eff < neff_ratio * N.
        seed          : semilla del generador aleatorio (reproducibilidad).
        """
        self.N = int(num_particles)
        self.alpha = alpha
        self.sigma_hit = sigma_hit
        self.z_hit = z_hit
        self.z_rand = z_rand
        self.neff_ratio = neff_ratio
        self.rng = np.random.default_rng(seed)

        map_args = map_args or {}
        # poses de las particulas: array (N,3) para mover todo vectorizado
        self.poses = np.zeros((self.N, 3), dtype=np.float64)
        # pesos normalizados
        self.weights = np.full(self.N, 1.0 / self.N)
        # un mapa propio por particula
        self.grids = [OccupancyGrid2D(**map_args) for _ in range(self.N)]

        self.map_initialized = False     # ¿ya integramos el primer escaneo?

        # "foto" de la mejor particula tomada en el momento de la correccion
        # (ANTES de resamplear, cuando los pesos todavia distinguen quien es la
        # mejor). Despues del resampleo los pesos quedan uniformes y argmax ya
        # no sirve, por eso guardamos esta referencia.
        self._best_pose = self.poses[0].copy()
        self._best_grid = self.grids[0]

    # ------------------------------------------------------------------
    # 1) PREDICCION: mover todas las particulas con la odometria + ruido.
    # ------------------------------------------------------------------
    def predict(self, dr1, dt, dr2):
        self.poses = sample_motion(self.poses, dr1, dt, dr2, self.alpha, self.rng)

    # ------------------------------------------------------------------
    # 2-3-4) CORRECCION + MAPEO + RESAMPLEO con un escaneo del LIDAR.
    # ranges, angles : escaneo YA submuestreado y limpio (sin inf/nan).
    # sensor_offset  : (dx, dy) del LIDAR respecto del centro del robot (base).
    # ------------------------------------------------------------------
    def update(self, ranges, angles, max_range, sensor_offset=(0.0, 0.0),
               map_update_options=None):
        sensor_poses = self._sensor_poses(sensor_offset)
        map_update_options = map_update_options or {}

        # --- Primer escaneo: no hay mapa todavia, solo inicializamos ---
        if not self.map_initialized:
            for k in range(self.N):
                self.grids[k].integrate_scan(
                    sensor_poses[k], ranges, angles, max_range,
                    **map_update_options)
            self.map_initialized = True
            self._best_pose = self.poses[0].copy()
            self._best_grid = self.grids[0]
            return

        # --- 2) CORRECCION: peso de cada particula via likelihood field ---
        logw = np.empty(self.N)
        for k in range(self.N):
            logw[k] = scan_log_likelihood(
                self.grids[k], sensor_poses[k], ranges, angles, max_range,
                self.sigma_hit, self.z_hit, self.z_rand)

        # log-pesos -> pesos (restamos el maximo por estabilidad numerica)
        w = np.exp(logw - np.max(logw))
        self.weights *= w
        s = np.sum(self.weights)
        if s <= 0 or not np.isfinite(s):
            self.weights = np.full(self.N, 1.0 / self.N)   # salvavidas numerico
        else:
            self.weights /= s

        # --- 3) MAPEO: integrar el escaneo al mapa de cada particula ---
        for k in range(self.N):
            self.grids[k].integrate_scan(
                sensor_poses[k], ranges, angles, max_range,
                **map_update_options)

        # --- Foto de la mejor particula AHORA (antes de resamplear) ---
        best = int(np.argmax(self.weights))
        self._best_pose = self.poses[best].copy()
        self._best_grid = self.grids[best]

        # --- 4) RESAMPLEO: solo si el filtro degenero ---
        if effective_sample_size(self.weights) < self.neff_ratio * self.N:
            self._resample()

    # ------------------------------------------------------------------
    # Resampleo low-variance. Copia PROFUNDA de los mapas para que dos
    # particulas no terminen compartiendo el mismo array.
    # ------------------------------------------------------------------
    def _resample(self):
        idx = low_variance_resample(self.weights, self.rng)
        self.poses = self.poses[idx].copy()
        self.grids = [self.grids[i].copy() for i in idx]
        self.weights = np.full(self.N, 1.0 / self.N)

    # ------------------------------------------------------------------
    # Pose del SENSOR de cada particula (la pose es del robot/base; el LIDAR
    # puede estar desplazado respecto del centro).
    # ------------------------------------------------------------------
    def _sensor_poses(self, sensor_offset):
        dx, dy = sensor_offset
        out = np.empty_like(self.poses)
        th = self.poses[:, 2]
        out[:, 0] = self.poses[:, 0] + dx * np.cos(th) - dy * np.sin(th)
        out[:, 1] = self.poses[:, 1] + dx * np.sin(th) + dy * np.cos(th)
        out[:, 2] = th
        return out

    # ------------------------------------------------------------------
    # Consultas para el nodo de ROS.
    # ------------------------------------------------------------------
    def best_pose(self):
        """Pose (x, y, theta) de la mejor particula (foto de la ultima correccion)."""
        return self._best_pose.copy()

    def best_grid(self):
        """Mapa (OccupancyGrid2D) de la mejor particula (foto de la ultima correccion)."""
        return self._best_grid

    def neff(self):
        return effective_sample_size(self.weights)
