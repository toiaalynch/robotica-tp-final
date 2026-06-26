#!/usr/bin/env python3
"""
Monte Carlo Localization (MCL) — filtro de particulas sobre un mapa CONOCIDO.
============================================================================

MATEMATICA PURA (solo numpy), SIN ROS. Se testea sola (ver test/).

Es el "primo localizador" del FastSLAM de la Parte A. La diferencia esencial:
  - En SLAM   : el mapa es desconocido y CADA particula construye el suyo.
  - En MCL (B): el mapa es FIJO y COMPARTIDO. Las particulas solo estiman la
                POSE del robot (x, y, theta) dentro de ese mapa conocido.

Por eso reutilizamos directamente, de la Parte A:
  - sample_motion()      -> mover la nube con odometria + ruido (prediccion).
  - scan_log_likelihood()-> pesar cada particula contra el mapa fijo (correccion).
  - low_variance_resample / effective_sample_size -> resampleo sano.

Ciclo MCL (Thrun, cap. 8), en cada paso disparado por movimiento:
  1) PREDICCION : se mueve toda la nube con el delta de odometria + ruido.
  2) CORRECCION : se pesa cada particula comparando el LIDAR contra el mapa.
  3) RESAMPLEO  : si N_eff cae, nos quedamos con las hipotesis buenas.

Extra de ROBUSTEZ — "Augmented MCL" (Thrun, tabla 8.3):
  Se llevan dos promedios del peso, uno lento (w_slow) y uno rapido (w_fast).
  Cuando el peso promedio cae de golpe (el robot "no se reconoce": secuestro,
  mala pose inicial, deriva grande), se inyectan particulas aleatorias en celdas
  libres del mapa para poder RE-LOCALIZARSE. Sin esto, si la nube se va a un mal
  lugar, el filtro nunca se recupera.
"""

import numpy as np

# Reutilizamos los modulos matematicos de la Parte A (paquete slam_gridmap).
from slam_gridmap.motion_model import sample_motion, normalize_angle
from slam_gridmap.likelihood_field import scan_log_likelihood
from slam_gridmap.resampling import effective_sample_size, low_variance_resample


class MonteCarloLocalization:
    def __init__(self,
                 static_map,
                 num_particles=500,
                 alpha=(0.05, 0.05, 0.05, 0.05),
                 sigma_hit=0.20, z_hit=0.85, z_rand=0.15,
                 neff_ratio=0.5,
                 alpha_slow=0.001, alpha_fast=0.1,
                 seed=None):
        """
        static_map  : StaticGridMap (mapa fijo + likelihood field precalculado).
        num_particles: tamano de la nube. En localizacion conviene MAS que en
                       SLAM (300-800): el mapa es fijo y barato de evaluar, y mas
                       particulas dan mejor cobertura para la pose inicial.
        alpha        : (a1..a4) ruido del modelo de movimiento por odometria.
        sigma_hit, z_hit, z_rand : parametros del modelo del sensor (likelihood).
        neff_ratio   : resamplea cuando N_eff < neff_ratio * N.
        alpha_slow, alpha_fast : tasas de los promedios lento/rapido del peso
                       (Augmented MCL). Debe cumplirse 0 <= alpha_slow << alpha_fast.
        """
        self.map = static_map
        self.N = int(num_particles)
        self.alpha = tuple(float(a) for a in alpha)
        self.sigma_hit = float(sigma_hit)
        self.z_hit = float(z_hit)
        self.z_rand = float(z_rand)
        self.neff_ratio = float(neff_ratio)
        self.alpha_slow = float(alpha_slow)
        self.alpha_fast = float(alpha_fast)
        self.rng = np.random.default_rng(seed)

        # Estado del filtro
        self.poses = np.zeros((self.N, 3), dtype=np.float64)   # (x, y, theta)
        self.weights = np.full(self.N, 1.0 / self.N)
        self.initialized = False

        # Promedios del peso para Augmented MCL (en escala LOG, ver update()).
        self.w_slow = 0.0
        self.w_fast = 0.0
        self._w_init = False

        # Precalculo: lista de celdas LIBRES del mapa (para inyectar / sembrar
        # particulas globales sin caer en paredes ni en desconocido).
        js, iss = np.where(self.map.occ == 0)             # FREE
        self._free_cells = np.stack([iss, js], axis=1)    # (M, 2) -> (i, j)

    # ======================================================================
    # Inicializacion
    # ======================================================================
    def initialize_gaussian(self, x, y, theta, std_xy=0.3, std_theta=0.25):
        """
        Siembra la nube como una gaussiana alrededor de (x, y, theta).
        Se usa cuando llega /initialpose (boton 2D Pose Estimate de RViz):
        el usuario da una estimacion y nos localizamos alrededor de ella.
        """
        self.poses[:, 0] = self.rng.normal(x, std_xy, self.N)
        self.poses[:, 1] = self.rng.normal(y, std_xy, self.N)
        self.poses[:, 2] = normalize_angle(
            self.rng.normal(theta, std_theta, self.N))
        self.weights[:] = 1.0 / self.N
        self.w_slow = self.w_fast = 0.0
        self._w_init = False
        self.initialized = True

    def initialize_global(self):
        """
        Localizacion global: nube uniforme sobre TODAS las celdas libres del
        mapa, con orientacion uniforme. Util si no se da pose inicial (o como
        fallback de re-localizacion). Mas dificil de converger que la gaussiana.
        """
        if len(self._free_cells) == 0:
            return
        idx = self.rng.integers(0, len(self._free_cells), self.N)
        cells = self._free_cells[idx]
        # centro de cada celda libre + jitter dentro de la celda
        self.poses[:, 0] = (self.map.origin_x
                            + (cells[:, 0] + self.rng.uniform(0, 1, self.N))
                            * self.map.resolution)
        self.poses[:, 1] = (self.map.origin_y
                            + (cells[:, 1] + self.rng.uniform(0, 1, self.N))
                            * self.map.resolution)
        self.poses[:, 2] = self.rng.uniform(-np.pi, np.pi, self.N)
        self.weights[:] = 1.0 / self.N
        self.w_slow = self.w_fast = 0.0
        self._w_init = False
        self.initialized = True

    # ======================================================================
    # 1) PREDICCION: mover la nube con el delta de odometria + ruido.
    # ======================================================================
    def predict(self, dr1, dt, dr2):
        self.poses = sample_motion(self.poses, dr1, dt, dr2, self.alpha, self.rng)

    # ======================================================================
    # 2) CORRECCION (+ 3 RESAMPLEO): pesar contra el mapa fijo y resamplear.
    # ranges, angles : escaneo YA submuestreado y limpio (sin inf/nan).
    # sensor_offset  : (dx, dy) del LIDAR respecto del centro del robot (base).
    # ======================================================================
    def update(self, ranges, angles, max_range, sensor_offset=(0.0, 0.0)):
        sensor_poses = self._sensor_poses(sensor_offset)

        # --- log-peso de cada particula contra el MISMO mapa estatico ---
        logw = np.empty(self.N)
        for k in range(self.N):
            logw[k] = scan_log_likelihood(
                self.map, sensor_poses[k], ranges, angles, max_range,
                self.sigma_hit, self.z_hit, self.z_rand)

        # log-pesos -> pesos (restar el maximo = estabilidad numerica)
        w = np.exp(logw - np.max(logw))
        self.weights *= w
        s = float(np.sum(self.weights))
        if s <= 0.0 or not np.isfinite(s):
            self.weights = np.full(self.N, 1.0 / self.N)      # salvavidas numerico
            return

        # --- Augmented MCL en escala LOG (numericamente estable) ---
        # Indicador de calidad del encaje: log-likelihood promedio de la nube.
        # Es un numero negativo; cuanto MAS cerca de 0, mejor explica el scan.
        # Trabajamos en log para no subdesbordar (exp de log-pesos muy negativos
        # daria 0). Comparar w_fast vs w_slow (rapido vs lento) detecta caidas
        # bruscas de calidad -> el robot "no se reconoce" -> reinyectar.
        w_avg = float(np.mean(logw))
        if not self._w_init:
            self.w_slow = self.w_fast = w_avg
            self._w_init = True
        else:
            self.w_slow += self.alpha_slow * (w_avg - self.w_slow)
            self.w_fast += self.alpha_fast * (w_avg - self.w_fast)

        self.weights /= s

        # --- 3) RESAMPLEO: solo si el filtro degenero ---
        if effective_sample_size(self.weights) < self.neff_ratio * self.N:
            self._resample()

    # ----------------------------------------------------------------------
    # Resampleo low-variance con inyeccion aleatoria (Augmented MCL).
    # Con probabilidad p_inject reemplazamos una particula por una global, para
    # poder recuperarnos de una mala localizacion.
    # ----------------------------------------------------------------------
    def _resample(self):
        # probabilidad de inyectar una particula aleatoria por slot.
        # w_fast/w_slow estan en LOG -> su diferencia es el log de la razon de
        # calidad reciente/historica; exp() la lleva a escala lineal.
        if self._w_init:
            ratio = float(np.exp(self.w_fast - self.w_slow))
            p_inject = max(0.0, 1.0 - ratio)
        else:
            p_inject = 0.0
        p_inject = min(p_inject, 0.5)            # nunca reinyectar mas del 50%

        idx = low_variance_resample(self.weights, self.rng)
        new_poses = self.poses[idx].copy()

        if p_inject > 0.0 and len(self._free_cells) > 0:
            inject_mask = self.rng.uniform(0, 1, self.N) < p_inject
            n_inj = int(np.count_nonzero(inject_mask))
            if n_inj > 0:
                cidx = self.rng.integers(0, len(self._free_cells), n_inj)
                cells = self._free_cells[cidx]
                new_poses[inject_mask, 0] = (
                    self.map.origin_x
                    + (cells[:, 0] + self.rng.uniform(0, 1, n_inj))
                    * self.map.resolution)
                new_poses[inject_mask, 1] = (
                    self.map.origin_y
                    + (cells[:, 1] + self.rng.uniform(0, 1, n_inj))
                    * self.map.resolution)
                new_poses[inject_mask, 2] = self.rng.uniform(-np.pi, np.pi, n_inj)

        self.poses = new_poses
        self.weights = np.full(self.N, 1.0 / self.N)

    # ----------------------------------------------------------------------
    # Pose del SENSOR de cada particula (pose es del robot/base; el LIDAR puede
    # estar desplazado). Identico criterio que el core de la Parte A.
    # ----------------------------------------------------------------------
    def _sensor_poses(self, sensor_offset):
        dx, dy = sensor_offset
        out = np.empty_like(self.poses)
        th = self.poses[:, 2]
        out[:, 0] = self.poses[:, 0] + dx * np.cos(th) - dy * np.sin(th)
        out[:, 1] = self.poses[:, 1] + dx * np.sin(th) + dy * np.cos(th)
        out[:, 2] = th
        return out

    # ======================================================================
    # Estimacion de pose: media pesada de la nube.
    #   - x, y : promedio pesado directo.
    #   - theta: promedio CIRCULAR (atan2 de las componentes sin/cos pesadas);
    #            NO se puede promediar angulos directamente (wrap en +-pi).
    # ======================================================================
    def estimate(self):
        w = self.weights
        x = float(np.sum(w * self.poses[:, 0]))
        y = float(np.sum(w * self.poses[:, 1]))
        c = float(np.sum(w * np.cos(self.poses[:, 2])))
        s = float(np.sum(w * np.sin(self.poses[:, 2])))
        theta = float(np.arctan2(s, c))
        return x, y, theta

    def covariance_xy(self):
        """Covarianza 2x2 de la posicion (para reportar incertidumbre en RViz)."""
        mx, my, _ = self.estimate()
        dx = self.poses[:, 0] - mx
        dy = self.poses[:, 1] - my
        w = self.weights
        cxx = float(np.sum(w * dx * dx))
        cyy = float(np.sum(w * dy * dy))
        cxy = float(np.sum(w * dx * dy))
        return cxx, cyy, cxy

    def neff(self):
        return effective_sample_size(self.weights)
