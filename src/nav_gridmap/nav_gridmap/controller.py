#!/usr/bin/env python3
"""
Seguimiento de trayectoria: Pure Pursuit + control de angulo final.
===================================================================

MATEMATICA PURA (solo numpy), SIN ROS. Se testea sola (sim cinematica).

El planificador entrega un camino (lista de waypoints). Este modulo decide, en
cada instante, que velocidad lineal v y angular w mandarle al robot para
seguirlo de forma SUAVE. Usamos Pure Pursuit (Coulter, 1992), el clasico:

  1) Se busca sobre el camino un "punto de mira" (lookahead) a una distancia L
     por delante del robot.
  2) Se calcula el arco de circunferencia que, partiendo de la pose actual,
     pasa por ese punto. Su curvatura es:
            kappa = 2 * y_local / L^2
     donde y_local es la coordenada lateral del punto de mira en el marco del
     robot. De ahi  w = v * kappa.
  3) v se reduce en curvas cerradas y al acercarse al final (perfil trapezoidal
     simple), para que el movimiento sea suave y no se pase del goal.

Maquina interna de 2 fases por tramo:
  - FASE 'drive'  : avanzar siguiendo el camino hasta llegar a la posicion goal.
  - FASE 'rotate' : ya en posicion, girar en el lugar hasta el angulo final.
Esto cumple el requisito de la consigna de llegar tambien al ANGULO final.

Limites por defecto: TurtleBot3 burger (v_max 0.22 m/s, w_max 2.84 rad/s).
"""

import numpy as np


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


class PurePursuitController:
    def __init__(self,
                 lookahead=0.35, min_lookahead=0.20, lookahead_ratio=0.0,
                 v_max=0.22, w_max=2.5, a_lin=0.6,
                 goal_tol=0.10, yaw_tol=0.08,
                 slow_radius=0.45, curve_gain=1.0):
        """
        lookahead      : distancia del punto de mira [m].
        min_lookahead  : cota inferior del lookahead [m].
        lookahead_ratio: si > 0, lookahead = max(min_lookahead, ratio * v) (se
                         adapta a la velocidad). 0 -> lookahead fijo.
        v_max, w_max   : limites de velocidad lineal/angular del robot.
        a_lin          : aceleracion lineal maxima [m/s^2] (suavizado de v).
        goal_tol       : radio para dar por alcanzada la POSICION [m].
        yaw_tol        : tolerancia del ANGULO final [rad].
        slow_radius    : a esta distancia del goal se empieza a frenar [m].
        curve_gain     : cuanto se frena en curvas (mas alto -> mas lento).
        """
        self.L0 = float(lookahead)
        self.min_L = float(min_lookahead)
        self.L_ratio = float(lookahead_ratio)
        self.v_max = float(v_max)
        self.w_max = float(w_max)
        self.a_lin = float(a_lin)
        self.goal_tol = float(goal_tol)
        self.yaw_tol = float(yaw_tol)
        self.slow_radius = float(slow_radius)
        self.curve_gain = float(curve_gain)

        self.reset()

    def reset(self):
        self.path = None              # np.array (M, 2)
        self.goal_yaw = None
        self.phase = 'idle'           # 'idle' | 'drive' | 'rotate' | 'done'
        self._v_prev = 0.0
        self._last_idx = 0

    # ------------------------------------------------------------------
    def set_path(self, waypoints, goal_yaw=None):
        """waypoints: lista de (x, y). goal_yaw: orientacion final deseada [rad]."""
        if waypoints is None or len(waypoints) == 0:
            self.reset()
            return
        self.path = np.asarray(waypoints, dtype=np.float64).reshape(-1, 2)
        self.goal_yaw = None if goal_yaw is None else float(goal_yaw)
        self.phase = 'drive'
        self._v_prev = 0.0
        self._last_idx = 0

    @property
    def active(self):
        return self.phase in ('drive', 'rotate')

    # ------------------------------------------------------------------
    # Calcula (v, w) para la pose actual. dt es el periodo de control [s].
    # Devuelve (v, w, status) con status in {'drive','rotate','done','idle'}.
    # ------------------------------------------------------------------
    def compute(self, pose, dt=0.05):
        if self.path is None or self.phase in ('idle', 'done'):
            return 0.0, 0.0, self.phase

        x, y, th = pose
        goal = self.path[-1]
        dist_goal = np.hypot(goal[0] - x, goal[1] - y)

        # ---- transicion drive -> rotate cuando se alcanza la posicion ----
        if self.phase == 'drive' and dist_goal <= self.goal_tol:
            self.phase = 'rotate' if self.goal_yaw is not None else 'done'
            self._v_prev = 0.0
            if self.phase == 'done':
                return 0.0, 0.0, 'done'

        # ---- FASE rotate: girar en el lugar hasta el angulo final ----
        if self.phase == 'rotate':
            err = _wrap(self.goal_yaw - th)
            if abs(err) <= self.yaw_tol:
                self.phase = 'done'
                return 0.0, 0.0, 'done'
            w = np.clip(2.0 * err, -self.w_max, self.w_max)
            # piso de velocidad para vencer friccion en giros chicos
            if 0 < abs(w) < 0.25:
                w = np.sign(w) * 0.25
            return 0.0, float(w), 'rotate'

        # ---- FASE drive: pure pursuit ----
        L = self.L0
        if self.L_ratio > 0:
            L = max(self.min_L, self.L_ratio * self._v_prev)
        L = max(self.min_L, min(L, max(dist_goal, self.min_L)))

        tx, ty = self._lookahead_point(x, y, L)
        # punto de mira en el marco del robot
        dx, dy = tx - x, ty - y
        x_loc = np.cos(-th) * dx - np.sin(-th) * dy
        y_loc = np.sin(-th) * dx + np.cos(-th) * dy

        # curvatura del arco de pure pursuit
        L_eff = max(np.hypot(x_loc, y_loc), 1e-3)
        kappa = 2.0 * y_loc / (L_eff * L_eff)

        # velocidad lineal: limitar por curva y por cercania al goal
        v = self.v_max
        v /= (1.0 + self.curve_gain * abs(kappa) * L_eff)     # frenar en curvas
        if dist_goal < self.slow_radius:                       # frenar al llegar
            v *= max(dist_goal / self.slow_radius, 0.15)
        # si el objetivo esta muy detras (x_loc<0), reducir y girar fuerte
        if x_loc < 0.0:
            v *= 0.3

        # suavizado de aceleracion lineal
        dv = np.clip(v - self._v_prev, -self.a_lin * dt, self.a_lin * dt)
        v = self._v_prev + dv
        v = float(np.clip(v, 0.0, self.v_max))
        self._v_prev = v

        w = float(np.clip(v * kappa, -self.w_max, self.w_max))
        return v, w, 'drive'

    # ------------------------------------------------------------------
    # Punto de mira: primer punto del camino, recorrido desde el segmento mas
    # cercano hacia adelante, que este a >= L del robot. Si no hay, el final.
    # ------------------------------------------------------------------
    def _lookahead_point(self, x, y, L):
        path = self.path
        # buscar el indice del punto mas cercano (desde el ultimo, no retroceder)
        d2 = (path[:, 0] - x) ** 2 + (path[:, 1] - y) ** 2
        nearest = int(np.argmin(d2))
        nearest = max(nearest, self._last_idx)
        self._last_idx = nearest

        # avanzar acumulando distancia hasta superar L
        for k in range(nearest, len(path) - 1):
            seg_end = path[k + 1]
            if np.hypot(seg_end[0] - x, seg_end[1] - y) >= L:
                # interpolar sobre el segmento [k, k+1] para caer justo a L
                p0 = path[k]
                pt = self._circle_segment_intersection(x, y, L, p0, seg_end)
                return pt if pt is not None else (seg_end[0], seg_end[1])
        return float(path[-1][0]), float(path[-1][1])

    @staticmethod
    def _circle_segment_intersection(cx, cy, r, p0, p1):
        """Interseccion del circulo (centro robot, radio L) con el segmento p0-p1.
        Devuelve el punto mas avanzado dentro del segmento, o None."""
        d = p1 - p0
        f = p0 - np.array([cx, cy])
        a = float(d @ d)
        if a < 1e-9:
            return None
        b = 2.0 * float(f @ d)
        c = float(f @ f) - r * r
        disc = b * b - 4 * a * c
        if disc < 0:
            return None
        disc = np.sqrt(disc)
        t = (-b + disc) / (2 * a)            # raiz mayor = punto mas adelante
        if t < 0.0 or t > 1.0:
            t = (-b - disc) / (2 * a)
        if t < 0.0 or t > 1.0:
            return None
        pt = p0 + t * d
        return float(pt[0]), float(pt[1])
