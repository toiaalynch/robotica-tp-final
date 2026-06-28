#!/usr/bin/env python3
"""
Planificacion de ruta sobre la grilla (A* con costmap inflado).
===============================================================

MATEMATICA PURA (solo numpy/scipy), SIN ROS. Se testea sola.

Dado el mapa estatico (Parte A), una pose de inicio y una de objetivo, calcula
un camino factible que:
  - NO atraviesa paredes ni celdas desconocidas,
  - se mantiene LEJOS de los obstaculos (no pasa pegado), via "inflado",
  - es lo mas corto posible dentro de esas restricciones (A*).

Dos ideas centrales:

1) COSTMAP INFLADO (igual filosofia que el costmap de Nav2).
   Con la transformada de distancia calculamos, para cada celda, la distancia a
   la pared mas cercana. Entonces:
     - dist < robot_radius              -> LETAL (el robot no entra: chocaria).
     - robot_radius <= dist < inflation -> transitable PERO con costo extra que
       decae con la distancia: A* preferira el centro de los pasillos.
     - celda DESCONOCIDA                -> no transitable (no sabemos que hay).

2) A* (Hart, Nilsson, Raphael).
   Busqueda de costo minimo con heuristica admisible (distancia euclidea * costo
   minimo de paso). Conectividad 8 (incluye diagonales, con su costo sqrt(2)).

Al final, el camino en celdas se suaviza por "linea de vision" (string pulling):
se eliminan waypoints intermedios mientras el segmento recto entre dos puntos no
atraviese una celda con costo letal. Eso da un camino con menos quiebres, mas
comodo de seguir para el controlador.
"""

import heapq

import numpy as np
from scipy.ndimage import distance_transform_edt

from .static_map import OCCUPIED, FREE


# costo de moverse a una celda vecina (orto / diagonal)
_ORTHO = 1.0
_DIAG = np.sqrt(2.0)
_NEIGHBORS = [(-1, 0, _ORTHO), (1, 0, _ORTHO), (0, -1, _ORTHO), (0, 1, _ORTHO),
              (-1, -1, _DIAG), (-1, 1, _DIAG), (1, -1, _DIAG), (1, 1, _DIAG)]


class GridPlanner:
    def __init__(self, static_map,
                 robot_radius=0.14, inflation_radius=0.35,
                 inflation_cost=8.0, allow_unknown=False):
        """
        static_map     : StaticGridMap (mapa fijo).
        robot_radius   : radio del robot [m]. Celdas a menor distancia de una
                         pared son letales. TB3 burger ~0.105 m; usamos un poco
                         mas por margen de seguridad.
        inflation_radius: hasta esta distancia [m] de una pared se penaliza
                         (pero se puede pasar). Mas alla, costo normal.
        inflation_cost : cuanto se penaliza estar pegado a una pared (en celdas
                         equivalentes). Mas alto -> rutas mas centradas.
        allow_unknown  : si False, las celdas desconocidas son intransitables.
        """
        self.map = static_map
        self.res = static_map.resolution
        self.robot_radius = float(robot_radius)
        self.inflation_radius = float(inflation_radius)
        self.inflation_cost = float(inflation_cost)
        self.allow_unknown = bool(allow_unknown)
        self._build_costmap()

    # ------------------------------------------------------------------
    # Construye el costmap inflado a partir del mapa estatico.
    # ------------------------------------------------------------------
    def _build_costmap(self):
        occ = self.map.occ
        obstacle = (occ == OCCUPIED)

        # distancia [m] de cada celda a la pared mas cercana
        if obstacle.any():
            dist = distance_transform_edt(~obstacle) * self.res
        else:
            dist = np.full(occ.shape, 1e3, dtype=np.float64)
        self.dist_to_wall = dist.astype(np.float32)

        # transitabilidad
        free_like = (occ == FREE)
        if self.allow_unknown:
            free_like = free_like | (occ != OCCUPIED)
        traversable = free_like & (dist >= self.robot_radius)
        self.traversable = traversable

        # costo extra por cercania a la pared (decae linealmente hasta inflation)
        extra = np.zeros(occ.shape, dtype=np.float32)
        band = (dist >= self.robot_radius) & (dist < self.inflation_radius)
        denom = max(self.inflation_radius - self.robot_radius, 1e-6)
        extra[band] = self.inflation_cost * (
            1.0 - (dist[band] - self.robot_radius) / denom)
        # costo de pisar cada celda = 1 (paso base) + extra por inflado
        self.cell_cost = (1.0 + extra).astype(np.float32)

    # ------------------------------------------------------------------
    # Helpers de conversion (centro de celda).
    # ------------------------------------------------------------------
    def world_to_cell(self, x, y):
        i = int((x - self.map.origin_x) / self.res)
        j = int((y - self.map.origin_y) / self.res)
        return i, j

    def cell_to_world(self, i, j):
        x = self.map.origin_x + (i + 0.5) * self.res
        y = self.map.origin_y + (j + 0.5) * self.res
        return x, y

    def _in_bounds(self, i, j):
        return 0 <= i < self.map.width and 0 <= j < self.map.height

    # ------------------------------------------------------------------
    # Si una celda no es transitable (p.ej. el click cayo sobre una pared o muy
    # cerca), busca la celda transitable mas cercana dentro de un radio. Hace al
    # planificador robusto a starts/goals imperfectos.
    # ------------------------------------------------------------------
    def nearest_traversable(self, i, j, max_cells=30):
        if self._in_bounds(i, j) and self.traversable[j, i]:
            return i, j
        for r in range(1, max_cells + 1):
            best = None
            best_d = None
            for dj in range(-r, r + 1):
                for di in range(-r, r + 1):
                    if max(abs(di), abs(dj)) != r:
                        continue            # solo el "anillo" de radio r
                    ni, nj = i + di, j + dj
                    if self._in_bounds(ni, nj) and self.traversable[nj, ni]:
                        d = di * di + dj * dj
                        if best_d is None or d < best_d:
                            best_d = d
                            best = (ni, nj)
            if best is not None:
                return best
        return None

    # ------------------------------------------------------------------
    # A*: devuelve una lista de waypoints (x, y) en el mundo, o None si no hay
    # camino. start/goal en coordenadas del mundo.
    # ------------------------------------------------------------------
    def plan(self, start_xy, goal_xy, simplify=True):
        si, sj = self.world_to_cell(*start_xy)
        gi, gj = self.world_to_cell(*goal_xy)

        start = self.nearest_traversable(si, sj)
        goal = self.nearest_traversable(gi, gj)
        if start is None or goal is None:
            return None
        if start == goal:
            return [self.cell_to_world(*goal)]

        cost = self.cell_cost
        trav = self.traversable
        W, H = self.map.width, self.map.height

        def h(i, j):                         # heuristica admisible (euclidea)
            return np.hypot(i - goal[0], j - goal[1])

        open_heap = [(h(*start), 0.0, start)]
        g_score = {start: 0.0}
        came_from = {}
        closed = set()

        while open_heap:
            f, gc, cur = heapq.heappop(open_heap)
            if cur in closed:
                continue
            if cur == goal:
                return self._reconstruct(came_from, cur, simplify)
            closed.add(cur)
            ci, cj = cur
            for di, dj, step in _NEIGHBORS:
                ni, nj = ci + di, cj + dj
                if not (0 <= ni < W and 0 <= nj < H) or not trav[nj, ni]:
                    continue
                # evitar "cortar esquinas" entre dos paredes en diagonal
                if di != 0 and dj != 0:
                    if not trav[cj, ni] or not trav[nj, ci]:
                        continue
                # costo del paso ponderado por el costo de la celda destino
                new_g = gc + step * cost[nj, ni]
                nb = (ni, nj)
                if new_g < g_score.get(nb, np.inf):
                    g_score[nb] = new_g
                    came_from[nb] = cur
                    heapq.heappush(open_heap, (new_g + h(ni, nj), new_g, nb))

        return None                          # no se encontro camino

    # ------------------------------------------------------------------
    def _reconstruct(self, came_from, cur, simplify):
        cells = [cur]
        while cur in came_from:
            cur = came_from[cur]
            cells.append(cur)
        cells.reverse()
        if simplify:
            cells = self._string_pull(cells)
        return [self.cell_to_world(i, j) for (i, j) in cells]

    # ------------------------------------------------------------------
    # Suavizado por linea de vision: quita waypoints intermedios mientras el
    # segmento recto entre el ultimo waypoint conservado y el candidato siguiente
    # no atraviese una celda no transitable.
    # ------------------------------------------------------------------
    def _string_pull(self, cells):
        if len(cells) <= 2:
            return cells
        out = [cells[0]]
        anchor = 0
        for k in range(1, len(cells)):
            if not self._line_clear(cells[anchor], cells[k]):
                out.append(cells[k - 1])
                anchor = k - 1
        out.append(cells[-1])
        return out

    def _line_clear(self, a, b):
        """True si el segmento a->b (en celdas) no toca celdas no transitables."""
        (i0, j0), (i1, j1) = a, b
        di, dj = abs(i1 - i0), abs(j1 - j0)
        si = 1 if i0 < i1 else -1
        sj = 1 if j0 < j1 else -1
        err = di - dj
        i, j = i0, j0
        trav = self.traversable
        while True:
            if not trav[j, i]:
                return False
            if i == i1 and j == j1:
                return True
            e2 = 2 * err
            if e2 > -dj:
                err -= dj
                i += si
            if e2 < di:
                err += di
                j += sj

    # ------------------------------------------------------------------
    # Re-planificacion con obstaculos dinamicos: marca celdas extra como letales
    # de forma temporal, planifica, y restaura el costmap. 'extra_cells' es un
    # iterable de (i, j).
    # ------------------------------------------------------------------
    def plan_with_dynamic(self, start_xy, goal_xy, extra_cells,
                          inflate_cells=3, simplify=True):
        inflate_cells = max(0, int(inflate_cells))
        if not extra_cells:
            return self.plan(start_xy, goal_xy, simplify=simplify)

        saved_trav = self.traversable.copy()
        saved_cost = self.cell_cost.copy()
        try:
            for (i, j) in extra_cells:
                for dj in range(-inflate_cells, inflate_cells + 1):
                    for di in range(-inflate_cells, inflate_cells + 1):
                        ni, nj = i + di, j + dj
                        if self._in_bounds(ni, nj):
                            if di * di + dj * dj <= inflate_cells * inflate_cells:
                                self.traversable[nj, ni] = False
            return self.plan(start_xy, goal_xy)
        finally:
            self.traversable = saved_trav
            self.cell_cost = saved_cost
