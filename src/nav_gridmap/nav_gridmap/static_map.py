#!/usr/bin/env python3
"""
Mapa estatico conocido (insumo de la Parte A) para la localizacion.
===================================================================

MATEMATICA PURA (solo numpy/scipy + lectura de archivos), SIN ROS. Se testea sola.

En la Parte A el mapa era DESCONOCIDO y cada particula construia el suyo. En la
Parte B el mapa YA esta hecho (mapa_fastslam_final_v2): es FIJO y COMPARTIDO por
todas las particulas. Este modulo:

  1) Carga el par .yaml + .pgm que dejo map_server en la Parte A.
  2) Arma una grilla con 3 estados por celda: ocupado / libre / desconocido.
  3) Precalcula el LIKELIHOOD FIELD (distancia de cada celda a la pared mas
     cercana), igual que en SLAM pero UNA sola vez (el mapa no cambia).

La clave del diseño: esta clase expone EXACTAMENTE la misma interfaz que el
occupancy_grid de la Parte A necesita para `scan_log_likelihood`
(origin_x, origin_y, resolution, width, height, likelihood_field()), asi
reutilizamos esa funcion tal cual, sin reescribir el modelo del sensor.

Formato map_server (estandar ROS):
  - .yaml: image, resolution, origin [x,y,theta], occupied_thresh, free_thresh, negate.
  - .pgm : imagen en escala de grises. Pixel claro (~254) = libre, oscuro (0) =
           ocupado, gris (~205) = desconocido. La imagen va con el eje Y hacia
           ABAJO, por eso al cargarla la invertimos (flipud) para que la fila 0
           corresponda a y = origin_y (convencion de nav_msgs/OccupancyGrid).
"""

import os

import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt


# Estados de celda (los mismos enteros que usa nav_msgs/OccupancyGrid al estilo
# costmap: -1 desconocido, 0 libre, 100 ocupado, pero aca los manejamos como
# mascaras booleanas internas para claridad).
UNKNOWN = -1
FREE = 0
OCCUPIED = 100


def _read_pgm(path):
    """
    Lee un PGM (P5 binario o P2 ascii) y devuelve un array uint8 (alto, ancho),
    con la fila 0 ARRIBA (tal cual el archivo). Implementacion propia para no
    depender de PIL en el camino critico; si el archivo es raro, cae a PIL.
    """
    with open(path, 'rb') as f:
        data = f.read()

    # --- parser minimo de cabecera PGM (P5/P2), tolerante a comentarios ---
    if not data[:2] in (b'P5', b'P2'):
        # formato no estandar -> intentar con PIL (soporta PNG, etc.)
        from PIL import Image
        img = np.array(Image.open(path).convert('L'), dtype=np.uint8)
        return img

    fmt = data[:2]
    idx = 2
    tokens = []                       # juntamos: ancho, alto, maxval
    while len(tokens) < 3:
        # saltar espacios en blanco
        while idx < len(data) and data[idx:idx + 1].isspace():
            idx += 1
        # saltar comentarios (# ... fin de linea)
        if data[idx:idx + 1] == b'#':
            while idx < len(data) and data[idx:idx + 1] not in (b'\n', b'\r'):
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx:idx + 1].isspace():
            idx += 1
        tokens.append(int(data[start:idx]))
    width, height, maxval = tokens
    idx += 1                          # saltar el unico whitespace tras maxval

    if fmt == b'P5':
        buf = data[idx:idx + width * height]
        img = np.frombuffer(buf, dtype=np.uint8).reshape(height, width)
    else:                             # P2 ascii
        vals = np.array(data[idx:].split(), dtype=np.int32)
        img = vals[:width * height].reshape(height, width).astype(np.uint8)
    return img


class StaticGridMap:
    """
    Mapa de ocupacion estatico y conocido. Compatible con `scan_log_likelihood`.

    Atributos publicos (los que espera el modelo del sensor de la Parte A):
      origin_x, origin_y : esquina inferior-izquierda del mapa [m].
      resolution         : metros por celda.
      width, height      : columnas (i, eje x) y filas (j, eje y).
    """

    def __init__(self, occ, resolution, origin_x, origin_y):
        """
        occ : array int (height, width) con UNKNOWN/FREE/OCCUPIED, fila 0 = y minimo.
        """
        self.occ = occ
        self.height, self.width = occ.shape
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)

        # Likelihood field: distancia [m] de cada celda a la pared mas cercana.
        # El mapa es fijo -> se calcula UNA sola vez en el constructor.
        occupied = (occ == OCCUPIED)
        if occupied.any():
            dist_celdas = distance_transform_edt(~occupied)
            self._lf = (dist_celdas * self.resolution).astype(np.float32)
        else:
            self._lf = np.full(occ.shape, 1e3, dtype=np.float32)

    # --- interfaz que consume scan_log_likelihood (Parte A) ---------------
    def likelihood_field(self):
        return self._lf

    # --- conversiones mundo <-> grilla (misma convencion que la Parte A) --
    def world_to_map(self, x, y):
        i = int((x - self.origin_x) / self.resolution)
        j = int((y - self.origin_y) / self.resolution)
        return i, j

    def in_bounds(self, i, j):
        return 0 <= i < self.width and 0 <= j < self.height

    def is_free(self, i, j):
        """True si la celda (i,j) esta dentro del mapa y es LIBRE."""
        return self.in_bounds(i, j) and self.occ[j, i] == FREE

    # ----------------------------------------------------------------------
    # Cargar desde un par .yaml + .pgm de map_server.
    # ----------------------------------------------------------------------
    @classmethod
    def from_yaml(cls, yaml_path):
        with open(yaml_path, 'r') as f:
            meta = yaml.safe_load(f)

        resolution = float(meta['resolution'])
        origin = meta['origin']
        origin_x, origin_y = float(origin[0]), float(origin[1])
        occ_thresh = float(meta.get('occupied_thresh', 0.65))
        free_thresh = float(meta.get('free_thresh', 0.196))
        negate = int(meta.get('negate', 0))

        # el 'image' del yaml es relativo a la carpeta del yaml
        img_path = meta['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)),
                                    img_path)
        img = _read_pgm(img_path)                       # uint8, fila 0 = arriba

        # map_server interpreta: p_ocupacion = (255 - pixel) / 255  (si negate=0)
        # Asi, pixel oscuro (0)  -> p=1.0 (ocupado); pixel claro (254) -> p~0 (libre).
        p = (255.0 - img.astype(np.float32)) / 255.0
        if negate:
            p = 1.0 - p

        occ = np.full(img.shape, UNKNOWN, dtype=np.int16)
        occ[p > occ_thresh] = OCCUPIED
        occ[p < free_thresh] = FREE
        # (lo que queda entre ambos umbrales permanece UNKNOWN)

        # El PGM tiene la fila 0 ARRIBA (y maximo). nav_msgs/OccupancyGrid usa
        # fila 0 ABAJO (y = origin_y). Invertimos para alinear con world_to_map.
        occ = np.flipud(occ)

        return cls(occ, resolution, origin_x, origin_y)

    # ----------------------------------------------------------------------
    # Salida como lista int8 para publicar en /map (nav_msgs/OccupancyGrid).
    # row-major, fila 0 primero (la convencion de ROS, ya alineada por flipud).
    # ----------------------------------------------------------------------
    def to_occupancy_int8(self):
        data = np.full(self.occ.shape, -1, dtype=np.int8)
        data[self.occ == FREE] = 0
        data[self.occ == OCCUPIED] = 100
        return data.flatten().tolist()
