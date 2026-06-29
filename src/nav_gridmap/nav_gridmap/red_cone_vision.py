#!/usr/bin/env python3
"""
Deteccion offline de conos rojos para la Parte C.
=================================================

Este modulo no depende de ROS: recibe una imagen RGB como ``numpy.ndarray`` y
devuelve la componente roja mas probable. La idea es poder ajustar y testear la
percepcion con imagenes de rosbag o capturas sin tener que levantar todo ROS.

La segmentacion combina dos criterios:
  1) HSV: tono cerca del rojo, saturacion alta y brillo suficiente.
  2) RGB: canal rojo claramente dominante frente a verde y azul.

El segundo criterio hace al detector mas tolerante a cambios de iluminacion del
laboratorio, donde el rojo puede verse oscuro o lavado por reflejos.
"""

from dataclasses import dataclass

import numpy as np
from scipy import ndimage


@dataclass
class RedConeDetection:
    """Resultado de deteccion de un objeto rojo."""

    found: bool
    bbox: tuple = None          # (x, y, w, h)
    center: tuple = None        # (u, v) en pixeles
    area: int = 0
    score: float = 0.0
    mask_fraction: float = 0.0


def _rgb_to_hsv(rgb):
    """Convierte RGB uint8/float a HSV vectorizado.

    Devuelve H en grados [0, 360), S y V en [0, 1]. Implementarlo aca evita una
    dependencia dura de OpenCV/cv_bridge para los tests offline.
    """
    arr = rgb.astype(np.float32) / 255.0
    r, g, b = arr[..., 0], arr[..., 1], arr[..., 2]
    mx = np.max(arr, axis=-1)
    mn = np.min(arr, axis=-1)
    diff = mx - mn

    hue = np.zeros_like(mx)
    nz = diff > 1e-6
    mask = (mx == r) & nz
    hue[mask] = (60.0 * ((g[mask] - b[mask]) / diff[mask]) + 360.0) % 360.0
    mask = (mx == g) & nz
    hue[mask] = 60.0 * ((b[mask] - r[mask]) / diff[mask] + 2.0)
    mask = (mx == b) & nz
    hue[mask] = 60.0 * ((r[mask] - g[mask]) / diff[mask] + 4.0)

    sat = np.zeros_like(mx)
    sat[mx > 1e-6] = diff[mx > 1e-6] / mx[mx > 1e-6]
    return hue, sat, mx


class RedConeDetector:
    def __init__(self, min_area=120, red_min=70, saturation_min=0.35,
                 value_min=0.12, open_size=2, close_size=4):
        self.min_area = int(min_area)
        self.red_min = float(red_min)
        self.saturation_min = float(saturation_min)
        self.value_min = float(value_min)
        self.open_size = int(open_size)
        self.close_size = int(close_size)

    def mask(self, rgb):
        """Devuelve mascara booleana de pixeles compatibles con rojo."""
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("La imagen debe tener forma HxWx3 en RGB.")

        hue, sat, val = _rgb_to_hsv(rgb)
        r = rgb[..., 0].astype(np.float32)
        g = rgb[..., 1].astype(np.float32)
        b = rgb[..., 2].astype(np.float32)

        red_hue = (hue <= 18.0) | (hue >= 342.0)
        hsv_mask = red_hue & (sat >= self.saturation_min) & (val >= self.value_min)

        rgb_mask = ((r >= self.red_min)
                    & (r >= 1.35 * g + 10.0)
                    & (r >= 1.25 * b + 10.0))

        mask = hsv_mask | rgb_mask
        if self.open_size > 0:
            mask = ndimage.binary_opening(mask, structure=np.ones(
                (self.open_size, self.open_size), dtype=bool))
        if self.close_size > 0:
            mask = ndimage.binary_closing(mask, structure=np.ones(
                (self.close_size, self.close_size), dtype=bool))
        return mask

    def detect(self, rgb):
        """Detecta la componente roja dominante."""
        mask = self.mask(rgb)
        labels, n = ndimage.label(mask)
        if n == 0:
            return RedConeDetection(False, mask_fraction=float(mask.mean()))

        objects = ndimage.find_objects(labels)
        best = None
        best_score = -1.0
        for idx, slc in enumerate(objects, start=1):
            if slc is None:
                continue
            ys, xs = slc
            comp = labels[ys, xs] == idx
            area = int(comp.sum())
            if area < self.min_area:
                continue
            h = ys.stop - ys.start
            w = xs.stop - xs.start
            # Los conos aparecen como manchas compactas verticales; aun asi no
            # imponemos una forma estricta para tolerar oclusiones parciales.
            fill = area / max(w * h, 1)
            score = area * (0.5 + fill)
            if score > best_score:
                best_score = score
                best = (xs.start, ys.start, w, h, area, score)

        if best is None:
            return RedConeDetection(False, mask_fraction=float(mask.mean()))

        x, y, w, h, area, score = best
        return RedConeDetection(
            True,
            bbox=(int(x), int(y), int(w), int(h)),
            center=(float(x + 0.5 * w), float(y + 0.5 * h)),
            area=int(area),
            score=float(score),
            mask_fraction=float(mask.mean()),
        )
