#!/usr/bin/env python3
"""Tests offline de percepcion Parte C."""

import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_HERE))
_PKG = os.path.join(_SRC, 'nav_gridmap')
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from nav_gridmap.red_cone_vision import RedConeDetector  # noqa: E402


def test_detects_red_cone_and_ignores_blue_distractor():
    img = np.full((160, 220, 3), 35, dtype=np.uint8)

    # Distractor azul grande.
    img[30:95, 25:85] = np.array([20, 50, 210], dtype=np.uint8)

    # Cono rojo aproximado: triangulo + base.
    for y in range(35, 125):
        half = max(4, int((y - 35) * 0.28))
        cx = 155
        img[y, cx - half:cx + half + 1] = np.array([220, 35, 25], dtype=np.uint8)
    img[120:135, 130:180] = np.array([190, 30, 25], dtype=np.uint8)

    det = RedConeDetector(min_area=80).detect(img)

    assert det.found
    assert det.area > 1000
    assert 140 <= det.center[0] <= 170
    assert det.center[1] > 70


def test_no_detection_for_non_red_scene():
    img = np.full((100, 120, 3), [40, 120, 40], dtype=np.uint8)
    img[30:70, 45:80] = np.array([40, 40, 210], dtype=np.uint8)
    det = RedConeDetector(min_area=50).detect(img)
    assert not det.found
