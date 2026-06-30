#!/usr/bin/env python3
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from slam_gridmap.occupancy_grid import OccupancyGrid2D  # noqa: E402


def _wall_cell(grid):
    return grid.world_to_map(1.0, 0.0)


def _paint_wall(grid, repeats=5):
    pose = (0.0, 0.0, 0.0)
    ranges = np.array([1.0])
    angles = np.array([0.0])
    for _ in range(repeats):
        grid.integrate_scan(pose, ranges, angles, max_range=3.5)


def test_hit_margin_keeps_noisy_hit_from_erasing_existing_wall():
    grid = OccupancyGrid2D(
        width_m=4.0, height_m=4.0, resolution=0.05,
        origin_x=-2.0, origin_y=-2.0, p_occ=0.75, p_free=0.45)
    _paint_wall(grid)
    i, j = _wall_cell(grid)
    before = float(grid.log_odds[j, i])
    assert before > grid.l_occ_threshold

    pose = (0.0, 0.0, 0.0)
    angles = np.array([0.0])
    noisy_long_hit = np.array([1.08])
    for _ in range(12):
        grid.integrate_scan(
            pose, noisy_long_hit, angles, max_range=3.5,
            hit_free_margin_m=0.10, occ_update_scale=0.0)

    after = float(grid.log_odds[j, i])
    assert after == before
    assert after > grid.l_occ_threshold


def test_rotation_mode_can_disable_free_updates():
    grid = OccupancyGrid2D(
        width_m=4.0, height_m=4.0, resolution=0.05,
        origin_x=-2.0, origin_y=-2.0, p_occ=0.75, p_free=0.45)
    _paint_wall(grid)
    i, j = _wall_cell(grid)
    before = float(grid.log_odds[j, i])

    pose = (0.0, 0.0, 0.0)
    angles = np.array([0.0])
    noisy_long_hit = np.array([1.20])
    for _ in range(12):
        grid.integrate_scan(
            pose, noisy_long_hit, angles, max_range=3.5,
            free_update_scale=0.0, occ_update_scale=0.0)

    assert float(grid.log_odds[j, i]) == before


def test_occupied_radius_marks_neighbor_cells():
    grid = OccupancyGrid2D(
        width_m=4.0, height_m=4.0, resolution=0.05,
        origin_x=-2.0, origin_y=-2.0, p_occ=0.75, p_free=0.45)
    pose = (0.0, 0.0, 0.0)
    grid.integrate_scan(
        pose, np.array([1.0]), np.array([0.0]), max_range=3.5,
        free_update_scale=0.0, occ_update_radius_cells=1)

    i, j = _wall_cell(grid)
    assert grid.log_odds[j, i] > grid.l_occ_threshold
    assert grid.log_odds[j, i - 1] > grid.l_occ_threshold
    assert grid.log_odds[j, i + 1] > grid.l_occ_threshold
