#!/usr/bin/env python3
"""
Test sintetico del nucleo Grid-Based FastSLAM (sin ROS).
========================================================
No reemplaza probarlo en Gazebo, pero valida que la MATEMATICA del filtro es
correcta: si funciona, la pose estimada por la mejor particula debe SEGUIR a la
pose verdadera a pesar del error de odometria, gracias a la correccion por
likelihood field contra el mapa que el propio filtro va construyendo.

Mundo de prueba: una habitacion rectangular con un obstaculo interior. El LIDAR
se simula por ray-casting contra ese mundo verdadero.

Correr:  python3 test/test_grid_fastslam_sim.py
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from slam_gridmap.grid_fastslam_core import GridFastSLAM
from slam_gridmap.motion_model import normalize_angle


# ----------------------------------------------------------------------
# Mundo verdadero: True donde hay pared.
# ----------------------------------------------------------------------
def is_wall(x, y):
    if abs(x) > 2.5 or abs(y) > 2.5:              # paredes de la habitacion
        return True
    if -2.3 <= x <= -1.6 and -2.3 <= y <= -1.6:   # obstaculo interior (esquina)
        return True
    return False


def simulate_scan(pose, n_beams, max_range, rng, noise=0.02):
    """Genera (ranges, angles) por ray-casting desde 'pose' contra el mundo."""
    x, y, th = pose
    angles = np.linspace(-np.pi, np.pi, n_beams, endpoint=False)
    ranges = np.full(n_beams, max_range)
    step = 0.02
    for k, a in enumerate(angles):
        ca, sa = np.cos(th + a), np.sin(th + a)
        d = step
        while d < max_range:
            if is_wall(x + d * ca, y + d * sa):
                ranges[k] = d + rng.normal(0, noise)
                break
            d += step
    return ranges, angles


def main():
    rng = np.random.default_rng(0)

    # filtro con un mapa chico (habitacion ~5x5) para que el test corra rapido
    map_args = dict(width_m=8.0, height_m=8.0, resolution=0.05,
                    origin_x=-4.0, origin_y=-4.0)
    filt = GridFastSLAM(num_particles=30, alpha=(0.01, 0.01, 0.01, 0.01),
                        map_args=map_args, sigma_hit=0.2, neff_ratio=0.5, seed=2)

    max_range = 6.0
    n_beams = 90
    drift = 0.01                  # error real de la odometria

    # El filtro arranca en (0,0,0): el robot verdadero TAMBIEN, asi comparten
    # frame y la pose estimada es comparable con la verdadera.
    # Con v/w = 1.0 el robot describe un circulo de radio 1 -> queda dentro de
    # la habitacion (+/-2.5) sin pisar el obstaculo de la esquina.
    true = np.array([0.0, 0.0, 0.0])
    prev = true.copy()
    v, w = 0.12, 0.12             # velocidad lineal y angular por paso

    pose_errors = []
    for step in range(70):
        # --- mover el robot verdadero (modelo de unicycle) ---
        true[0] += v * np.cos(true[2])
        true[1] += v * np.sin(true[2])
        true[2] = normalize_angle(true[2] + w)

        # --- delta de odometria (decomposicion dr1, dt, dr2) + deriva ---
        dx, dy = true[0] - prev[0], true[1] - prev[1]
        dt = np.hypot(dx, dy)
        dr1 = normalize_angle(np.arctan2(dy, dx) - prev[2])
        dr2 = normalize_angle(true[2] - prev[2] - dr1)
        odr1 = dr1 + rng.normal(0, drift)
        odt = dt + rng.normal(0, drift)
        odr2 = dr2 + rng.normal(0, drift)

        # --- escaneo del LIDAR desde la pose verdadera ---
        ranges, angles = simulate_scan(true, n_beams, max_range, rng)

        # --- paso de SLAM ---
        filt.predict(odr1, odt, odr2)
        filt.update(ranges, angles, max_range)

        best = filt.best_pose()
        err = np.hypot(best[0] - true[0], best[1] - true[1])
        pose_errors.append(err)
        prev = true.copy()

    final_err = pose_errors[-1]
    mean_err = float(np.mean(pose_errors[10:]))   # ignorar el transitorio inicial

    print("=== Grid-Based FastSLAM - test sintetico ===")
    print(f"  pose verdadera final : x={true[0]:+.3f} y={true[1]:+.3f} th={true[2]:+.3f}")
    b = filt.best_pose()
    print(f"  pose estimada  final : x={b[0]:+.3f} y={b[1]:+.3f} th={b[2]:+.3f}")
    print(f"  error de pose final  : {final_err:.3f} m")
    print(f"  error de pose medio  : {mean_err:.3f} m")
    print(f"  N_eff final          : {filt.neff():.1f} / {filt.N}")
    print(f"  celdas ocupadas mapa : {(filt.best_grid().log_odds > filt.best_grid().l_occ_threshold).sum()}")

    ok = final_err < 0.30 and mean_err < 0.30
    print("\nRESULTADO:", "OK - el filtro localiza y mapea" if ok else "FALLO - revisar")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
