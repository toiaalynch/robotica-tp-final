#!/usr/bin/env python3
"""
Modelo de movimiento por odometria (muestreo), de Thrun et al.
==============================================================

MATEMATICA PURA (solo numpy), SIN ROS.

El robot reporta su movimiento entre dos instantes descompuesto en 3 pasos:
    1) rotar  dr1   (orientarse hacia el destino)
    2) avanzar dt   (trasladarse)
    3) rotar  dr2   (orientacion final)
Esta es la misma parametrizacion (dr1, dt, dr2) que usa el TP5 y el mensaje
custom_msgs/DeltaOdom.

La odometria MIENTE un poco (las ruedas patinan, hay error de medicion). El
modelo agrega ruido gaussiano cuyo desvio CRECE con cuanto se roto/avanzo,
gobernado por 4 parametros alpha = (a1, a2, a3, a4):

    sd_rot1  = sqrt(a1*dr1^2 + a2*dt^2)
    sd_trans = sqrt(a3*dt^2  + a4*(dr1^2 + dr2^2))
    sd_rot2  = sqrt(a1*dr2^2 + a2*dt^2)

En el filtro de particulas, cada particula toma una muestra distinta de ese
ruido: por eso la nube de particulas se "abre" en abanico tras moverse, y cada
una representa una hipotesis ligeramente diferente de donde esta el robot.
"""

import numpy as np


def normalize_angle(a):
    """Lleva un angulo al rango (-pi, pi]. Imprescindible al sumar/restar angulos."""
    return np.arctan2(np.sin(a), np.cos(a))


def motion_noise_std(dr1, dt, dr2, alpha):
    """Desvios del ruido (rot1, trans, rot2) para un movimiento (dr1, dt, dr2)."""
    a1, a2, a3, a4 = alpha
    sd_r1 = np.sqrt(a1 * dr1 ** 2 + a2 * dt ** 2)
    sd_t = np.sqrt(a3 * dt ** 2 + a4 * (dr1 ** 2 + dr2 ** 2))
    sd_r2 = np.sqrt(a1 * dr2 ** 2 + a2 * dt ** 2)
    return sd_r1, sd_t, sd_r2


def sample_motion(poses, dr1, dt, dr2, alpha, rng):
    """
    Aplica el modelo de movimiento con ruido a TODAS las particulas a la vez.

    poses : array (N, 3) con (x, y, theta) de cada particula.
    dr1, dt, dr2 : movimiento reportado por la odometria.
    alpha : (a1,a2,a3,a4) parametros de ruido.
    rng   : np.random.Generator.
    Devuelve un nuevo array (N, 3) con las poses movidas (vectorizado = rapido).
    """
    n = poses.shape[0]
    sd_r1, sd_t, sd_r2 = motion_noise_std(dr1, dt, dr2, alpha)

    # cada particula toma su propia muestra de ruido (vectorizado sobre N)
    r1 = dr1 + rng.normal(0.0, sd_r1, n)
    t = dt + rng.normal(0.0, sd_t, n)
    r2 = dr2 + rng.normal(0.0, sd_r2, n)

    x, y, th = poses[:, 0], poses[:, 1], poses[:, 2]
    new = np.empty_like(poses)
    new[:, 0] = x + t * np.cos(th + r1)
    new[:, 1] = y + t * np.sin(th + r1)
    new[:, 2] = normalize_angle(th + r1 + r2)
    return new
