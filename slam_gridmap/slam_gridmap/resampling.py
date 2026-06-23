#!/usr/bin/env python3
"""
Resampleo del filtro de particulas.
===================================

MATEMATICA PURA (solo numpy), SIN ROS.

Tras pesar las particulas, el resampleo se queda (CON repeticion) con las de
mayor peso: las hipotesis buenas se multiplican, las malas desaparecen. Asi la
nube converge hacia la verdadera pose del robot.

Dos piezas:
  1) N_eff (numero efectivo de particulas): mide cuanto "degenero" el filtro.
     N_eff = 1 / sum(w_i^2).  Si todas pesan igual -> N_eff = N (sano).
     Si una sola domina -> N_eff -> 1 (degenerado). Solo resampleamos cuando
     N_eff cae por debajo de un umbral (evita resamplear de mas, que empobrece
     la diversidad de hipotesis).
  2) Low-variance / systematic resampling (Thrun, tabla 4.4): recorre la ruleta
     de pesos con UN solo numero aleatorio y pasos regulares. Es O(N), de baja
     varianza y mejor que el muestreo multinomial ingenuo.
"""

import numpy as np


def effective_sample_size(weights):
    """N_eff = 1 / sum(w_i^2). 'weights' debe estar normalizado (suma 1)."""
    return 1.0 / np.sum(np.square(weights))


def low_variance_resample(weights, rng):
    """
    Devuelve los INDICES de las particulas elegidas (array de longitud N).
    weights : pesos normalizados (suma 1).
    rng     : np.random.Generator.
    """
    n = len(weights)
    indices = np.zeros(n, dtype=np.int32)
    r = rng.uniform(0.0, 1.0 / n)        # un unico numero aleatorio
    c = weights[0]                       # suma acumulada de pesos
    i = 0
    for m in range(n):
        u = r + m * (1.0 / n)            # puntero que avanza en pasos regulares
        while u > c and i < n - 1:
            i += 1
            c += weights[i]
        indices[m] = i
    return indices
