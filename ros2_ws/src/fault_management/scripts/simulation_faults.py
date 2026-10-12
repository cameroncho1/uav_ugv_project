"""Reproducible fault assumptions; never used to estimate vehicle truth."""

import math
import random

import numpy as np


def common_position_bias(seed, standard_deviation):
    """
    Return one ENU x/y offset shared by GPS and a synthetic UAV anchor for a trial.

    A constant offset is temporally correlated across every reading. Using a
    dedicated seeded stream keeps independent sensor noise sequences unchanged.
    """
    if not math.isfinite(standard_deviation) or standard_deviation < 0:
        raise ValueError('Shared position deviation must be finite and nonnegative')
    generator = random.Random(int(seed) + 37001)
    return np.array([generator.gauss(0, standard_deviation),
                     generator.gauss(0, standard_deviation), 0.0])
