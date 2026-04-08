import numpy as np
import math

def dh_matrix(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    T = np.array([
        [math.cos(theta), - math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
        [math.sin(theta), math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
        [0, math.sin(alpha), math.cos(alpha), d],
        [0, 0, 0, 1]
    ])

    return T

