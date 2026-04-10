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

def rot_matrix_to_euler(R: np.ndarray) -> np.ndarray:
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)

    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.atan2(-R[2, 0], sy)
    yaw = math.atan2(R[1, 0], R[0, 0])

    return np.array([roll, pitch, yaw])

def normalize_angle(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi
