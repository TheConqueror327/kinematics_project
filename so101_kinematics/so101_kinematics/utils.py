import numpy as np
import math

# translational matrix transformation


def translation_matrix(x: float, y: float, z: float) -> np.ndarray:
    TM = np.identity(4)
    TM[0, 3] = x
    TM[1, 3] = y
    TM[2, 3] = z

    return TM


# rotational matrix transformations around the main axes


def rot_x(angle: float) -> np.ndarray:
    Rx = np.identity(4)
    Rx[1, 1] = math.cos(angle)
    Rx[1, 2] = -math.sin(angle)
    Rx[2, 1] = math.sin(angle)
    Rx[2, 2] = math.cos(angle)

    return Rx


def rot_y(angle: float) -> np.ndarray:
    Ry = np.identity(4)
    Ry[0, 0] = math.cos(angle)
    Ry[2, 0] = -math.sin(angle)
    Ry[0, 2] = math.sin(angle)
    Ry[2, 2] = math.cos(angle)

    return Ry


def rot_z(angle: float) -> np.ndarray:
    Rz = np.identity(4)
    Rz[0, 0] = math.cos(angle)
    Rz[0, 1] = -math.sin(angle)
    Rz[1, 0] = math.sin(angle)
    Rz[1, 1] = math.cos(angle)

    return Rz


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    # URDF standard: Z, Y, X
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def joint_rotation_matrix(axis: list, theta: float) -> np.ndarray:
    """Axes:\n
    x: [1, 0, 0]\n
    y: [0, 1, 0]\n
    z: [0, 0, 1]"""

    if axis == [1, 0, 0]:
        return rot_x(theta)
    elif axis == [0, 1, 0]:
        return rot_y(theta)
    elif axis == [0, 0, 1]:
        return rot_z(theta)
    else:
        raise ValueError("Use only '0' or '1' to define an axis in the list")


def rot_matrix_to_euler(R: np.ndarray) -> np.ndarray:
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)

    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.atan2(-R[2, 0], sy)
    yaw = math.atan2(R[1, 0], R[0, 0])

    return np.array([roll, pitch, yaw])


def normalize_angle(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi
