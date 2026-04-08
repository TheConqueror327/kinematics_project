import numpy as np
import math
from .utils import dh_matrix

class Robot:
    def __init__(self, dh_params: np.ndarray) -> None:
        self.dh_params = dh_params


    def forward_kinematics(self, joint_angles: np.ndarray) -> np.ndarray:
        T_total = np.identity(4)
        for idx, i in enumerate(self.dh_params):
            T_current = dh_matrix(joint_angles[idx], i[1], i[2], i[3])
            T_total = T_total @ T_current
        return T_total
    
    
    def IK_analytic(self, target_pos: np.ndarray) -> np.ndarray:
        x, y, z = target_pos

        d1 = self.dh_params[0, 1]
        a2 = self.dh_params[1, 2]
        a3 = self.dh_params[2, 2]

        theta1 = math.atan2(y, x)
    
        r = math.sqrt(x ** 2 + y ** 2)

        z_prime = z - d1

        D = math.sqrt(r ** 2 + z_prime ** 2)

        if D > (a2 + a3):
            raise ValueError(f"The target position ({x}, {y}, {z}) is too far! Workspace radius: {a2+a3}, the given distance: {D:.2f}")
        
        phi = math.atan2(z_prime, r)

        alpha = math.acos((a2 ** 2 + r ** 2 + z_prime ** 2  - a3 ** 2) / (2 * a2 * D))

        theta2 = alpha + phi

        beta = math.acos((a2 ** 2 + a3 ** 2 - D ** 2) / (2 * a2 * a3))

        theta3 = -(math.pi - beta)

        return np.array([theta1, theta2, theta3])
    

    def compute_jacobian(self, joint_angles: np.ndarray) -> np.ndarray:
        delta = 1e-5

        J = np.zeros((3, len(joint_angles)))

        current_pos = self.forward_kinematics(joint_angles)[:3, 3]

        for i in range(len(joint_angles)):
            joint_angles_copy = list(joint_angles)
            joint_angles_copy[i] += delta
            
            new_pos = self.forward_kinematics(np.array(joint_angles_copy))[:3, 3]

            J[:, i] = (new_pos - current_pos) / delta
            
        return J
    
    
    def IK_iterative(self, target_pos: np.ndarray, max_iter: int = 500, tolerance: float = 1e-3, step_size: float = 0.1) -> np.ndarray:
        
        angles = np.array([0.0, 0.0, 0.0])

        target_pos_array = np.array(target_pos)

        for i in range(max_iter):
            current_pos = self.forward_kinematics(angles)[:3, 3]
            error = target_pos_array - current_pos

            if np.linalg.norm(error) < tolerance:
                return angles
            
            J = self.compute_jacobian(angles)

            J_pseudo_inv = np.linalg.pinv(J)

            delta_theta = J_pseudo_inv @ error

            angles += delta_theta * step_size

        raise ValueError(f'Failed to converge to the target position ({target_pos[0]}, {target_pos[1]}, {target_pos[2]}) within {max_iter} iterations. Final error: {np.linalg.norm(error):.4f}')

        return angles
