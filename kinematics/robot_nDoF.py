import numpy as np
import math
from .utils import *



class Robot:
    def __init__(self, dh_params: np.ndarray) -> None:
        self.dh_params = dh_params

    
    @classmethod
    def from_urdf(cls, file_path: str) -> None:
        dh_list_temp = []
        from_urdf = [] # export format of urdf2dh: [{"name": "joint1", "a": 0.12, "alpha": 1.507, "d": 0.340, "theta_offset":  1.517}, ...]
        for idx, element in enumerate(from_urdf):
            dh_list_temp[idx] = element


    
    def rot_mat_to_euler(self, R: np.ndarray) -> np.ndarray:
        return rot_matrix_to_euler(R)
    

    def normalize_angle(self, angle: float) -> float:
        return normalize_angle(angle)


    def normalize_angles_array(self, array: np.ndarray) -> np.ndarray:
        return np.array([self.normalize_angle(i) for i in array])


    def FK(self, joint_angles: np.ndarray) -> np.ndarray:
        T_total = np.identity(4)
        for idx, i in enumerate(self.dh_params):
            T_current = dh_matrix(joint_angles[idx], i[1], i[2], i[3])
            T_total = T_total @ T_current
        return T_total
    

    def compute_jacobian(self, joint_angles: np.ndarray, req_dim: int) -> np.ndarray: # required dimension: (X, Y, Z) -> 3 x N matrix, (X, Y, Z, Roll, Pitch, Yaw) -> 6 x N matrix, where N is the number of joints.
        delta = 1e-5

        J = np.zeros((req_dim, len(joint_angles)))

        current_T = self.FK(joint_angles)

        current_pos = current_T[:3, 3]

        current_rot_mat = current_T[:3, :3]
        current_orientation = self.rot_mat_to_euler(current_rot_mat)

        if req_dim == 6:
            current_pose = np.concatenate((current_pos, current_orientation)) # pose = position + orientation
        else:
            current_pose = current_pos


        for i in range(len(joint_angles)):
            joint_angles_copy = list(joint_angles)
            joint_angles_copy[i] += delta
            
            new_T = self.FK(np.array(joint_angles_copy))

            new_pos = new_T[:3, 3]

            new_rot_mat = new_T[:3, :3]
            new_orientation = self.rot_mat_to_euler(new_rot_mat)

            if req_dim == 6:
                new_pose = np.concatenate((new_pos, new_orientation)) # pose = position + orientation
                pose_diff = new_pose - current_pose

                pose_diff[3:] = self.normalize_angles_array(pose_diff[3:]) # to prevent Euler-angles from jumping across a boundary
            else:
                new_pose = new_pos
                pose_diff = new_pose - current_pose

            J[:, i] = pose_diff / delta
            
        return J
    
    
    def IK(self, target_pose: np.ndarray, max_iter: int = 500, tolerance: float = 1e-3, step_size: float = 0.1) -> np.ndarray:
        req_dim = len(target_pose)
        if req_dim not in [3, 6]:
            raise ValueError(f'Invalid target dimension: {target_pose}')
        
        joint_angles = np.zeros(len(self.dh_params)) # general (here 6) case for the number of joints

        for i in range(max_iter):
            current_T = self.FK(joint_angles)

            current_pos = current_T[:3, 3]

            current_rot_mat = current_T[:3, :3]
            current_orientation = self.rot_mat_to_euler(current_rot_mat)

            if req_dim == 6:
                current_pose = np.concatenate((current_pos, current_orientation)) # pose = position + orientation
            else:
                current_pose = current_pos

            error = target_pose - current_pose

            if req_dim == 6:
                error[3:] = self.normalize_angles_array(error[3:]) # to prevent spinning

            if np.linalg.norm(error) < tolerance:
                return joint_angles
            
            J = self.compute_jacobian(joint_angles, req_dim)

            J_pseudo_inv = np.linalg.pinv(J)

            delta_theta = J_pseudo_inv @ error

            joint_angles += delta_theta * step_size

        raise ValueError(f'Failed to converge to the target position [{target_pose[0]}, {target_pose[1]}, {target_pose[2]}] and to the target orientation [{np.rad2deg(target_pose[3])}°, {np.rad2deg(target_pose[4])}°, {np.rad2deg(target_pose[5])}°] within {max_iter} iterations. Final error: {np.linalg.norm(error):.4f}')
