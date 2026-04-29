import numpy as np
import math
from .utils import *
import xml.etree.ElementTree as ET
from typing import Self


class Robot:
    def __init__(self, kinematic_chain: list) -> None:
        self.kinematic_chain = kinematic_chain

    # Static method for parsing a string to a list:  "0 1.2 -3.5" -> [0, 1.2, -3.5]

    @staticmethod
    def _parse_string_to_floats(txt: str) -> list:
        return [float(value) for value in txt.split()]
    
    # Factory method for importing a robot from an URDF file with debugging
    
    @classmethod
    def from_urdf_debug(cls, file_path: str, base_link: str = "base", end_link: str = "gripper"):
        tree = ET.parse(file_path)
        root = tree.getroot()

        joints_by_child = {}
        
        all_joints = list(root.iter('joint'))
        print(f"\n[DEBUG X-RAY] Fájl: {file_path}")
        print(f"[DEBUG X-RAY] Gyökér (Root) elem: {root.tag}")
        print(f"[DEBUG X-RAY] Összesen {len(all_joints)} db <joint> taget találtam a fájlban!")
        print("-" * 40)

        for joint in all_joints:
            name = joint.get('name', 'Unknown name')
            j_type = joint.get('type')
            print(f" -> Vizsgálom: '{name}' (Típus: {j_type})")

            if j_type not in ['revolute', 'continuous', 'fixed']:
                print(f"    [KIDOBVA] Nem megfelelő típus: {j_type}")
                continue

            parent_tag = joint.find('parent')
            child_tag = joint.find('child')
            
            if parent_tag is None or child_tag is None:
                print(f"    [KIDOBVA] Hiányzik a <parent> vagy <child> tag!")
                continue
            
            parent_link = parent_tag.get('link')
            child_link = child_tag.get('link')
            
            print(f"    [ELFOGADVA] Szülő: {parent_link} -> Gyerek: {child_link}")

            origin_tag = joint.find('origin')

            if origin_tag is not None:
                xyz = cls._parse_string_to_floats(origin_tag.get('xyz', '0 0 0'))
                rpy = cls._parse_string_to_floats(origin_tag.get('rpy', '0 0 0'))
            else:
                xyz, rpy = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

            axis_tag = joint.find('axis')
            if axis_tag is not None:
                axis = cls._parse_string_to_floats(axis_tag.get('xyz', '1 0 0'))
            else:
                axis = [1.0, 0.0, 0.0]

            joints_by_child[child_link] = {
                'name': name,
                'parent': parent_link,
                'child': child_link,
                'type': j_type,
                'xyz': xyz,
                'rpy': rpy,
                'axis': axis
            }

        print("-" * 40)
        print(f"[DEBUG X-RAY] Number of joints: {len(joints_by_child)}")
        print(f"[DEBUG X-RAY] Child links: {list(joints_by_child.keys())}\n")
        

        chain = []

        current_link = end_link

        while current_link != base_link:
            if current_link not in joints_by_child:
                raise ValueError(f'Open chain! Cannot find a joint which is connected to {current_link}')
            
            joint_data = joints_by_child[current_link]
            chain.append(joint_data)

            current_link = joint_data["parent"]

        chain.reverse()

        return cls(chain)

    # Factory method for importing a robot from an URDF file

    @classmethod
    def from_urdf(cls, file_path: str, base_link: str = "base", end_link: str = "gripper") -> Self:
        tree = ET.parse(file_path)
        root = tree.getroot()

        joints_by_child = {}
        
        all_joints = list(root.iter('joint'))

        for joint in all_joints:
            name = joint.get('name', 'Unknown name')
            j_type = joint.get('type')

            if j_type not in ['revolute', 'continuous', 'fixed']:
                continue

            parent_tag = joint.find('parent')
            child_tag = joint.find('child')
            
            if parent_tag is None or child_tag is None:
                continue
            
            parent_link = parent_tag.get('link')
            child_link = child_tag.get('link')
            

            origin_tag = joint.find('origin')

            if origin_tag is not None:
                xyz = cls._parse_string_to_floats(origin_tag.get('xyz', '0 0 0'))
                rpy = cls._parse_string_to_floats(origin_tag.get('rpy', '0 0 0'))
            else:
                xyz, rpy = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

            axis_tag = joint.find('axis')
            if axis_tag is not None:
                axis = cls._parse_string_to_floats(axis_tag.get('xyz', '1 0 0'))
            else:
                axis = [1.0, 0.0, 0.0]

            joints_by_child[child_link] = {
                'name': name,
                'parent': parent_link,
                'child': child_link,
                'type': j_type,
                'xyz': xyz,
                'rpy': rpy,
                'axis': axis
            }

        chain = []

        current_link = end_link

        while current_link != base_link:
            if current_link not in joints_by_child:
                raise ValueError(f'Open chain! Cannot find a joint which is connected to {current_link}')
            
            joint_data = joints_by_child[current_link]
            chain.append(joint_data)

            current_link = joint_data["parent"]

        chain.reverse()

        return cls(chain)

    def rot_mat_to_euler(self, R: np.ndarray) -> np.ndarray:
        return rot_matrix_to_euler(R)

    def normalize_angle(self, angle: float) -> float:
        return normalize_angle(angle)

    def normalize_angles_array(self, array: np.ndarray) -> np.ndarray:
        return np.array([self.normalize_angle(i) for i in array])

    def FK(self, joint_angles: np.ndarray) -> np.ndarray:
        T_total = np.identity(4)

        angle_index = 0

        for joint_data in self.kinematic_chain:
            xyz = joint_data["xyz"]
            rpy = joint_data["rpy"]

            T_translation = translation_matrix(xyz[0], xyz[1], xyz[2])
            T_rpy = rpy_to_matrix(rpy[0], rpy[1], rpy[2])

            T_motor = np.identity(4)

            if joint_data["type"] in ["revolute", "continuous"]:
                theta = joint_angles[angle_index]
                T_motor = joint_rotation_matrix(joint_data["axis"], theta)
                angle_index += 1

            T_current = T_translation @ T_rpy @ T_motor

            T_total = T_total @ T_current

        return T_total

    def compute_jacobian(
        self, joint_angles: np.ndarray, req_dim: int
    ) -> np.ndarray:  # required dimension: (X, Y, Z) -> 3 x N matrix, (X, Y, Z, Roll, Pitch, Yaw) -> 6 x N matrix, where N is the number of joints.
        delta = 1e-5

        J = np.zeros((req_dim, len(joint_angles)))

        current_T = self.FK(joint_angles)

        current_pos = current_T[:3, 3]

        current_rot_mat = current_T[:3, :3]
        current_orientation = self.rot_mat_to_euler(current_rot_mat)

        if req_dim == 6:
            current_pose = np.concatenate(
                (current_pos, current_orientation)
            )  # pose = position + orientation
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
                new_pose = np.concatenate(
                    (new_pos, new_orientation)
                )  # pose = position + orientation
                pose_diff = new_pose - current_pose

                pose_diff[3:] = self.normalize_angles_array(
                    pose_diff[3:]
                )  # to prevent Euler-angles from jumping across a boundary
            else:
                new_pose = new_pos
                pose_diff = new_pose - current_pose

            J[:, i] = pose_diff / delta

        return J

    def IK(
        self,
        target_pose: np.ndarray,
        max_iter: int = 500,
        tolerance: float = 1e-3,
        step_size: float = 0.1,
    ) -> np.ndarray:
        req_dim = len(target_pose)
        if req_dim not in [3, 6]:
            raise ValueError(f"Invalid target dimension: {target_pose}")

        joint_angles = np.zeros(
            len(self.kinematic_chain)
        )  # general (here 6) case for the number of joints

        for i in range(max_iter):
            current_T = self.FK(joint_angles)

            current_pos = current_T[:3, 3]

            current_rot_mat = current_T[:3, :3]
            current_orientation = self.rot_mat_to_euler(current_rot_mat)

            if req_dim == 6:
                current_pose = np.concatenate(
                    (current_pos, current_orientation)
                )  # pose = position + orientation
            else:
                current_pose = current_pos

            error = target_pose - current_pose

            if req_dim == 6:
                error[3:] = self.normalize_angles_array(
                    error[3:]
                )  # to prevent spinning

            if np.linalg.norm(error) < tolerance:
                return joint_angles

            J = self.compute_jacobian(joint_angles, req_dim)

            J_pseudo_inv = np.linalg.pinv(J)

            delta_theta = J_pseudo_inv @ error

            joint_angles += delta_theta * step_size

        raise ValueError(
            f"Failed to converge to the target position [{target_pose[0]}, {target_pose[1]}, {target_pose[2]}] and to the target orientation [{np.rad2deg(target_pose[3])}°, {np.rad2deg(target_pose[4])}°, {np.rad2deg(target_pose[5])}°] within {max_iter} iterations. Final error: {np.linalg.norm(error):.4f}"
        )
