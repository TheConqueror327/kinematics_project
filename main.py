import numpy as np
from kinematics.robot_6DoF import Robot
from kinematics.utils import rot_matrix_to_euler

# 3 DoF

dh_3DoF = np.array([
    [0, 10.0, 0.0, np.deg2rad(90)],  # Link 1
    [0, 0.0, 10.0, 0.0],             # Link 2
    [0, 0.0, 10.0, 0.0]              # Link 3
])

# 6 DoF - simplified UR5 Robotic arm

dh_6DoF = np.array([
    [0.0, 8.9,  0.0,  np.pi/2],  # Base
    [0.0, 0.0, -42.5, 0.0],      # Shoulder
    [0.0, 0.0, -39.2, 0.0],      # Elbow
    [0.0, 10.9, 0.0,  np.pi/2],  # Wrist 1
    [0.0, 9.4,  0.0, -np.pi/2],  # Wrist 2
    [0.0, 8.2,  0.0,  0.0]       # Wrist 3
#   theta, d,   a,    alpha
])

"""

# 3 DoF FK and IK test

my_robot = Robot(dh_3DoF)

target_pos = np.array([20.0, 0.0, 10.0])

try:
    calculated_angles_deg = my_robot.IK_analytic(target_pos)
    calculated_angles_deg = [np.rad2deg(szog) for szog in calculated_angles_deg]

    print("--- INVERSE KINEMATICS TEST - ANALYTIC ---")
    print(f"Target position: {target_pos}")
    print(f"Calculated angles using the analytic method: [{calculated_angles_deg[0]:.2f}°, {calculated_angles_deg[1]:.2f}°, {calculated_angles_deg[2]:.2f}°]")


    calculated_angles_rad = my_robot.IK_iterative(target_pos)
    calculated_angles_deg = [np.rad2deg(angle) for angle in calculated_angles_rad]

    print("\n--- INVERSE KINEMATICS TEST - ITERATIVE ---")
    print(f'Target position: {target_pos}')
    print(f'Calulated angles using the iterative method: [{calculated_angles_deg[0]:.2f}°, {calculated_angles_deg[1]:.2f}°, {calculated_angles_deg[2]:.2f}°]')
    

    T_end = my_robot.FK(calculated_angles_rad)
    calculated_pos = [T_end[0, 3], T_end[1, 3], T_end[2, 3]]

    print("\n--- CHECK WITH FORWARD KINEMATICS ---")
    print(f"Actual position with these angles: [{calculated_pos[0]:.2f}, {calculated_pos[1]:.2f}, {calculated_pos[2]:.2f}]")

except ValueError as e:
    print(f"An error occured: {e}")

"""


"""
T_initial = my_robot.FK(np.array([0, 0, 0, 0, 0, 0]))

current_pos = T_initial[:3, 3]

current_rot_matrix = T_initial[:3, :3]

current_orientation = rot_matrix_to_euler(current_rot_matrix)

print(f'Initial position: [{current_pos[0]}, {current_pos[1]}, {current_pos[2]}], current_orientation: [{np.rad2deg(current_orientation[0])}°, {np.rad2deg(current_orientation[1])}°, {np.rad2deg(current_orientation[2])}°]')
"""
my_robot = Robot(dh_6DoF)

angles = np.array([0.1, 0.2, 0.3, 0.1, 0.2, 0.3])

T = my_robot.FK(angles)

current_pos = T[:3, 3]

current_orientation = my_robot.rot_mat_to_euler(T[:3, :3])

current_pose = np.concatenate((current_pos, current_orientation))

print(f'Current pose: {current_pose}')

# checking the IK solver:

angles_IK = my_robot.IK(current_pose)

print(f'Angles calculated by IK: {angles_IK}')

# checking back again with FK:

ver_T = my_robot.FK(angles_IK)

ver_pos = ver_T[:3, 3]

ver_orientation = my_robot.rot_mat_to_euler(ver_T[:3, :3])

ver_pose = np.concatenate((ver_pos, ver_orientation))

print(f'Final check wit FK: {ver_pose} == {current_pose}')