import numpy as np
from kinematics.robot_3DoF import Robot

# 3 DoF

dh_parameters = np.array([
    [0, 10.0, 0.0, np.deg2rad(90)],  # Link 1
    [0, 0.0, 10.0, 0.0],             # Link 2
    [0, 0.0, 10.0, 0.0]              # Link 3
])


my_robot = Robot(dh_parameters)

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