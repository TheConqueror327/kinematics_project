import sys
import numpy as np

sys.path.append("../kinematics_project")

from kinematics.robot_nDoF import Robot

SO101 = Robot.from_urdf("LeRobot_SO101/LeRobot_SO101.urdf")

"""
for i in SO101.kinematic_chain:
    print(i)"""


test_angles = np.array([0.5, -0.2, 0.4, 1.1, -0.5, 0.1])

T = SO101.FK(test_angles)

position = T[:3, 3]

orientation = SO101.rot_mat_to_euler(T[:3, :3])

pose = np.concatenate((position, orientation))

angles_IK = SO101.IK(pose)


# Verification

verificated_T = SO101.FK(angles_IK)

verificated_position = verificated_T[:3, 3]


print(f"Test angles: {test_angles}")
print(f"Angles calculated with IK: {angles_IK}")

print(f"Test position: {position}")
print(f"Verificated position with FK: {verificated_position}")
