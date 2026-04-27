import unittest
import numpy as np
import sys
from kinematics.robot_nDoF import Robot

sys.path.append("../kinematics_project")


class TestnDoF(unittest.TestCase):
    def test_nDoF(self):

        my_robot = Robot.from_urdf("LeRobot_SO101/LeRobot_SO101.urdf")

        angles = np.array([0.1, 0.2, 0.3, 0.1, 0.2, 0.3])

        T_3 = my_robot.FK(angles)

        current_pos = T_3[:3, 3]  # (X, Y, Z)

        angles_IK = my_robot.IK(current_pos)

        T_3_ver = my_robot.FK(angles_IK)

        ver_pos = T_3_ver[:3, 3]

        print(f"Original angles: {angles}, FK: {current_pos}")

        print(f"Angles calculated with IK: {angles_IK}, FK: {ver_pos}")
        self.assertAlmostEqual(current_pos[0], ver_pos[0], delta=0.001)
