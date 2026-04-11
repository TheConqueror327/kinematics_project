import unittest
import numpy as np
from kinematics.robot_nDoF import Robot

class TestnDoF(unittest.TestCase):
    def test_nDoF(self):
        dh_6DoF = np.array([
            [0.0, 8.9,  0.0,  np.pi/2],  # Base
            [0.0, 0.0, -42.5, 0.0],      # Shoulder
            [0.0, 0.0, -39.2, 0.0],      # Elbow
            [0.0, 10.9, 0.0,  np.pi/2],  # Wrist 1
            [0.0, 9.4,  0.0, -np.pi/2],  # Wrist 2
            [0.0, 8.2,  0.0,  0.0]       # Wrist 3
        #   theta, d,   a,    alpha
        ])

        my_robot = Robot(dh_6DoF)


        angles = np.array([0.1, 0.2, 0.3, 0.1, 0.2, 0.3])

        T_3 = my_robot.FK(angles)

        current_pos = T_3[:3, 3] # (X, Y, Z)


        angles_IK = my_robot.IK(current_pos)

        T_3_ver = my_robot.FK(angles_IK)

        ver_pos = T_3_ver[:3, 3]


        print(f'Original angles: {angles}, FK: {current_pos}')

        print(f'Angles calculated with IK: {angles_IK}, FK: {ver_pos}')
        self.assertAlmostEqual(current_pos[0], ver_pos[0], delta=0.001)

        