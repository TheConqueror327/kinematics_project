import sys
import numpy as np

sys.path.append('../kinematics_project')

from kinematics.robot_nDoF import Robot

SO101 = Robot.from_urdf('LeRobot_SO101/LeRobot_SO101.urdf')


for i in SO101.kinematic_chain:
    print(i)


T_end = SO101.FK(np.array([0, 0, 0, 0, 0, 0]))

print(T_end)