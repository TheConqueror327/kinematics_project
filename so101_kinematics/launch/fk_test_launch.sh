cd ~/ros2_ws/
colcon build --symlink-install
source install/setup.bash
ros2 launch so101_kinematics fk_test.launch.py
