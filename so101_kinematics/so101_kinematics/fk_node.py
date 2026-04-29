import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import math
import time
import os
from ament_index_python.packages import get_package_share_directory


from .robot_nDoF import Robot


class FKTestNode(Node):
    def __init__(self):
        super().__init__("fk_test_node")

        pkg_share = get_package_share_directory("so101_kinematics")
        urdf_path = os.path.join(pkg_share, "urdf", "LeRobot_SO101.urdf")
        self.my_robot = Robot.from_urdf(urdf_path, base_link="base", end_link="gripperframe")

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)

        self.marker_pub = self.create_publisher(Marker, "fk_marker", 10)

        self.timer = self.create_timer(0.033, self.timer_callback)
        self.start_time = time.time()

        self.joint_names = ["1", "2", "3", "4", "5", "6"]

    def timer_callback(self):
        t = time.time() - self.start_time

        q = [
            math.sin(t) * 1.0,
            math.sin(t * 0.8) * 0.5,
            math.sin(t * 1.2) * 0.5,
            math.sin(t * 0.9) * 0.5,
            math.sin(t * 1.1) * 0.5,
            math.sin(t * 1.3) * 0.5,
        ]

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = q
        self.joint_pub.publish(js)

        T = self.my_robot.FK(q)
        pos_x = T[0, 3]
        pos_y = T[1, 3]
        pos_z = T[2, 3]

        marker = Marker()
        marker.header.frame_id = "base"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "fk_test"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pos_x
        marker.pose.position.y = pos_y
        marker.pose.position.z = pos_z

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = FKTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
