import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from .robot_nDoF import Robot
import tf2_ros
import math
import time
import os
import sys
from ament_index_python.packages import get_package_share_directory


class FKDebugNode(Node):
    def __init__(self):
        super().__init__("fk_debug_node")

        pkg_share = get_package_share_directory("so101_kinematics")
        urdf_path = os.path.join(pkg_share, "urdf", "LeRobot_SO101.urdf")
        self.my_robot = Robot.from_urdf_debug(urdf_path)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        """
        self.subscription = self.create_subscription(
            JointState, "joint_states", self.listener_callback, 10
        )"""

        self.end_effector_link = "moving_jaw_so101_v1"

        self.base_link = "baseframe"

    def listener_callback(self, msg):
        q = msg.position
        T_custom = self.my_robot.FK(q)
        pos_custom = T_custom[:3, 3]

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.end_effector_link, self.base_link, now
            )

            pos_ros = [
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
            ]

            self.get_logger().info(
                f"\n OWN FK POSITION:  X:{pos_custom[0]:.4f}, Y:{pos_custom[1]:.4f}, Z:{pos_custom[2]:.4f}"
                + f"\nROS 2 TF POSITION:  X:{pos_ros[0]:.4f}, Y:{pos_ros[1]:.4f}, Z:{pos_ros[2]:.4f}"
                + f"\nDIFFERENCE (Error):    {abs(pos_custom[0] - pos_ros[0]):.4f}"
                + "\n"
                + "=" * 40
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            self.get_logger().warn("Waiting for the ROS2 TF transformation...")


def main(args=None):
    rclpy.init(args=args)
    node = FKDebugNode()
    rclpy.spin(node)
    node.destroy_node(node)
    rclpy.shutdown()
