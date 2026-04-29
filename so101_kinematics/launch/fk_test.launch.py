import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = "so101_kinematics"
    pkg_share = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(pkg_share, "urdf", "LeRobot_SO101.urdf")

    with open(urdf_file, "r") as infp:
        robot_desc = infp.read()

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
    )

    fk_node = Node(package="so101_kinematics", executable="fk_test", output="screen")

    # RViz2
    rviz_node = Node(package="rviz2", executable="rviz2", output="screen")

    return LaunchDescription([rsp_node, fk_node, rviz_node])
