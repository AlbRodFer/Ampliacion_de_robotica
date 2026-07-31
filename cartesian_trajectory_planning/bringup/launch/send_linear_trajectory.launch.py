# Author: Juan M. Gandarias
# email: jmgandarias@uma.es
# web: https://jmgandarias.com
#
# Inspired from the ros2_control Development Team demos

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("cartesian_trajectory_planning"),
                    "urdf",
                    "r6bot.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    send_linear_trajectory_node = Node(
        package="cartesian_trajectory_planning",
        executable="send_linear_trajectory",
        name="send_linear_trajectory_node",
        parameters=[robot_description],
    )

    nodes_to_start = [send_linear_trajectory_node]
    return LaunchDescription(nodes_to_start)
