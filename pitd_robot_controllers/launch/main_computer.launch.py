from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pitd_robot_controllers",
            namespace="attacker",
            executable="controller",
            name="attacker_controller"
        ),
        Node(
            package="pitd_robot_controllers",
            namespace="defender",
            executable="controller",
            name="defender_controller"
        ),
        # Node(
        #     package="pitd_robot_controllers",
        #     executable="position",
        #     name="position_controller"
        # )
    ])