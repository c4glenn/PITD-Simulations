import rclpy
from rclpy.node import Node

class IHateThis(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_producer")

        