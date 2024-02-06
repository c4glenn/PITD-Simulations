#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

class Spawner(Node):
    def __init__(self):
        super().__init__("gazeboSpawner")

    def spawn_attacker(self):
        

def main():
    rclpy.init()
    spawner = Spawner()
    rclpy.spin(spawner)

    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()