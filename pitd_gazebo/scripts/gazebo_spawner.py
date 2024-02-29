#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from pitd_interfaces.msg import AttackerSpawn


class Spawner(Node):
    def __init__(self):
        super().__init__("gazeboSpawner")
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service (spawn_entity) Not Avaliable, waiting again...')
        self.spawn_sub = self.create_subscription(AttackerSpawn, "/pitd/spawn", self.spawn_attacker_callback, 10)
    
    def spawn_attacker_callback(self, data: AttackerSpawn):
        self.get_logger().info(f"spawning in attacker named {data.name}")
        self.spawn_attacker(data.name, data.pose)

    def spawn_attacker(self, name: str, pose: Pose):
        request = SpawnEntity.Request()
        request.name = name
        TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
        model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
        urdf_path = os.path.join(
            get_package_share_directory('pitd_gazebo'),
            'models',
            model_folder,
            'model.sdf'
        )
        with open(urdf_path, "r") as f:
            request.xml = f.read()
        
        request.robot_namespace = name
        request.initial_pose = pose

        self.future = self.spawn_client.call_async(request)
        
def main():
    rclpy.init()
    spawner = Spawner()
    rclpy.spin(spawner)

    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()