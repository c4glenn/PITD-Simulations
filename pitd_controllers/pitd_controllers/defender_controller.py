from typing import List

from numpy import sqrt
import rclpy
from rclpy.context import Context

from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from pitd_interfaces.msg import AttackerSpawn

class Defender_control(Node):
    def __init__(self) -> None:
        super().__init__("/defender/controller")
        self.declare_parameter('sensing_radius', 10)
        self.pose = Pose()
        self.odo_subscription = self.create_subscription(Odometry, "/defender/odom", self.odom_callback, 10)
        self.last_known_location = {}
        self.subscribers = {}
        self.attacker_spawn_subscription = self.create_subscription(AttackerSpawn, "/pitd/spawn", self.new_attacker_callback,10)
        
    def odom_callback(self, data: Odometry):
        self.pose = data.pose
    
    def inside_sensing(self, pose: Pose) -> bool:
        return sqrt((pose.position.x ** 2) + (pose.position.y ** 2) + (pose.position.z ** 2)) <= self.get_parameter("sensing_radius")
    
    def attacker_odom_callback(self, msg: Odometry, name:str):
        if self.inside_sensing(msg.pose):
            self.last_known_location[name] = msg.pose

    def new_attacker_callback(self, data: AttackerSpawn):
        if data.name in self.subscribers.keys():
            self.get_logger().warning(f"Failed to track new attacker spawn with identical name {data.name} did you publish twice?")
        self.subscribers[data.name] = self.create_subscription(Odometry, f"{data.name}/odom", lambda msg: self.attacker_odom_callback(msg, data.name), 10)

def main():
    rclpy.init()
    defen = Defender_control()

    rclpy.spin(defen)

    defen.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()