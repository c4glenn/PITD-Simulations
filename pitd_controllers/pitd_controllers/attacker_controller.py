import rclpy
from rclpy.node import Node

from pitd_interfaces.msg import AttackerSpawn
from geometry_msgs.msg import Twist, Pose, Vector3, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import numpy as np

ROT_KP = 1
ANGLE_TOL = .1


class Attacker_controller(Node):
    def __init__(self) -> None:
        super().__init__("attacker_controller")
        self.attacker_pose: dict[str, Pose] = {}
        self.attacker_subs = {}
        self.attacker_pubs = {}

        self.declare_parameter("attacker_speed", .8)
        self.attacker_speed = self.get_parameter("attacker_speed").get_parameter_value().double_value

        self.new_attacker_sub = self.create_subscription(AttackerSpawn, "/pitd/spawn", self.new_attacker, 10)
        self.delete_attacker_sub = self.create_subscription(String, "/pitd/delete", self.remove_attacker, 10)

        self.logic_clock = self.create_timer(.1, self.niave_attacker)

        self.get_logger().info(f"Started")
        
    def new_attacker(self, data:AttackerSpawn):
        self.get_logger().info(f"logging attacker named {data.name}")
        self.attacker_pose[data.name] = data.pose
        self.attacker_pubs[data.name] = self.create_publisher(Pose, f"{data.name}/goal", 10)
        self.attacker_subs[data.name] = self.create_subscription(Odometry, f"{data.name}/odom", lambda x: self.track_attacker_pos(x, data.name), 10)

    def remove_attacker(self, data:String):
        self.get_logger().info(f"removing {data.data}")
        #TODO: safe removal needed
        #self.attacker_subs[data.data].destroy()
        #del self.attacker_pose[data.data]

    def track_attacker_pos(self, data:Odometry, name:str):
        self.attacker_pose[name] = data.pose.pose

    def niave_attacker(self):
        #is niave attacker strat
        for attacker, pose in self.attacker_pose.items():
            goal = Pose()
            self.attacker_pubs[attacker].publish(goal)




def main():
    rclpy.init()
    ac = Attacker_controller()
    rclpy.spin(ac)

    ac.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()