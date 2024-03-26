import rclpy
from rclpy.node import Node

from pitd_interfaces.msg import AttackerSpawn, Goal
from geometry_msgs.msg import Twist, Pose, Vector3, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult

import numpy as np


class Attacker_controller(Node):
    def __init__(self) -> None:
        super().__init__("attacker_controller")
        self.attacker_pose: dict[str, Pose] = {}
        self.attacker_subs = {}
        self.attacker_pubs = {}

        self.declare_parameter("attacker_speed", .8)
        self.attacker_speed = self.get_parameter("attacker_speed").get_parameter_value().double_value

        self.declare_parameter("attacker_sensing_radius", 1.0)
        self.attacker_sensing_radius = self.get_parameter("attacker_sensing_radius").get_parameter_value().double_value

        self.declare_parameter("defender_sensing_radius", 10.0)
        self.defender_sensing_radius = self.get_parameter("defender_sensing_radius").get_parameter_value().double_value

        self.defender_pose = Pose()

        self.defender_sub = self.create_subscription(Odometry, "defender/odom", self.defender_tracker, 10)

        self.new_attacker_sub = self.create_subscription(AttackerSpawn, "/pitd/spawn", self.new_attacker, 10)
        self.delete_attacker_sub = self.create_subscription(String, "/pitd/delete", self.remove_attacker, 10)

        self.logic_clock = self.create_timer(.1, self.attacker_logic)

        # self.add_on_set_parameters_callback(self.params_callback)

        self.get_logger().info(f"Started")

    def params_callback(self, params):
        self.attacker_speed = self.get_parameter("attacker_speed").get_parameter_value().double_value
        self.attacker_sensing_radius = self.get_parameter("attacker_sensing_radius").get_parameter_value().double_value
        self.defender_sensing_radius = self.get_parameter("defender_sensing_radius").get_parameter_value().double_value

        return SetParametersResult(successful=True)

    def defender_tracker(self, odom:Odometry):
        self.defender_pose = odom.pose.pose
        
    def new_attacker(self, data:AttackerSpawn):
        self.get_logger().info(f"logging attacker named {data.name}")
        self.attacker_pose[data.name] = data.pose
        self.attacker_pubs[data.name] = self.create_publisher(Goal, f"{data.name}/goal", 10)
        self.attacker_subs[data.name] = self.create_subscription(Odometry, f"{data.name}/odom", lambda x: self.track_attacker_pos(x, data.name), 10)

    def remove_attacker(self, data:String):
        self.get_logger().info(f"removing {data.data}")
        #TODO: safe removal needed
        #self.attacker_subs[data.data].destroy()
        #del self.attacker_pose[data.data]

    def track_attacker_pos(self, data:Odometry, name:str):
        self.attacker_pose[name] = data.pose.pose

    def within_sensing(self, attacker):
        self.attacker_sensing_radius = self.get_parameter("attacker_sensing_radius").get_parameter_value().double_value
        pose = self.attacker_pose[attacker]
        x_diff = pose.position.x - self.defender_pose.position.x
        y_diff = pose.position.y - self.defender_pose.position.y
        distance = np.sqrt((x_diff ** 2) + (y_diff ** 2))
        #self.get_logger().info(f"{attacker} {x_diff} {y_diff} {distance} {self.attacker_sensing_radius} {distance < self.attacker_sensing_radius}")

        return distance < self.attacker_sensing_radius
            

    def attacker_logic(self):
        self.defender_sensing_radius = self.get_parameter("defender_sensing_radius").get_parameter_value().double_value
        self.attacker_speed = self.get_parameter("attacker_speed").get_parameter_value().double_value

        for attacker, pose in self.attacker_pose.items():
            goal = Pose()
            if self.within_sensing(attacker):
                theta = np.arctan2(pose.position.y, pose.position.x)
                r = self.defender_sensing_radius + ( self.attacker_sensing_radius / 4)
                goal = Pose(position=Point(x=r*np.cos(theta), y=r*np.sin(theta)))
            
            self.attacker_pubs[attacker].publish(Goal(pose=goal, min_speed=self.attacker_speed))




def main():
    rclpy.init()
    ac = Attacker_controller()
    rclpy.spin(ac)

    ac.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()