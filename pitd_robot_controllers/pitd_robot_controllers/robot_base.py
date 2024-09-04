import rclpy
from rclpy.node import Node

import numpy as np

from pitd_interfaces.srv import GetParams

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool



class RobotBase(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.attacker_pose_sub = self.create_subscription(Odometry, "/attacker/odom", lambda x: self.pose_callback("attacker"))
        self.defender_pose_sub = self.create_subscription(Odometry, "/defender/odom", lambda x: self.pose_callback("defender"))
        self.active_sub = self.create_subscription(Bool, "/active", self.active_callback)
        self.get_params()
        self.active = False
        
    def get_params(self) -> None:
        cli = self.create_client(GetParams, "/settings") = res.create_client(GetParams, "/settings")
        future = cli.call_service(GetParams.Request())
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        
        self.attacker_speed = res.attacker_speed
        self.defender_speed = res.defender_speed
        self.attacker_sensing_radius = res.attacker_sensing_radius
        self.defender_sensing_radius = res.defender_sensing_radius
        self.target_radius = res.target_radius
        
    
    def set_goal(self, goal: Pose):
        self.goal = goal

    def call_each_step(self):
        if not self.active:
            return
    
    def distance(p1: Pose, p2: Pose) -> float:
        return np.sqrt((p2.position.x - p1.position.x)**2 + (p2.position.y - p1.position.y)**2)
    
    
    def pose_callback(self, robot: str, msg: Odometry) -> None:
        pass                

    def active_callback(self, msg: Bool) -> None:
        self.active = msg.data
    
