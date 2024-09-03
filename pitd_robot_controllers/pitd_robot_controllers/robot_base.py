import rclpy
from rclpy.node import Node

from pitd_interfaces.srv import GetParams

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class RobotBase(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.attacker_pose_sub = self.create_subscription(Odometry, "/attacker/odom", lambda x: self.pose_callback("attacker"))
        self.defender_pose_sub = self.create_subscription(Odometry, "/defender/odom", lambda x: self.pose_callback("defender"))
        self.active_sub = self.create_subscription(Bool, "/active", self.active_callback)
        
    def get_params(self) -> None:
        cli = self.create_client()
    
    def pose_callback(self, robot: str, msg: Odometry) -> None:
        pass    

    def active_callback(self, msg: Bool) -> None:
        pass
    
