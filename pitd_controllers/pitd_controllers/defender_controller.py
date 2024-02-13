from typing import List

from numpy import sqrt
import rclpy
from rclpy.context import Context

from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from pitd_interfaces.msg import AttackerSpawn
from geometry_msgs.msg import Twist
import math


class Defender_control(Node):
    def __init__(self) -> None:
        super().__init__("/defender/controller")
        self.declare_parameter('sensing_radius', 10.0)
        self.declare_parameter('defender_strategy', 'paper_1')
        self.declare_parameter('attacker_speed', .8)
        self.declare_parameter('attacker_sensing_radius', 2.0)
        self.declare_parameter('target_radius', 10)
        self.pose = Pose()
        self.odo_subscription = self.create_subscription(Odometry, "/defender/odom", self.odom_callback, 10)
        self.last_known_location = {}
        self.subscribers = {}
        self.attacker_spawn_subscription = self.create_subscription(AttackerSpawn, "/pitd/spawn", self.new_attacker_callback,10)
        self.cmd_vel = self.create_publisher(Twist, "/defender/cmd_vel", 10)
        self.logic_timer = self.create_timer(1/30, self.logic)

        self.load_params()
    
    def vec_mag(self, pose:Pose):
        return sqrt((pose.x ** 2) + (pose.y ** 2) + (pose.z ** 2))
    
    def magnit(self, x, y):
        return sqrt((x ** 2) + (y ** 2))
    
    def load_params(self):
        self.strategy = self.get_parameter('defender_strategy').get_parameter_value().string_value
        self.sensing_radius = self.get_parameter("sensing_radius").get_parameter_value().double_value
        self.attacker_speed = self.get_parameter("attcker_speed").get_parameter_value().double_value
        self.attacker_sensing_radius = self.get_parameter('attacker_sensing_radius').get_parameter_value().double_value
        self.target_radius = self.get_parameter("target_radius").get_parameter_value().double_value
        self.alpha = 1/(1-(self.attacker_speed ** 2))
        self.beta = (self.attacker_speed ** 2) / (1 - (self.attacker_speed ** 2))
        self.gamma = self.attacker_speed / (1 - (self.attacker_speed ** 2))
        
    def odom_callback(self, data: Odometry):
        self.pose = data.pose
    
    def inside_sensing(self, pose: Pose) -> bool:
        return self.vec_mag(pose) <= self.sensing_radius
    
    def attacker_odom_callback(self, msg: Odometry, name:str):
        if self.inside_sensing(msg.pose):
            self.last_known_location[name] = msg.pose

    def new_attacker_callback(self, data: AttackerSpawn):
        if data.name in self.subscribers.keys():
            self.get_logger().warning(f"Failed to track new attacker spawn with identical name {data.name} did you publish twice?")
        self.subscribers[data.name] = self.create_subscription(Odometry, f"{data.name}/odom", lambda msg: self.attacker_odom_callback(msg, data.name), 10)

    def logic(self):
        if(self.strategy == "paper_1"):
            self.paper1()
        else:
            self.get_logger().error("undefined strategy selected")

    def within_range(self, pose1: Pose, pose2: Pose, radius):
        return sqrt(((pose1.position.x - pose2.position.x)**2) + ((pose1.position.y - pose2.position.y) ** 2))
    

    def paper1(self):
        message = Twist()
        if self.subscribers.keys[0] in self.last_known_location.keys:
            attacker = self.subscribers.keys[0]
            # Asymmetric or Engagment Phase
            appollonius_center_x = (self.alpha * self.last_known_location[attacker].position.x) - (self.beta * self.last_known_location[attacker].position.x)
            appollonius_center_y = (self.alpha * self.last_known_location[attacker].position.y) - (self.beta * self.last_known_location[attacker].position.y)
            sub_x = self.last_known_location[attacker].position.x - self.pose.position.x
            sub_y = self.last_known_location[attacker].position.y - self.pose.position.y
            appollonius_radius = self.gamma * self.magnit(sub_x, sub_y)

            if not self.within_range(self.pose, self.last_known_location[attacker], self.attacker_sensing_radius):
                # asynch info stage
                pass
            else:
                pass
                # engagment stage


        else:
            # Deployment Phase
            pose_mag = self.vec_mag(self.pose)

            if not (pose_mag <= self.target_radius + self.sensing_radius - self.attacker_sensing_radius):
                #not following the requirment so head in till you do
                
                if self.pose.position.x <= self.pose.position.y:
                    message.linear.y = math.min(self.pose.position.y, 1)
                else:
                    message.linear.x = math.min(self.pose.position.x, 1)
            else:
                message.linear.x = 0
                message.linear.y = 0
                message.linear.z = 0
                message.angular.z = 0

        self.cmd_vel.publish(message)

def main():
    rclpy.init()
    defen = Defender_control()

    rclpy.spin(defen)

    defen.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()