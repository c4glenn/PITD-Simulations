from typing import List

from numpy import sqrt
import rclpy
from rclpy.context import Context

from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Pose, Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from pitd_interfaces.msg import AttackerSpawn, Goal
from rcl_interfaces.msg import SetParametersResult
import numpy as np


def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [w, x, y, z]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


class Defender_control(Node):
    def __init__(self) -> None:
        super().__init__("defender_controller")
        print("am i doing anything")

        self.declare_parameter('defender_sensing_radius', 10.0)
        self.declare_parameter('defender_strategy', 'paper_1')
        self.declare_parameter('attacker_speed', .8)
        self.declare_parameter('attacker_sensing_radius', 1.0)
        self.declare_parameter('target_radius', 5.0)
        self.pose = Pose()
        self.odo_subscription = self.create_subscription(Odometry, "/defender/odom", self.odom_callback, 10)
        self.last_known_location = {}
        self.subscribers = {}
        self.attacker_spawn_subscription = self.create_subscription(AttackerSpawn, "/pitd/spawn", self.new_attacker_callback,10)
        self.goal_pub = self.create_publisher(Goal, "/defender/goal", 10)
        self.logic_timer = self.create_timer(1/30, self.logic)
        self.add_on_set_parameters_callback(self.param_callback)
        self.load_params()
        self.current_target = ""

    def param_callback(self, params):
        self.load_params()
        return SetParametersResult(successful=True)
    
    def vec_mag(self, pose:Pose):
        return sqrt((pose.position.x ** 2) + (pose.position.y ** 2) + (pose.position.z ** 2))
    
    def magnit(self, x, y):
        return sqrt((x ** 2) + (y ** 2))
    
    def load_params(self):
        self.strategy = self.get_parameter('defender_strategy').get_parameter_value().string_value
        self.sensing_radius = self.get_parameter("defender_sensing_radius").get_parameter_value().double_value
        self.attacker_speed = self.get_parameter("attacker_speed").get_parameter_value().double_value
        self.attacker_sensing_radius = self.get_parameter('attacker_sensing_radius').get_parameter_value().double_value
        self.target_radius = self.get_parameter("target_radius").get_parameter_value().double_value
        self.alpha = 1/(1-(self.attacker_speed ** 2))
        self.beta = (self.attacker_speed ** 2) / (1 - (self.attacker_speed ** 2))
        self.gamma = self.attacker_speed / (1 - (self.attacker_speed ** 2))
        
    def odom_callback(self, data: Odometry):
        self.pose = data.pose.pose
    
    def inside_sensing(self, pose: Pose) -> bool:
        self.load_params()
        return self.vec_mag(pose) <= self.sensing_radius
    
    def attacker_odom_callback(self, msg: Odometry, name:str):
        if self.inside_sensing(msg.pose.pose):
            self.last_known_location[name] = msg.pose.pose

    def new_attacker_callback(self, data: AttackerSpawn):
        if data.name in self.subscribers.keys():
            self.get_logger().warning(f"Failed to track new attacker spawn with identical name {data.name} did you publish twice?")
        self.subscribers[data.name] = self.create_subscription(Odometry, f"{data.name}/odom", lambda msg: self.attacker_odom_callback(msg, data.name), 10)
        self.current_target = data.name
    def logic(self):
        if(self.strategy == "paper_1"):
            self.paper1()
        else:
            self.get_logger().error("undefined strategy selected")

    def within_range(self, pose1: Pose, pose2: Pose, radius):
        return sqrt(((pose1.position.x - pose2.position.x)**2) + ((pose1.position.y - pose2.position.y) ** 2)) <= radius
    

    def paper1(self):
        self.load_params()
        message = Pose()

        if len(self.subscribers.keys()) <= 0:
            self.goal_pub.publish(Goal(pose=Pose(position=Point(x=0.0, y=0.0))))
            return

        if self.current_target in self.last_known_location.keys():
            attacker = self.current_target
            # Asymmetric or Engagment Phase
            appollonius_center_x = (self.alpha * self.last_known_location[attacker].position.x) - (self.beta * self.last_known_location[attacker].position.x)
            appollonius_center_y = (self.alpha * self.last_known_location[attacker].position.y) - (self.beta * self.last_known_location[attacker].position.y)
            sub_x = self.last_known_location[attacker].position.x - self.pose.position.x
            sub_y = self.last_known_location[attacker].position.y - self.pose.position.y
            appollonius_radius = self.gamma * self.magnit(sub_x, sub_y)
            attacker_pose: Pose = self.last_known_location[attacker]

            if not self.within_range(self.pose, self.last_known_location[attacker], self.attacker_sensing_radius):
                theta = np.arctan2(attacker_pose.position.y, attacker_pose.position.x)
                bot_angle = np.arctan2(attacker_pose.position.y - self.pose.position.y, attacker_pose.position.x - self.pose.position.x)
                r = self.target_radius - (1 - self.attacker_speed)
                x = r * np.cos(theta)
                y = r * np.sin(theta)
                quat = quaternion_from_euler(0, 0, normalize_angle(bot_angle + (np.pi/2)))

                self.goal_pub.publish(Goal(pose=Pose(position=Point(x=x, y=y), orientation=Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3]))))
            else:
                self.goal_pub.publish(Goal(pose=attacker_pose, min_speed=1.0))
                # engagment stage


        else:
            self.goal_pub.publish(Goal(pose=Pose(position=Point(x=0.0, y=0.0))))

def main():
    rclpy.init()
    defen = Defender_control()

    rclpy.spin(defen)

    defen.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()