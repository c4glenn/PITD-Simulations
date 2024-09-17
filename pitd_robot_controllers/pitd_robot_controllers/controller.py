from typing import List
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Pose, Vector3, Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Bool

import numpy as np

DIST_TOL = 0.05
ROT_TOL = 0.034
TURTLEBOT_MAX_SPEED = 0.07
TURTLEBOT_MAX_ROT = 1

ATTACKER_MAX_SPEED = 0.8 * TURTLEBOT_MAX_SPEED


K_rot = 1
K_trans = 0.5

class Controller(Node):
    def normalize_angle(self, angle: float) -> float:
        return np.arctan2(np.sin(angle), np.cos(angle))

    def __init__(self) -> None:
        super().__init__("_goal_follower")
        self.goal_sub = self.create_subscription(Pose, "goal", self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.goal_reach_pub = self.create_publisher(Bool, "goal_finished", 10)


        self.goal = Pose()

        self.is_attacker = "attacker" in self.get_namespace()

    def odom_callback(self, msg: Odometry):
        self.pose = msg.pose.pose
        robot_angle = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w])[2]

        distance_error = np.sqrt((self.goal.position.x - self.pose.position.x)**2 + (self.goal.position.y - self.pose.position.y)**2)
        self.get_logger().info(f"dist: {distance_error}")
        if abs(distance_error) <= DIST_TOL:
            #inside distance just fix headin
            goal = euler_from_quaternion([self.goal.orientation.x, self.goal.orientation.y, self.goal.orientation.z, self.goal.orientation.w])[2]
            rotation_error = self.normalize_angle(goal- robot_angle)
            self.get_logger().info(f"rot: {rotation_error}")
            if abs(rotation_error) <= ROT_TOL:
                self.cmd_pub.publish(Twist())
                self.goal_reach_pub.publish(Bool(data=True))
                return
            
            self.cmd_pub.publish(Twist(angular=Vector3(z=K_rot*rotation_error)))
            self.goal_reach_pub.publish(Bool(data=False))
            return
        
        self.goal_reach_pub.publish(Bool(data=False))
        goal_heading = np.arctan2(self.goal.position.y - self.pose.position.y, self.goal.position.x - self.pose.position.x)
        rotation_error = self.normalize_angle(goal_heading-robot_angle)
        self.get_logger().info(f"rot: {rotation_error, goal_heading,robot_angle}")
        # if abs(rotation_error) <= ROT_TOL:
        #     rot_val = 0.0
        # else:
        rot_val = K_rot*rotation_error #np.sign(rotation_error) * TURTLEBOT_MAX_ROT
        self.get_logger().info(f"{rot_val}")
        x_val = ATTACKER_MAX_SPEED if self.is_attacker else TURTLEBOT_MAX_SPEED
        self.get_logger().info(f"x: {x_val}")
        self.cmd_pub.publish(Twist(linear=Vector3(x=x_val), angular=Vector3(z=float(rot_val))))

    def goal_callback(self, msg: Pose):
        self.goal = msg

def main():
    rclpy.init()
    hcp = Controller()
    rclpy.spin(hcp)

    hcp.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    pass