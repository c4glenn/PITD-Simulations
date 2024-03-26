import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Twist
from pitd_interfaces.msg import AttackerSpawn, Goal
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult

import numpy as np

ROT_KP = 1
KP = 2
DIST_TOL = .1
ROT_TOL = .01


def normalize_angle(angle):
    return np.arctan2(np.cos(angle), np.sin(angle))


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class Pathing(Node):
    def __init__(self) -> None:
        super().__init__("Pather")

        self.goal_subs = {
            "defender":self.create_subscription(Goal, "/defender/goal", lambda x: self.handle_goal(x, "defender"), 10)
        }

        self.goals: dict[str, Pose] = {}

        self.odo_subs = {
            "defender":self.create_subscription(Odometry, "/defender/odom", lambda x: self.handle_odom(x, "defender"), 10)
        }

        self.cmd_pubs = {
            "defender":self.create_publisher(Twist, "/defender/cmd_vel", 10)
        }

        self.newAttacker = self.create_subscription(AttackerSpawn, "/pitd/spawn", self.new_attacker, 10)
        self.removeAttacker = self.create_subscription(String, "/pitd/delete", self.delete_attacker, 10)
        self.declare_parameter('attacker_speed', .8)
        self.attacker_speed = self.get_parameter("attacker_speed").get_parameter_value().double_value

        # self.add_on_set_parameters_callback(self.params_callback)

    def params_callback(self, params):
        self.attacker_speed = self.get_parameter("attacker_speed").get_parameter_value().double_value

        return SetParametersResult(successful=True)

    
    def new_attacker(self, data: AttackerSpawn):
        self.get_logger().info(f"registering an attacker {data.name}")
        self.goal_subs[data.name] = self.create_subscription(Goal, f"/{data.name}/goal", lambda x: self.handle_goal(x, data.name), 10)
        self.odo_subs[data.name] = self.create_subscription(Odometry, f"/{data.name}/odom", lambda x: self.handle_odom(x, data.name), 10)
        self.cmd_pubs[data.name] = self.create_publisher(Twist, f"/{data.name}/cmd_vel", 10)

    def delete_attacker(self, data:String):
        #TODO: safe deletion needed
        #del self.goal_subs[data.data]
        #del self.cmd_pubs[data.data]
        #del self.odo_subs[data.data]
        pass
        
    def handle_goal(self, data:Goal, name:str):
        self.goals[name] = data

    def handle_odom(self, data:Odometry, name:str):
        self.attacker_speed = self.get_parameter("attacker_speed").get_parameter_value().double_value
        goal = self.goals.get(name, None)
        if not goal:
            return
        min_speed = goal.min_speed
        goal = goal.pose
        
        goal_heading = euler_from_quaternion(goal.orientation)[2]

        pose = data.pose.pose
        current_heading = euler_from_quaternion(pose.orientation)[2]


        max_speed = self.attacker_speed
        if name == "defender":
            max_speed = 1

        command = Twist()

        x_error = goal.position.x - pose.position.x
        y_error = goal.position.y - pose.position.y

        distance_error = np.sqrt((x_error**2) + (y_error ** 2))

        if abs(distance_error) <= DIST_TOL:
            #just rotate to the goal
            theta_error = normalize_angle(goal_heading - current_heading)
            if abs(theta_error) <= ROT_TOL:
                rotation_correction = 0
                distance_correction = 0
            else:
                rotation_correction = theta_error * ROT_KP * -1
                distance_correction = 0
        
        else:
            goal_theta = normalize_angle(np.arctan2(-y_error, x_error))

            theta_error = normalize_angle(goal_theta - current_heading)

            m = (-1 * (min(abs(theta_error), np.pi/4)/(np.pi/4))) + 1

            distance_correction = distance_error * KP * m
            rotation_correction = theta_error * ROT_KP * -1

        scaled_dist = np.sign(distance_correction) * min(max(abs(distance_correction), min_speed), max_speed)
        scaled_rot = np.sign(rotation_correction) *  min(max(abs(rotation_correction), min_speed), max_speed)

        command.linear.x = float(scaled_dist)
        command.angular.z = float(scaled_rot)

        self.cmd_pubs[name].publish(command)

def main():
    rclpy.init()
    pathing = Pathing()
    rclpy.spin(pathing)

    pathing.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()