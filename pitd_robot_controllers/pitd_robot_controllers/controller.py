from __future__ import annotations


from typing import Optional

import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Vector3, Twist, Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Bool
from curve_planners_msgs.action import PlanCurve


DIST_TOL = 0.05
ROT_TOL = 0.034
TURTLEBOT_MAX_SPEED = 0.1
TURTLEBOT_MAX_ROT = 1
TURNING_RADIUS = 0.12
ATTACKER_MAX_SPEED = 0.8 * TURTLEBOT_MAX_SPEED


K_rot = 1
K_trans = 0.5

class Controller(Node):
    def normalize_angle(self, angle: float) -> float:
        return np.arctan2(np.sin(angle), np.cos(angle))


    def __init__(self) -> None:
        super().__init__("goal_follower")

        self.goal_sub = self.create_subscription(Pose, "goal", self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.goal_reach_pub = self.create_publisher(Bool, "goal_finished", 10)

        self.is_attacker = "attacker" in self.get_namespace()


        self.path_planner_client = ActionClient(self, PlanCurve, '/plan_curve')

        self.robot_pose = None

        self.path: Optional[Path] = None
        self.path_tracker = 0
        self.last_goal = None

        self.not_first = False


    def goal_callback(self, msg: Pose):
        if self.last_goal == msg and self.not_first:
            return

        self.last_goal = msg

        if not self.robot_pose:
            return

        goal = PoseStamped()
        goal.pose = msg

        self.get_logger().info(f"recieved goal {msg}")

        self.plan_curve(self.robot_pose, goal)
        self.not_first = True

    def odom_callback(self, msg: Odometry):
        self.robot_pose = PoseStamped()
        self.robot_pose.pose = msg.pose.pose
        if self.is_attacker:
            self.robot_pose.pose.position.x += 1.1984
            self.robot_pose.pose.position.y += 2.3546
        self.robot_pose.header.stamp = msg.header.stamp

        if not self.path:
            return


        self.pose = msg.pose.pose
        # if self.is_attacker:
        #     self.pose.position.x += 1.1984
        #     self.pose.position.y += 2.3546
        self.goal = self.path.poses[self.path_tracker].pose


        self.get_logger().info(f"{self.path_tracker} / {len(self.path.poses) - 1} {self.goal.position}")
        self.get_logger().info(f"{self.pose.position}")
        robot_angle = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w])[2]

        distance_error = np.sqrt((self.goal.position.x - self.pose.position.x)**2 + (self.goal.position.y - self.pose.position.y)**2)
        self.get_logger().info(f"dist: {distance_error}")
        if abs(distance_error) <= DIST_TOL:
            if len(self.path.poses) -1 <= self.path_tracker:
                self.get_logger().info(f"LAST PATH STEP")
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
            self.path_tracker += 1
            
        
        self.goal_reach_pub.publish(Bool(data=False))
        goal_heading = np.arctan2(self.goal.position.y - self.pose.position.y, self.goal.position.x - self.pose.position.x)
        rotation_error = self.normalize_angle(goal_heading-robot_angle)
        self.get_logger().info(f"rot: {rotation_error, goal_heading,robot_angle}")
        rot_val = K_rot*rotation_error
        x_val = ATTACKER_MAX_SPEED if self.is_attacker else TURTLEBOT_MAX_SPEED
        self.cmd_pub.publish(Twist(linear=Vector3(x=x_val), angular=Vector3(z=float(rot_val))))





    def plan_curve(self, start: PoseStamped, goal: PoseStamped):
        msg = PlanCurve.Goal()
        msg.start = start
        msg.goal = goal

        self.get_logger().info(f"waiting for server...")
        self.path_planner_client.wait_for_server()
        self.get_logger().info(f"sending goal {msg}")
        self.future = self.path_planner_client.send_goal_async(msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f"rejected :(")

        self.get_logger().info(f"accepted")

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result: PlanCurve.Result = future.result().result

        self.path = result.plan
        self.path_tracker = 0
        self.get_logger().info(f"{result.plan}")


def main():
    rclpy.init()
    hcp = Controller()
    rclpy.spin(hcp)

    hcp.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()