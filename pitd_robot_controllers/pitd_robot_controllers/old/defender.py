import rclpy
import numpy as np
from rclpy.node import Node
from pitd_robot_controllers.robot_base import RobotBase


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion 

# from tf2_tools import 

DIST_TOL = 0.01
ROT_TOL = 0.1
TURTLEBOT_MAX_SPEED = 0.2

K_rot = 0.1
K_trans = 0.01


class Defender(RobotBase):
    def __init__(self) -> None:
        super().__init__("defender")
        self.pose = Pose()
        self.attacker_pose = None
        self.cmd_pub = self.create_publisher(Twist, "/defender/cmd_vel", 10)
        
    def call_each_step(self) -> None:
        super().call_each_step()
        if not self.attacker_pose:
            self.no_information_phase()
        else:
            if self.distance(self.pose, self.attacker_pose) <= self.attacker_sensing_radius:
                self.full_information_phase()
            else:
                self.partial_information_phase()
        
    def move_towards_goal(self):
        #turn move turn
        robot_angle = euler_from_quaternion(self.pose.orientation)[2]

        distance_error = np.sqrt((self.goal.position.x - self.pose.position.x)**2 + (self.goal.position.y - self.pose.position.y)**2)
        if distance_error <= DIST_TOL:
            #inside distance just fix headin
            goal = euler_from_quaternion(self.goal.orientation)[2]
            rotation_error = self.normalize_angle(robot_angle - goal)
            if abs(rotation_error) <= ROT_TOL:
                self.cmd_pub.publish(Twist())
                return
            
            self.cmd_pub.publish(Twist(angular=Vector3(z=K_rot*rotation_error)))
            return
        
        goal_heading = np.arctan2(self.goal.position.x - self.pose.position.x, self.goal.position.y - self.pose.position.y)
        rotation_error = self.normalize_angle(robot_angle - goal_heading)
        if abs(rotation_error) <= np.pi/4:
            self.cmd_pub.publish(Twist(linear=Vector3(x=K_trans*distance_error), angular=Vector3(z=K_rot*rotation_error)))
            
    
    def no_information_phase(self):
        self.set_goal(Pose())
        self.move_towards_goal()
    
    def partial_information_phase(self):
        goal = Pose(position=Point(), orientation=Quaternion())
        # TODO find goal pose
        self.set_goal(goal)
        self.move_towards_goal()
    
    def full_information_phase(self):
        goal = self.attacker_pose
        self.set_goal(goal)
        self.move_towards_goal()
    
    def pose_callback(self, robot: str, msg: Odometry) -> None:
        if robot == "defender":
            self.pose = msg.pose.pose
        if robot == "attacker":
            temp_save = msg.pose.pose
            if self.distance(Pose(), temp_save) <= self.defender_sensing_radius:
                self.attacker_pose = temp_save
            else:
                self.attacker_pose = None
    
def main(args=None):
    rclpy.init(args=args)
    defender = Defender()
    rclpy.spin(defender)
    
    defender.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()