import rclpy
import numpy as np
from rclpy.node import Node
from pitd_robot_controllers import robot_base


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool


class Defender(robot_base):
    def __init__(self) -> None:
        super().__init__("defender")
        self.pose = Pose()
        self.attacker_pose = None
        
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
        pass #TODO: write motion algorithm
    
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