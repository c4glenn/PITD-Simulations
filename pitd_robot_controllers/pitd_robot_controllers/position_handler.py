import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Point, Quaternion
from tf_transformations import quaternion_from_euler

import numpy as np

RUN = 2


def make_pose(x=0, y=0, theta=0) -> Pose:
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    q = quaternion_from_euler(0, 0, theta)
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    return p

# defender start, defender engagement, defender end
s1_d = [make_pose(), 
        make_pose(0.171, 0.3365, 1.1),
        make_pose(0.253*2, 0.729*2, 1.41)]

s2_d = [make_pose(0.253*2, 0.729*2, 1.41),
        make_pose(0.37, 0.075, 0.2),
        make_pose(0.7285*2, 0.2545*2, 0.51)]

s3_d = [make_pose(),
        make_pose(0.7285*2, 0.2545*2, 0.51),
        make_pose(0, 0)]

#attacker start, attacker engagment, attacker end
s1_a = [make_pose(1.1984, 2.3546, 1.1-np.pi), 
        make_pose(0.2621*2, 0.515*2, 1.1-np.pi), 
        make_pose(0.253*2, 0.729*2, 1.41)]

s2_a = [make_pose(2.5893, 0.5249, 0.2-np.pi),
        make_pose(0.5664*2, 0.1148*2, 0.2-np.pi),
        make_pose(0.7285*2, 0.2545*2, 0.51)]

s3_a = [make_pose(-2.6156, -0.3728, -3-np.pi),
        make_pose(-2.6156, -0.3728, -3-np.pi),
        make_pose(-0.4841*.8, -0.0690*.8, -3-np.pi)]


# "{position: {x: 1.1984, y: 2.3546}}"

class PositionHandler(Node):
    def __init__(self) -> None:
        super().__init__("position_handler")
        self.defender_goal_reached_sub = self.create_subscription(Bool, "/defender/goal_finished", lambda x: self.goal_reached_callback("defender", x), 10)
        self.defender_goal_pub = self.create_publisher(Pose, "/defender/goal", 10)
        self.defender_at_goal = False

        self.attacker_goal_reached_sub = self.create_subscription(Bool, "/attacker/goal_finished", lambda x: self.goal_reached_callback("attacker", x), 10)
        self.attacker_goal_pub = self.create_publisher(Pose, "/attacker/goal", 10)
        self.attacker_at_goal = False

        self.logic_loop_tmr = self.create_timer(1/30, self.logic_loop_tmr_callback)
        if RUN==1:
            self.attacker_poses = s1_a
            self.defender_poses = s1_d
        elif RUN==2:
            self.attacker_poses = s2_a
            self.defender_poses = s2_d
        else:
            self.attacker_poses = s3_a
            self.defender_poses = s3_d
        self.state = 0
        self.state_lock = True



    def logic_loop_tmr_callback(self) -> None:
        self.get_logger().info(f"{self.state}")
        if self.state >= len(self.defender_poses):
            return

        if not self.attacker_at_goal or not self.defender_at_goal:
            self.state_lock = False

        if self.attacker_at_goal and self.defender_at_goal and not self.state_lock:
            self.state += 1
            self.state_lock = True
        
        self.defender_goal_pub.publish(self.defender_poses[self.state])
        self.attacker_goal_pub.publish(self.attacker_poses[self.state])
        # self.get_logger().info(f"attacker: {self.attacker_poses[self.state]}")

    def goal_reached_callback(self, robot: str, msg: Bool) -> None:
        if robot == "attacker":
            self.attacker_at_goal = msg.data
        else:
            self.defender_at_goal = msg.data

def main():
    rclpy.init()
    ph = PositionHandler()
    rclpy.spin(ph)

    ph.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()