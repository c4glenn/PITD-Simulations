import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from pitd_robot_controllers.robot_base import RobotBase

class Attacker(RobotBase):
    def __init__(self) -> None:
        super().__init__('attacker')
    
def main(args=None):
    rclpy.init(args=args)
    attacker = Attacker()
    rclpy.spin(attacker)
    
    attacker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()