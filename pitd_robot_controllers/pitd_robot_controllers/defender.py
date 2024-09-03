import rclpy
from rclpy.node import Node

class Defender(Node):
    def __init__(self) -> None:
        pass
    
def main(args=None):
    rclpy.init(args=args)
    defender = Defender()
    rclpy.spin(defender)
    
    defender.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()