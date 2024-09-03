import rclpy
from rclpy.node import Node

class ParamServer(Node):
    def __init__(self) -> None:
        self.__init__('param_server')
        
    
def main(args=None):
    rclpy.init(args=args)
    ps = ParamServer()
    rclpy.spin(ps)
    
    ps.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()