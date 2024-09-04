import rclpy
from rclpy.node import Node

from pitd_interfaces.srv import GetParams

class ParamServer(Node):
    def __init__(self) -> None:
        self.__init__('param_server')
        self.srv = self.create_service(GetParams, '/settings', self.param_callback)
        self.attacker_speed = 0.8 * self.defender_speed
        self.defender_speed = 0.2
        self.attacker_sensing_radius = 1
        self.defender_sensing_radius = 1
        self.target_radius = 0.5
        

    def param_callback(self, request, response):
        response.attacker_speed = self.attacker_speed
        response.defender_speed = self.defender_speed
        response.attacker_sensing_radius = self.attacker_sensing_radius
        response.defender_sensing_radius = self.defender_sensing_radius
        response.target_radius = self.target_radius
        return response

def main(args=None):
    rclpy.init(args=args)
    ps = ParamServer()
    rclpy.spin(ps)
    
    ps.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()