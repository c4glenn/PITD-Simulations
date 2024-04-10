import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from pitd_interfaces.msg import CaptureInfo

from tqdm import tqdm

import numpy as np
class hyperviser_param_setter(Node):
    def __init__(self) -> None:
        super().__init__("param_setter")
        self.defender_controller_client = self.create_client(SetParameters, '/defender_controller/set_parameters')
        self.pathing_client = self.create_client(SetParameters, '/Pather/set_parameters')
        self.attacker_controller_client = self.create_client(SetParameters, '/attacker_controller/set_parameters')
        self.world_controller_client = self.create_client(SetParameters, '/world_handler/set_parameters')
        self.defender_future = None
        self.pathing_future = None
        self.attacker_future = None
        self.world_future = None
        # self.looper = self.create_timer(1, self.loop)


    def loop(self):
        if self.defender_future:
            if self.defender_future.done():
                print("defender is done")

        if self.pathing_future:
            if self.pathing_future.done():
                print("pathing is done")

        if self.attacker_future:
            if self.attacker_future.done():
                print("attackiung is done")

        if self.world_future:
            if self.world_future.done():
                print("world is done")

    def set_params(self, attacker_speed, attacker_sensing_radius, defender_sensing_radius, target_radius):
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name="attacker_speed", value=ParameterValue(type=3, double_value=attacker_speed)),
            Parameter(name="attacker_sensing_radius", value=ParameterValue(type=3, double_value=attacker_sensing_radius)),
            Parameter(name="defender_sensing_radius", value=ParameterValue(type=3, double_value=defender_sensing_radius)),
            Parameter(name="target_radius", value=ParameterValue(type=3, double_value=target_radius))
        ]

        self.defender_future = self.defender_controller_client.call_async(request)
        self.pathing_future = self.pathing_client.call_async(request)
        self.attacker_future = self.attacker_controller_client.call_async(request)
        self.world_future = self.world_controller_client.call_async(request)

class hyperviser(Node):
    def __init__(self) -> None:
        super().__init__("hyperviser")
        self.param_setter = hyperviser_param_setter()
        self.capture_sub = self.create_subscription(CaptureInfo, "/capture_stats", self.capture_callback, 10)
        self.update = self.update_parms()
        self.current_params = next(self.update)
        self.param_setter.set_params(self.current_params[0], self.current_params[1], self.current_params[2], self.current_params[3])
        self.looper = self.create_timer(1/30, self.loop)
        self.reset_count = 0
        self.start_val = 12.5
        self.p_bar = tqdm(range(100))
        # print(f"~~~~~~ {self.current_params} ~~~~")
    
    def loop(self):
        rclpy.spin_once(self.param_setter, timeout_sec=0)


    def update_parms(self):
        self.start_val = 11.5
        vals = [(1.3, 11.5), (1.3, 12.0), (1.3, 12.5), (1.3, 13.0), (1.4, 10.5), (1.4, 11.0), (1.4, 11.5), (1.4, 12.0), (1.4, 12.5), (1.4, 13.0), (1.5, 10.5), (1.5, 11.0), (1.5, 11.5), (1.5, 12.0), (1.5, 12.5), (1.5, 13.0), (1.6, 10.5), (1.6, 11.0), (1.6, 11.5), (1.6, 12.0), (1.6, 12.5), (1.6, 13.0), (1.7, 10.5), (1.7, 11.0), (1.7, 11.5), (1.7, 12.0), (1.7, 12.5), (1.7, 13.0)]
        for j in tqdm(vals):
            yield [0.8, j[0], j[1], 5.0]
    def capture_callback(self, msg:CaptureInfo):
        # print(msg)
        self.p_bar.n = msg.arrivals
        self.p_bar.refresh()
        with open("data.txt", "a") as f:
            f.write(str(msg) + "\n")
        if msg.arrivals >= 100:
            self.current_params = next(self.update)
            with open("data.txt", "a") as f:
                f.write(f"~~~~ {self.current_params} ~~~~\n")
            # print(f"~~~~~~ {self.current_params} ~~~~")
            self.param_setter.set_params(self.current_params[0], self.current_params[1], self.current_params[2], self.current_params[3])


def main():
    rclpy.init()
    hyper = hyperviser()
    rclpy.spin(hyper)

if __name__ == "__main__":
    main()