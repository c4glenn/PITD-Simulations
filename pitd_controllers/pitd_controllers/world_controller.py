import rclpy
from rclpy.node import Node
import random
import numpy as np
from gazebo_msgs.srv import DeleteEntity
from pitd_interfaces.msg import AttackerSpawn, CaptureInfo
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
from rcl_interfaces.msg import SetParametersResult


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [w, x, y, z]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

class World_Controller(Node):
    def __init__(self) -> None:
        super().__init__("world_handler")

        self.delete_client = self.create_client(DeleteEntity, "/delete_entity")
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service (delete_entity) Not Avaliable, waiting again.... ')

        self.declare_parameter("attack_spawn_pattern", 1)
        self.declare_parameter("defender_sensing_radius", 10.0)
        self.declare_parameter("target_radius", 5.0)
        self.declare_parameter("capture_distance", 0.3)
        self.attacker_subs = {} # the actual subscriptions are created dynamically in build_and_publish

        self.defender_tracking = self.create_subscription(Odometry, "/defender/odom", self.track_defender, 10)
        self.attacker_deleting = self.create_publisher(String, "/pitd/delete", 10)


        self.ratio_publisher = self.create_publisher(CaptureInfo, "/capture_stats", 10)

        self.parameters_callback(None)

        self.info = CaptureInfo()

        self.total_breached = 0
        self.captured_num = 0
        self.breached = []
        self.captured = []
        self.total_spawned = 0

        self.attackers = []
        self.next_attacker_num = 0

        self.delete_trys = {}

        self.defender = Point()

        self.reset_count = 0
        self.reset_num = 100

        self.attacker_publisher = self.create_publisher(AttackerSpawn, "/pitd/spawn", 10)
        
        self.rand_pose_attack()

        self.get_logger().info(f"attack pattern: {self.attack_spawn_pattern}")

        self.add_on_set_parameters_callback(self.parameters_callback)


    def parameters_callback(self, params):
        self.attack_spawn_pattern = self.get_parameter("attack_spawn_pattern").get_parameter_value().integer_value
        self.sensing_radius = self.get_parameter("defender_sensing_radius").get_parameter_value().double_value
        self.target_radius = self.get_parameter("target_radius").get_parameter_value().double_value
        self.capture_distance = self.get_parameter("capture_distance").get_parameter_value().double_value

        return SetParametersResult(successful=True)
        
          
    
    def track_defender(self, data: Odometry):
        self.defender=data.pose.pose.position

    def rand_pose_attack(self, name=None):
        if not name:
            name = f"attacker{self.next_attacker_num}"
        #generates a distance from origin outside the sensing radius but not too far (+5 is magic and should be tweaked)
        r = random.uniform(self.sensing_radius, self.sensing_radius+1)
        #generates the angle to spawn at
        theta = random.uniform(0, 2*np.pi)

        x = r * np.cos(theta)
        y = r * np.sin(theta)

        self.get_logger().info(f"spawning {name} at x:{x} y:{y}")

        self.build_and_publish(x, y, name)

    def build_and_publish(self, x, y, name):
        quat = quaternion_from_euler(0, 0, np.arctan2(y, x) + np.pi)
        attacker = AttackerSpawn()
        attacker.name = name
        attacker.pose = Pose(position=Point(x=x,y=y), orientation=Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3]))
        self.attackers.append(attacker.name)
        self.attacker_publisher.publish(attacker)
        self.attacker_subs[name] = self.create_subscription(Odometry, f"{name}/odom", lambda x: self.track_attacker(x, name), 10)
        self.next_attacker_num += 1
        self.total_spawned += 1

    def delete_attacker(self, name:str):
        print(self.total_spawned)
        self.info.arrivals = self.total_spawned
        self.info.capture_percentage = (self.captured_num) / self.total_spawned

        self.ratio_publisher.publish(self.info)

        if self.total_spawned >= self.reset_num:
            self.reset_count += 1
            self.total_spawned = 0
            self.captured_num = 0

        potential_fail = False
        if name not in self.attackers:
            self.get_logger().warning(f"attempt to delete {name} that is not in attackers")
            potential_fail = True
        

        self.delete_trys[name] = self.delete_trys.get(name, 0) + 1
        if self.delete_trys[name] >= 3:
            potential_fail = True

        request = DeleteEntity.Request()
        request.name = name

        self.future = self.delete_client.call_async(request)

        self.delete_trys[name] = 0
        #TODO: make this safely delete... currently a memory leak
        #self.attackers.pop(self.attackers.index(name))
        #self.attacker_subs[name].destroy() 
        #self.attacker_deleting.publish(String(data=name))
        if self.attack_spawn_pattern == 1:
            self.rand_pose_attack()


    def track_attacker(self, data: Odometry, name:str):
        self.attack_spawn_pattern = self.get_parameter("attack_spawn_pattern").get_parameter_value().integer_value
        self.sensing_radius = self.get_parameter("defender_sensing_radius").get_parameter_value().double_value
        self.target_radius = self.get_parameter("target_radius").get_parameter_value().double_value
        self.capture_distance = self.get_parameter("capture_distance").get_parameter_value().double_value

        if name in self.breached or name in self.captured:
            return
        if name not in self.attackers:
            self.get_logger().warning(f"{name} not in attackers list how did you spawn it?")

        pose = data.pose.pose.position

        if np.abs(pose.x - float(self.defender.x)) <= self.capture_distance and np.abs(pose.y - float(self.defender.y))<=self.capture_distance:
            self.get_logger().info(f"{name} was captured")
            self.captured.append(name)
            self.captured_num += 1
            self.delete_attacker(name)
            
            return

        if np.sqrt((pose.x ** 2) + (pose.y ** 2)) <= self.target_radius:
            self.get_logger().info(f"{name} breached")
            self.breached.append(name)
            self.total_breached += 1
            self.delete_attacker(name)
            return
        


        

def main():
    rclpy.init()
    wc = World_Controller()
    rclpy.spin(wc)

    wc.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()