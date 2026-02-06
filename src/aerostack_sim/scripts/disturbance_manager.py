#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aerostack_interfaces.msg import Disturbance
from geometry_msgs.msg import Vector3
import random
import time

class DisturbanceManager(Node):
    def __init__(self):
        super().__init__('disturbance_manager')
        
        # Parameters
        self.declare_parameter('enable_random_gusts', False)
        self.enable_random = self.get_parameter('enable_random_gusts').value
        
        # Subscribers
        self.create_subscription(Disturbance, '/uav1/disturbance', self.disturbance_callback, 10)
        
        # Note: In Gazebo Harmonic, wind is often controlled via the /world/<world_name>/wind topic
        # which expects gz.msgs.EntityWrench or specific wind messages.
        # However, the wind-effects system plugin also listens to /wind topic in GZ.
        
        # We will bridge this via ros_gz_bridge later if needed, 
        # but for now we'll simulate the logic.
        
        self.get_logger().info("Disturbance Manager started")
        
        if self.enable_random:
            self.timer = self.create_timer(5.0, self.random_gust_timer)

    def disturbance_callback(self, msg):
        self.get_logger().info(f"Received disturbance: Wind={msg.wind_vector}, Duration={msg.duration}s")
        # Logic to send this to Gazebo via bridge or service
        # For now, we log it. In a full implementation, we would publish to bridged /wind topic.

    def random_gust_timer(self):
        if not self.enable_random:
                return
        
        gust_strength = random.uniform(0.5, 3.0)
        self.get_logger().info(f"Generating random gust: {gust_strength} m/s")
        # Publish random gust logic here

def main(args=None):
    rclpy.init(args=args)
    node = DisturbanceManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
