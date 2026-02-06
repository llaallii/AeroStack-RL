#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aerostack_interfaces.msg import UavState
from sensor_msgs.msg import LaserScan
import time

class SimHealthCheck(Node):
    def __init__(self):
        super().__init__('sim_health_check')
        
        self.state_count = 0
        self.lidar_count = 0
        self.start_time = time.time()
        
        self.create_subscription(UavState, '/uav1/uav_state', self.state_cb, 10)
        self.create_subscription(LaserScan, '/uav1/scan', self.lidar_cb, 10)
        
        self.timer = self.create_timer(10.0, self.report)
        self.get_logger().info("SITL Health Check started (10s window)...")

    def state_cb(self, msg):
        self.state_count += 1

    def lidar_cb(self, msg):
        self.lidar_count += 1

    def report(self):
        duration = time.time() - self.start_time
        state_rate = self.state_count / duration
        lidar_rate = self.lidar_count / duration
        
        self.get_logger().info(f"--- SITL Health Report ---")
        self.get_logger().info(f"UAV State Rate: {state_rate:.2f} Hz (Target > 50Hz)")
        self.get_logger().info(f"Lidar Scan Rate: {lidar_rate:.2f} Hz (Target > 10Hz)")
        
        if state_rate > 40 and lidar_rate > 8:
            self.get_logger().info("RESULT: PASS")
        else:
            self.get_logger().error("RESULT: FAIL - Performance below requirements")
            
        rclpy.shutdown()

def main():
    rclpy.init()
    node = SimHealthCheck()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
