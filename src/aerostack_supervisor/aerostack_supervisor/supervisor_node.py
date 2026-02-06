import rclpy
from rclpy.node import Node
from aerostack_interfaces.msg import UavState, RlAction
from std_msgs.msg import String

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('aerostack_supervisor')
        
        self.declare_parameter('uav_name', 'uav1')
        self.uav_name = self.get_parameter('uav_name').get_parameter_value().string_value
        
        # State Subscriber
        self.state_sub = self.create_subscription(
            UavState,
            f'/{self.uav_name}/uav_state',
            self.state_callback,
            10
        )
        
        # System Health Publisher
        self.health_pub = self.create_publisher(
            String,
            f'/{self.uav_name}/system_health',
            10
        )
        
        # Override Subscriber
        self.action_sub = self.create_subscription(
            RlAction,
            f'/{self.uav_name}/rl_action',
            self.action_callback,
            10
        )
        
        self.current_state = "IDLE"
        self.last_heartbeat = self.get_clock().now()
        
        self.create_timer(1.0, self.health_check_timer)
        
        self.get_logger().info(f"AeroStack Supervisor started for {self.uav_name}")

    def state_callback(self, msg):
        self.last_heartbeat = self.get_clock().now()

    def action_callback(self, msg):
        """
        Intercepts RL actions and validates them against safety constraints.
        """
        # Safety check logic here
        pass

    def health_check_timer(self):
        msg = String()
        now = self.get_clock().now()
        dt = (now - self.last_heartbeat).nanoseconds / 1e9
        
        if dt > 1.0:
            self.current_state = "FAILSAFE"
            msg.data = f"STATUS: {self.current_state} (Connection Lost)"
        else:
            self.current_state = "READY"
            msg.data = f"STATUS: {self.current_state}"
            
        self.health_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
