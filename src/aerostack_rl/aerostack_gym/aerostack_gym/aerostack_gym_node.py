import rclpy
from rclpy.node import Node
from aerostack_interfaces.msg import UavState, RlAction
from std_srvs.srv import Empty

class AeroStackGymNode(Node):
    def __init__(self):
        super().__init__('aerostack_gym_node')
        
        self.declare_parameter('uav_name', 'uav1')
        self.uav_name = self.get_parameter('uav_name').get_parameter_value().string_value
        
        # State Subscriber
        self.state_sub = self.create_subscription(
            UavState,
            f'/{self.uav_name}/uav_state',
            self.state_callback,
            10
        )
        
        # Action Publisher
        self.action_pub = self.create_publisher(
            RlAction,
            f'/{self.uav_name}/rl_action',
            10
        )
        
        # Services for Gym Environment Control
        self.reset_service = self.create_service(Empty, f'/{self.uav_name}/reset_env', self.reset_callback)
        self.step_service = self.create_service(Empty, f'/{self.uav_name}/step_env', self.step_callback)
        
        self.latest_state = None
        self.get_logger().info(f"AeroStack Gym Node started for {self.uav_name}")

    def state_callback(self, msg):
        self.latest_state = msg

    def reset_callback(self, request, response):
        """
        Resets the simulation environment.
        """
        self.get_logger().info("Resetting environment...")
        # Add logic to reset SITL/Gazebo
        return response

    def step_callback(self, request, response):
        """
        Triggers a simulation step if required (for stepped sims).
        """
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AeroStackGymNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
