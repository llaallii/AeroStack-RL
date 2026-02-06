import rclpy
from rclpy.node import Node
from aerostack_interfaces.msg import UavState, RlAction
import torch
import numpy as np

class PolicyRunnerNode(Node):
    def __init__(self):
        super().__init__('policy_runner')
        
        self.declare_parameter('uav_name', 'uav1')
        self.declare_parameter('policy_path', '')
        
        self.uav_name = self.get_parameter('uav_name').get_parameter_value().string_value
        self.policy_path = self.get_parameter('policy_path').get_parameter_value().string_value
        
        # Subscribe to UAV State
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
        
        # Load policy (placeholder)
        if self.policy_path:
            self.get_logger().info(f"Loading policy from {self.policy_path}")
            # self.model = torch.load(self.policy_path)
            # self.model.eval()
        
        self.get_logger().info(f"Policy Runner started for {self.uav_name}")

    def state_callback(self, msg):
        """
        Receives state, runs inference, and publishes action.
        """
        # Placeholder for state preprocessing and inference
        # state_tensor = torch.FloatTensor(msg.data).unsqueeze(0)
        # with torch.no_grad():
        #     action = self.model(state_tensor).numpy()
        
        action_msg = RlAction()
        action_msg.header.stamp = self.get_clock().now().to_msg()
        # action_msg.action = action.tolist()
        
        self.action_pub.publish(action_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PolicyRunnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
