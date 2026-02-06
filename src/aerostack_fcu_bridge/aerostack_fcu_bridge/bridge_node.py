import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from aerostack_interfaces.msg import UavState, RlAction
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
try:
    from px4_msgs.msg import VehicleOdometry, VehicleStatus
except ImportError:
    from px4.msg import VehicleOdometry, VehicleStatus

# Note: Using px4_msgs or px4 depending on what's available in the workspace.

class FcuBridgeNode(Node):
    def __init__(self):
        super().__init__('fcu_bridge')
        
        # Parameters
        self.declare_parameter('uav_name', 'uav1')
        self.uav_name = self.get_parameter('uav_name').get_parameter_value().string_value
        
        # QoS Profiles
        from rclpy.qos import DurabilityPolicy
        self.fcu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers
        self.uav_state_pub = self.create_publisher(UavState, f'/{self.uav_name}/uav_state', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            VehicleOdometry, 
            '/fmu/out/vehicle_odometry', 
            self.odom_callback, 
            self.fcu_qos
        )
        
        # Try status_v1 first (V1.14+), fallback to standard status
        status_topic = '/fmu/out/vehicle_status_v1'
        self.status_sub = self.create_subscription(
            VehicleStatus,
            status_topic,
            self.status_callback,
            self.fcu_qos
        )
        
        # State variables
        self.current_status = VehicleStatus()
        
        self.get_logger().info(f"AeroStack FCU Bridge started for {self.uav_name}")

    def status_callback(self, msg):
        self.get_logger().info("Received VehicleStatus")
        self.current_status = msg

    def odom_callback(self, msg):
        self.get_logger().info("Received VehicleOdometry")
        """
        Translates PX4 VehicleOdometry (NED) to AeroStack UavState (ENU).
        """
        state = UavState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'world'
        
        # NED (PX4) to ENU (ROS) Conversion
        # Position: x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned
        # Orientation: q_enu = [qy, qx, -qz, qw]
        
        state.pose.pose.position.x = msg.position[1]
        state.pose.pose.position.y = msg.position[0]
        state.pose.pose.position.z = -msg.position[2]
        
        state.pose.pose.orientation.x = msg.q[2] # qy
        state.pose.pose.orientation.y = msg.q[1] # qx
        state.pose.pose.orientation.z = -msg.q[3] # -qz
        state.pose.pose.orientation.w = msg.q[0] # qw
        
        # Velocity conversion
        state.twist.twist.linear.x = msg.velocity[1]
        state.twist.twist.linear.y = msg.velocity[0]
        state.twist.twist.linear.z = -msg.velocity[2]
        
        # Status Mapping
        state.armed = (self.current_status.arming_state == 2) # ARMING_STATE_ARMED
        state.failsafe = self.current_status.failsafe
        
        # Nav State Mapping
        # mapping PX4 nav_state to AeroStack UavState nav_state
        px4_nav = self.current_status.nav_state
        if px4_nav == 0: # MANUAL
            state.nav_state = UavState.NAV_STATE_MANUAL
        elif px4_nav == 1: # ALTCTL
            state.nav_state = UavState.NAV_STATE_ALTCTL
        elif px4_nav == 2: # POSCTL
            state.nav_state = UavState.NAV_STATE_POSCTL
        elif px4_nav == 17: # OFFBOARD (Standard PX4 value)
            state.nav_state = UavState.NAV_STATE_OFFBOARD
        else:
            state.nav_state = UavState.NAV_STATE_AUTO
            
        self.uav_state_pub.publish(state)

def main(args=None):
    rclpy.init(args=args)
    node = FcuBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
