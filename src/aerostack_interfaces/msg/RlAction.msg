std_msgs/Header header

# Normalized thrust (0.0 to 1.0)
# Represents collective thrust along the vehicle body +Z axis
float32 thrust

# Body rates in rad/s (Body Frame: Roll, Pitch, Yaw)
geometry_msgs/Vector3 body_rate

# Action Type Switch
uint8 action_type

# Constants for action type
uint8 RATE_THRUST = 0
uint8 ATT_THRUST  = 1
