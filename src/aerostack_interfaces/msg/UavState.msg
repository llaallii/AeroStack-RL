std_msgs/Header header

# ENU Coordinate Frame (SI Units)
# Position (meters), Orientation (quaternion)
geometry_msgs/PoseWithCovariance pose

# Linear (m/s), Angular (rad/s) in ENU
geometry_msgs/TwistWithCovariance twist

# Flight Status
bool armed
bool failsafe
uint8 nav_state
uint8 land_state

# Battery status
float32 battery_remaining  # Normalized 0..1

# Constants (Matching PX4 or standard definitions where applicable)
uint8 NAV_STATE_MANUAL = 0
uint8 NAV_STATE_ALTCTL = 1
uint8 NAV_STATE_POSCTL = 2
uint8 NAV_STATE_AUTO   = 3
uint8 NAV_STATE_OFFBOARD = 4

uint8 LAND_STATE_ON_GROUND = 0
uint8 LAND_STATE_IN_AIR    = 1
uint8 LAND_STATE_LANDING   = 2
