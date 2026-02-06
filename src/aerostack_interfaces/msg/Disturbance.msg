std_msgs/Header header

# Wind vector in world frame (m/s)
geometry_msgs/Vector3 wind_vector

# Duration of the disturbance (seconds)
# Set to -1 for permanent wind
float32 duration

# Type of disturbance (optional for future expansion)
uint8 TYPE_WIND = 0
uint8 TYPE_SENSOR_NOISE = 1
uint8 type 0
