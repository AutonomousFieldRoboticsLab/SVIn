# depth_node_py/msg/Depth
# Message definition for Bar30 depth sensor data

std_msgs/Header header

# Depth measurement in meters
float64 depth

# Variance of the depth measurement
float64 depth_variance

# Altitude measurement in meters (distance from bottom)
float64 altitude

# Variance of the altitude measurement
float64 altitude_variance
