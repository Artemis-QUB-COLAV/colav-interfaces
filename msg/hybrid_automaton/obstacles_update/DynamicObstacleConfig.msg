# DynamicObstacle.msg
# Represents a dynamic obstacle in a 2d Environment
string id
string type

geometry_msgs/Pose pose
float64 velocity
float64 acceleration
float64 yaw_rate
string state  # e.g., "active", "inactive", "stationary"

# The geometric shape representing the obstacle
geometry_msgs/Polygon geometry
float64 safety_radius  
