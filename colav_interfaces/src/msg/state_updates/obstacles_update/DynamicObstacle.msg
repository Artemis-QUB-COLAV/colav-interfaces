# DynamicObstacle.msg
# Represents a dynamic obstacle in a 2d Environment
string tag
string type

geometry_msgs/Pose pose
float64 velocity
float64 acceleration
float64 yaw_rate

# The geometric shape representing the obstacle
colav_interfaces/DynamicObstacleGeometry geometry
