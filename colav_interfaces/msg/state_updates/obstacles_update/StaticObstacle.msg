# StaticObstacle.msg
# Represents a Static Obstsacle in a 2d environment

string tag
string type            # Unique identifier for the static obstacle
geometry_msgs/Pose pose                  # Position and orientation of the obstacle
colav_interfaces/StaticObstacleGeometry geometry  # The geometric shape representing the obstacle
