# StaticObstacle.msg
# Represents a Static Obstsacle in a 2d environment

string tag                  # Unique identifier for the static obstacl
string type                 # obstacle classification
geometry_msgs/Pose pose     # Obstacles pose
colav_interfaces/StaticObstacleGeometry geometry  # The geometric shape representing the obstacle
