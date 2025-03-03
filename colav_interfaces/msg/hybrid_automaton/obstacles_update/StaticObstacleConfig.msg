# StaticObstacle.msg
# Represents a Static Obstsacle in a 2d environment

string id
string type            # Unique identifier for the static obstacle
geometry_msgs/Pose pose                  # Position and orientation of the obstacle
geometry_msgs/Polygon geometry                 # Shape of the obstacle defined by its vertices
float64 safety_radius                       # Safety margin around the obstacle (optional)
