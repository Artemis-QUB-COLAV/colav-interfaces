# ObstaclesConfig.msg
std_msgs/Header header
string mission_id
StaticObstacleConfig[] static_obstacles # Array of static obstacles
DynamicObstacleConfig[] dynamic_obstacles # Array of dynamic obstacles
string timestamp
string timestep