# ObstaclesConfig.msg
std_msgs/Header header
string mission_tag
StaticObstacle[] static_obstacles # Array of static obstacles
DynamicObstacle[] dynamic_obstacles # Array of dynamic obstacles