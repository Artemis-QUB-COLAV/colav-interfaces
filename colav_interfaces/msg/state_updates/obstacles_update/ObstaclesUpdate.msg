# ObstaclesConfig.msg
std_msgs/Header header # Header makes sense because it sets frame of refernce for the collision obstacle
string mission_tag
StaticObstacle[] static_obstacles # Array of static obstacles
DynamicObstacle[] dynamic_obstacles # Array of dynamic obstacles