# ObstaclesConfig.msg
# This is the interface publishing raw obstacle states received

std_msgs/Header header # Header makes sense because it sets frame of refernce for the collision obstacle

colav_interfaces/StaticObstacleState[] static_obstacles # Array of static obstacles
colav_interfaces/DynamicObstacleState[] dynamic_obstacles # Array of dynamic obstacles