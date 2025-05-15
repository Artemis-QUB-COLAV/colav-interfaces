# CollisionMetrics.msg
# This message contains real-time collision metrics computed from the agent's current trajectory.
# The header includes a timestamp and the reference frame (typically 'map') in which these metrics are defined.
# Metrics are computed for both static and dynamic obstacles relative to the agent vessel.

std_msgs/Header header

colav_interfaces/StaticObstacleCollisionMetric[] static_obstacles   # Collision metrics for static obstacles
colav_interfaces/DynamicObstacleCollisionMetric[] dynamic_obstacles # Collision metrics for dynamic obstacles
