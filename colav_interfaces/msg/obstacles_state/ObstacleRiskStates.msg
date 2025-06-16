# CollisionMetrics.msg
# This message contains real-time collision metrics computed from the agent's current trajectory.
# The header includes a timestamp and the reference frame (typically 'map') in which these metrics are defined.
# Metrics are computed for both static and dynamic obstacles relative to the agent vessel.

std_msgs/Header header

colav_interfaces/StaticObstacleRiskState[] static_obstacle_risk_states   # Collision metrics for static obstacles
colav_interfaces/DynamicObstacleRiskState[] dynamic_obstacle_risk_states # Collision metrics for dynamic obstacles
