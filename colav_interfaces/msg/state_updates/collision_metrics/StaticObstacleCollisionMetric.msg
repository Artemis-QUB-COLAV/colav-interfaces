# StaticObstacleCollisionMetric
# Computes collision metrics for a static obstacle relative to the agent's current trajectory.
# Specifically, it calculates:
#   - The minimum Euclidean distance from any point on the agent's trajectory to the static obstacle.
#   - The time at which this minimum distance occurs.

string obstacle_tag           # Identifier or label for the static obstacle

float64 min_distance          # Minimum Euclidean distance to the obstacle over the trajectory
float64 time_to_min_distance  # Time (relative or absolute) at which the minimum distance occurs
