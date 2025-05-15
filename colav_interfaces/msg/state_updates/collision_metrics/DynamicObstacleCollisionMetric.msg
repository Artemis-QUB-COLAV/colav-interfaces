# DynamicObstacleCollisionMetric
# Computes dynamic collision metrics between the agent and a moving obstacle.
# Specifically, it calculates:
#   - TCPA (Time to Closest Point of Approach): the time at which the agent and obstacle are closest.
#   - DCPA (Distance at Closest Point of Approach): the minimum distance between the agent and obstacle at TCPA.

string obstacle_tag  # Identifier or label for the dynamic obstacle

float64 tcpa         # Time to Closest Point of Approach (in seconds)
float64 dcpa         # Distance at Closest Point of Approach (in meters)
