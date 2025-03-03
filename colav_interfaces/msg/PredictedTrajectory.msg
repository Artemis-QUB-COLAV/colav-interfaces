# PredictedTrajectory.msg
# Represents the predicted trajectory of an obstacle

geometry_msgs/Pose2D[] waypoints # Array of predicted waypoinst (positions over time)
builtin_interfaces/Time[] timestamps # Corresponding timestamps for each waypoint
float64 confidence # confidence level of prediction