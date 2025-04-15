# GuardsStatus.msg
# A ROS 2 interface utilized by colav_hybrid_eval for publishing dynamic updates continuously.

# Current control mode (e.g., CRUISE, T2LOS, etc.)
string control_mode

# Guard statuses for each transition (true = guard passed, false = guard failed)
colav_interfaces/CruiseDynamics cruise_dynamics
colav_interfaces/T2LOSDynamics t2los_dynamics
colav_interfaces/FBDynamics fallback_dynamics
colav_interfaces/WaypointReachedDynamics waypoint_reached_dynamics

builtin_interfaces/Time timestamp

bool error
string error_message
