# GuardsStatus.msg
# A ROS 2 interface utilized by colav_hybrid_eval for publishing guard statuses continuously.

# Current control mode (e.g., CRUISE, T2LOS, etc.)
string control_mode

# Stores the guard names evaluated for the current control mode
string[] guard_names

# Guard statuses for each transition (true = guard passed, false = guard failed)
bool cruise_to_t2los1
bool cruise_to_t2los2
bool cruise_to_waypoint_reached
bool cruise_to_fb  # Fallback guard for CRUISE mode

bool t2los_to_cruise
bool t2los_to_fb  # Fallback guard for T2LOS mode
bool t2los_to_waypoint_reached

bool waypoint_reached_to_cruise  # Transition from WAYPOINT_REACHED to CRUISE mode

# Fallback Guards
# No Fallback Guards currently

# Optionally, include timestamps if you need to track when each transition was evaluated
builtin_interfaces/Time timestamp

# error indication
bool error
string error_message
