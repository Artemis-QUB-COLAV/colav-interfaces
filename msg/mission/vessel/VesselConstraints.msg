# VesselConstraints.msg
# This message defines the operational constraints of a vessel (e.g., maximum speed, acceleration limits, etc.)
# These constraints are typically used for planning, control, and safety systems.

# Maximum allowed acceleration in m/s^2
float32 max_acceleration

# Maximum allowed deceleration (negative acceleration) in m/s^2
float32 max_deceleration

# Maximum allowed velocity in m/s
float32 max_velocity

# Minimum allowed velocity in m/s (may be used for safety or maneuvering at low speeds)
float32 min_velocity

# Maximum allowed yaw rate (rate of rotation around the vertical axis) in rad/s
float32 max_yaw_rate
