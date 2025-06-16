# DynamicObstacleRiskState.msg
# Describes collision risk and navigation-relevant metrics for dynamic obstacles

# === OBSTACLE INFO ===
colav_interfaces/DynamicObstacleState obstacle_state  # Obstacle ID and current state (position, velocity, geometry)

# === RELATIVE STATE METRICS ===
geometry_msgs/Vector3 relative_position               # Vector from agent to obstacle [m]
geometry_msgs/Vector3 relative_velocity               # Obstacle velocity relative to agent [m/s]

float64 raw_distance                                 # Euclidean distance to obstacle center [m]
float64 buffered_clearance                           # Distance to obstacle's surface (accounting for safety radii) [m]

float64 relative_agent_bearing                        # Angle from obstacle's heading to agent [-180, 180] degrees
float64 relative_obstacle_bearing                     # Angle from agent's heading to obstacle [-180, 180] degrees

bool is_risk_obstacle_currently                       # True if obstacle is currently within inflated safety buffer (dsf)

# === PREDICTION METADATA ===
float64 tsf                                            # Time Safety Factor (prediction/event horizon) [s]
float64 dsf                                            # Distance Safety Factor (inflated dynamic safety buffer) [m]

# === CPA METRICS ===
float64 tcpa                                           # Time to Closest Point of Approach [s]
float64 dcpa                                           # Distance at CPA [m]
float64 distance_to_cpa                                # Distance along agent’s path to CPA point [m]

bool is_current_trajectory_colliding                   # True if agent’s current trajectory leads to collision

geometry_msgs/Pose cpa_pose_agent                      # Agent's predicted pose at CPA
geometry_msgs/Pose cpa_pose_obstacle                   # Obstacle's predicted pose at CPA

# === OBSTACLE TRAJECTORY PREDICTION ===
geometry_msgs/Pose[] obstacle_trajectory               # Predicted obstacle trajectory up to CPA or TSF
float64 obstacle_trajectory_dt                         # Time step between trajectory poses [s]
uint32 obstacle_trajectory_timesteps                   # Number of predicted poses

# === COLLISION RISK INDEX (CRI) ===
float64 proximity_risk                                 # Proximity risk component [0.0–1.0]
float64 dcpa_risk                                      # DCPA risk component [0.0–1.0]
float64 tcpa_risk                                      # TCPA risk component [0.0–1.0]
float64 cri                                            # Combined Collision Risk Index [0.0–1.0]
colav_interfaces/CRIClassification cri_classification  # Classification based on CRI thresholds

# === COLREGs CLASSIFICATION ===
colav_interfaces/COLREGClassification colreg_encounter # Encounter type classification (e.g., crossing, overtaking)
