# Mission.msg
# This message defines the details of a mission, including the mission's tag, timestamps, 
# the vessel performing the mission, and the mission's initial and goal positions.
string mission_tag
string mission_sent_timestamp
colav_interfaces/Vessel vessel
geometry_msgs/Point32 mission_init_position
geometry_msgs/Point32 mission_goal_position
float32 mission_goal_acceptance_radius
