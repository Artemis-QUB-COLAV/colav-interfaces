# Mission.msg
# This message defines the details of a mission, including the mission's tag, timestamps, 
# the vessel performing the mission, and the mission's initial and goal positions.
string tag
std_msgs/Header header
colav_interfaces/Vessel vessel
geometry_msgs/Point32 init_position
colav_interfaces/Waypoint[] goal_waypoints
