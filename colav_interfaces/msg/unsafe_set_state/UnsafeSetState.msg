# UnsafeSet.msg
# Message definition for maritime collision avoidance unsafe set data

std_msgs/Header header

# Dynamic Safety Framework parameters
float64 dsf  # Dynamic Safety Factor - safety distance threshold

# Obstacle classification based on proximity and collision risk
string[] i1    # I1: Primary risk: obstacles approaching ship within DSF
string[] i2    # I2: Secondary risk: obstacles approaching i1 obstacles within DSF  
string[] i3    # I3Temporal risk: obstacles too close to CPA at TCPA
string[] uioi  # UIOI: Union of Indices of Interest: combined set of all unsafe obstacles

# Unsafe set geometry
std_msgs/Float64MultiArray convex_hull_vertices  # Vertices defining the convex hull of the unsafe set,
                                                 # enclosing obstacles in uioi at their current state and TCPA

# Definitions:
# DSF: Dynamic Safety Factor - minimum safe separation distance
# CPA: Closest Point of Approach
# TCPA: Time to Closest Point of Approach