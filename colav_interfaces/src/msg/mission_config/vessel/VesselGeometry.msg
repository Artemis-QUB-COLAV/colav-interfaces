# VesselGeometry.msg
# This message defines the geometric properties of the vessel, including its shape and acceptance radius.

# Polygon representing the vessel's shape, defined by a set of points in 2D or 3D space (depending on application)
geometry_msgs/Polygon polyshape

# Acceptance radius around the vessel, typically used for navigation or collision avoidance, in meters
float32 acceptance_radius
