# VesselGeometry.msg
# This message defines the geometric properties of the vessel, including its shape and acceptance radius.

# loa and beam of the vessel
float64 loa
float64 beam

# Acceptance radius around the vessel, typically used for navigation or collision avoidance, in meters
float32 safety_radius
