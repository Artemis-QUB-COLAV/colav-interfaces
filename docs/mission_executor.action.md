# MissionExecutor.action

## Description
The `MissionExecutor` action is responsible for executing a mission based on the provided `MissionRequest`. It returns the success status and a message upon completion while providing feedback in the form of serialized protobuf data control feedback commands.

---

## Goal
The goal of this action is to execute a mission using the provided `MissionRequest` message.

### MissionRequest (colav_interfaces/MissionRequest)
The mission request contains details necessary to execute a mission, including the mission's tag, timestamps, the vessel performing the mission, and the mission's initial and goal positions.

#### Fields:
- `string mission_tag`  
  *A unique identifier for the mission.*
- `string mission_sent_timestamp`  
  *The timestamp when the mission request was sent.*
- `colav_interfaces/Vessel vessel`  
  *The vessel assigned to perform the mission.*
- `geometry_msgs/Point32 mission_init_position`  
  *The initial position of the mission.*
- `geometry_msgs/Point32 mission_goal_position`  
  *The goal position of the mission.*
- `float32 mission_goal_acceptance_radius`  
  *The radius within which the mission is considered successfully completed.*

---

## Result
The result provides the outcome of the mission execution.

#### Fields:
- `bool success`  
  *Indicates whether the mission execution was successful.*
- `string status_message`  
  *A message describing the execution status (e.g., success, failure reason).*

---

## Feedback
Feedback provides real-time updates on the mission execution status.

#### Fields:
- `std_msgs/ByteMultiArray serialised_protobuf_controller_feedback`  
  *Serialized protobuf data representing controller feedback.*

---

## Supporting Messages

### Vessel (colav_interfaces/Vessel)
This message provides detailed information about the vessel executing the mission.

#### Fields:
- `string tag`  
  *A unique identifier for the vessel.*
- `string type`  
  *The type or category of the vessel.*
- `colav_interfaces/VesselConstraints dynamic_constraints`  
  *Operational constraints related to vessel motion.*
- `colav_interfaces/VesselGeometry geometry`  
  *Geometric representation of the vessel.*

### VesselConstraints (colav_interfaces/VesselConstraints)
Defines the operational constraints for a vessel.

#### Fields:
- `float32 max_acceleration`  
  *Maximum allowed acceleration (m/s²).*
- `float32 max_deceleration`  
  *Maximum allowed deceleration (m/s²).*
- `float32 max_velocity`  
  *Maximum allowed velocity (m/s).*
- `float32 min_velocity`  
  *Minimum allowed velocity (m/s).*
- `float32 max_yaw_rate`  
  *Maximum yaw rate (rad/s).*

### VesselGeometry (colav_interfaces/VesselGeometry)
Defines the geometric properties of the vessel.

#### Fields:
- `geometry_msgs/Polygon polyshape`  
  *The polygonal shape of the vessel.*
- `float32 acceptance_radius`  
  *The vessel's acceptance radius for navigation (meters).*
