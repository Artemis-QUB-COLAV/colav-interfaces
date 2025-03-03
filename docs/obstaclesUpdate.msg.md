# ObstaclesConfig Message

The `ObstaclesUpdate.msg` message provides mission-specific information about static and dynamic obstacles within the environment. It includes an array of both static and dynamic obstacles, each described by position, motion attributes, and geometric properties.

## Message Definition


## Field Descriptions

- **`header`** (`std_msgs/Header`): Standard ROS message header containing a timestamp and frame information.
- **`mission_tag`** (`string`): Unique identifier for the mission associated with this obstacle configuration.
- **`static_obstacles`** (`StaticObstacle[]`)[]: An array of static obstacles.
- **`dynamic_obstacles`** (`DynamicObstacle[]`): An array of dynamic obstacles.

---

## `DynamicObstacle.msg`

Represents a dynamic obstacle in a 2D environment.

### Message Definition

### Field Descriptions

- **`tag`** (`string`): Unique identifier for the obstacle.
- **`type`** (`string`): Type or classification of the obstacle (e.g., "vehicle", "ship", "drone").
- **`pose`** (`geometry_msgs/Pose`): The current position and orientation of the obstacle.
- **`velocity`** (`float64`): The linear velocity of the obstacle (m/s).
- **`acceleration`** (`float64`): The linear acceleration of the obstacle (m/s²).
- **`yaw_rate`** (`float64`): The angular velocity of the obstacle around the yaw axis (rad/s).
- **`geometry`** (`colav_interfaces/DynamicObstacleGeometry`): The geometric shape representing the obstacle.

---

## `StaticObstacle.msg`

Represents a static obstacle in a 2D environment.

### Message Definition


### Field Descriptions

- **`tag`** (`string`): Unique identifier for the obstacle.
- **`type`** (`string`): Type or classification of the obstacle.
- **`pose`** (`geometry_msgs/Pose`): The fixed position and orientation of the obstacle.
- **`velocity`** (`float64`): Always `0.0` for static obstacles.
- **`acceleration`** (`float64`): Always `0.0` for static obstacles.
- **`yaw_rate`** (`float64`): Always `0.0` for static obstacles.
- **`geometry`** (`colav_interfaces/DynamicObstacleGeometry`): The geometric shape representing the obstacle.

---

## Example Usage

```yaml
header:
  seq: 1
  stamp: { secs: 1700000000, nsecs: 123456789 }
  frame_id: "map"
mission_tag: "Mission_001"
static_obstacles:
  - tag: "Obstacle_1"
    type: "rock"
    pose:
      position: { x: 50.0, y: 30.0, z: 0.0 }
      orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    velocity: 0.0
    acceleration: 0.0
    yaw_rate: 0.0
    geometry:
      shape: "polygon"
      vertices: [{ x: 49.5, y: 29.5 }, { x: 50.5, y: 29.5 }, { x: 50.5, y: 30.5 }, { x: 49.5, y: 30.5 }]
dynamic_obstacles:
  - tag: "Obstacle_2"
    type: "vessel"
    pose:
      position: { x: 100.0, y: 75.0, z: 0.0 }
      orientation: { x: 0.0, y: 0.0, z: 0.707, w: 0.707 }
    velocity: 3.5
    acceleration: 0.2
    yaw_rate: 0.1
    geometry:
      shape: "circle"
      radius: 2.0
