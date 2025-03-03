*# AgentUpdate Message

The `AgentUpdate.msg` message is designed to provide real-time updates on the state of an agent within a mission. This message includes critical information such as the agent's pose, velocity, acceleration, yaw rate, and safety radius. It is primarily used in real-time mission planning and collision avoidance systems to monitor and track the state of agents, ensuring that planners or monitoring systems have the most up-to-date information.

This message is essential for applications that require precise and timely updates on agent states, such as autonomous navigation, multi-agent coordination, and dynamic obstacle avoidance.

## Message Definition
## Field Descriptions
The fields for the agent update are as follows:

- **`header`** (`std_msgs/Header`): Standard ROS message header containing a timestamp and frame information.
- **`mission_tag`** (`string`): Unique identifier for the mission associated with this agent update.
- **`agent_tag`** (`string`): Unique identifier for the agent.
- **`pose`** (`geometry_msgs/Pose`): The agent's current position and orientation in the world frame.
- **`velocity`** (`float64`): The agent's current linear velocity (m/s).
- **`acceleration`** (`float64`): The agent's current linear acceleration (m/s²).
- **`yaw_rate`** (`float64`): The agent's angular velocity around the yaw axis (rad/s).

## Example Formatting
In yaml format the AgentUpdate update is showed.

```yaml
header:
  seq: 42
  stamp: { secs: 1700000000, nsecs: 123456789 }
  frame_id: "map"
mission_tag: "Mission_001"
agent_tag: "Agent_A"
pose:
  position: { x: 10.0, y: 5.0, z: 0.0 }
  orientation: { x: 0.0, y: 0.0, z: 0.707, w: 0.707 }
velocity: 2.5
acceleration: 0.1
yaw_rate: 0.05
```

## Usage Example
Assuming the colav_interface has been colcon built and sourced you can programmatically import and use it.

```python
    from colav_interfaces.msg import AgentUpdate
    from geometry_msgs.msg import Pose

    agent_update = AgentUpdate()
    agent_update.header.stamp = self.get_clock().now().to_msg()
    agent_update.header.frame_id = "map"  # sample frame_id

    agent_update.mission_tag = "mission_001"
    agent_update.agent_tag = "agent_007"

    agent_update.port = Pose()
    agent_update.pose.position.x = 10.0
    agent_update.pose.position.y = 5.0
    agent_update.pose.position.z = 0.0
    agent_update.pose.orientation.x = 0.0
    agent_update.pose.orientation.y = 0.0
    agent_update.pose.orientation.z = 0.0
    agent_update.pose.orientation.w = 1.0

    agent_update.velocity = 5.0
    agent_update.acceleration = 2.0
    agent_update.yaw_rate = 0.1
```

*