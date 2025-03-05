# COLAV Interfaces

ColavInterfaces is a repository that provides custom interface definitions for topics, services, and action goals utilized in the COLAV communication framework. These interfaces facilitate efficient communication and data exchange among various components of the COLAV system.

## Message Definitions

- **[msg](https://github.com/Artemis-QUB-COLAV/colav-interfaces/tree/main/colav_interfaces/msg)**
    - **[ControllerFeedback.msg](https://github.com/Artemis-QUB-COLAV/colav-interfaces/blob/main/colav_interfaces/msg/controller_data/ControllerFeedback.msg)**: Contains feedback data from the controller.
    - **[MissionRequest.msg](https://github.com/Artemis-QUB-COLAV/colav-interfaces/blob/main/colav_interfaces/msg/mission_config/MissionRequest.msg)**: Used to request the initiation of a mission.
    - **[AgentUpdate.msg](https://github.com/Artemis-QUB-COLAV/colav-interfaces/blob/main/colav_interfaces/msg/state_updates/agent_update/AgentUpdate.msg)**: Provides updates on the state of an agent.
    - **[ObstaclesUpdate.msg](https://github.com/Artemis-QUB-COLAV/colav-interfaces/blob/main/colav_interfaces/msg/state_updates/obstacles_update/ObstaclesUpdate.msg)**: Contains updates on detected obstacles.
    - **[UnsafeSet.msg](https://github.com/Artemis-QUB-COLAV/colav-interfaces/blob/main/colav_interfaces/msg/state_updates/unsafe_set_update/UnsafeSet.msg)**: Provides updates on unsafe sets within the environment.

## Service Definitions

- **[srv](https://github.com/Artemis-QUB-COLAV/colav-interfaces/tree/main/colav_interfaces/srv)**
    - **[StartHybridAutomaton.srv](https://github.com/Artemis-QUB-COLAV/colav-interfaces/blob/main/colav_interfaces/srv/hybrid_automaton/StartHybridAutomaton.srv)**: Service to start the hybrid automaton.

## Action Definitions

- **[action](https://github.com/Artemis-QUB-COLAV/colav-interfaces/tree/main/colav_interfaces/action)**
    - **[MissionExecutor.action](https://github.com/Artemis-QUB-COLAV/colav-interfaces/tree/main/colav_interfaces/action/MissionExecutor.action)**: Defines the action for executing a mission.

## Table of Contents

- [Installation](#installation)
- [Structure](#structure)
- [Usage](#usage)
- [License](#license)

## Installation

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone git@github.com:Artemis-QUB-COLAV/colav-interfaces.git
cd ~/ros2_ws && colcon build && source install/setup.bash

ros2 pkg list | grep colav_interfaces # should now show colav_interfaces
```

## Structure

- `.devcontainer`: Contains `devcontainer.json` for creating development environments for working on this package without needing to worry about environment setup. This works with VSCode and the DevContainer VSCode extension. With both installed, assuming you are at the root of the project, you can open the command palette and enter "build container" to build the container for this project.
- `.github/workflows`: Contains `CI/CD workflow.yml` for GitHub Actions which keeps the colav-interfaces in check.
- `colav_interfaces`: Contains the package with colav_interface tests and interface definitions.
- `docs`: Contains additional documentation on the package.

## Usage

Sample definition for mission request from colav_interfaces. More examples are contained in:

```python
from colav_interfaces.msg import MissionRequest

mission_request = MissionRequest()
mission_request.tag = "mission_0001"
mission_request.header.stamp.sec = 1000
mission_request.header.stamp.nanosec = 20030
mission_request.header.frame_id = "map"
mission_request.vessel.tag = "vessel_0001"
mission_request.vessel.type = "hydrofoil"
mission_request.vessel.constraints.max_acceleration = 2.0
mission_request.vessel.constraints.max_deceleration = 1.0
mission_request.vessel.constraints.max_velocity = 30.0
mission_request.vessel.constraints.min_velocity = 10.0
mission_request.vessel.constraints.max_yaw_rate = 0.2
mission_request.vessel.geometry.loa = 10.0
mission_request.vessel.geometry.beam = 2.0
mission_request.vessel.geometry.safety_radius = 1.5
mission_request.goal_waypoint.position.x = 10.0
mission_request.goal_waypoint.position.y = 20.0
mission_request.goal_waypoint.position.z = 0.0
mission_request.goal_waypoint.acceptance_radius = 1.0
```

## License

`colav-interfaces` is distributed under the terms of the [MIT](https://github.com/Artemis-QUB-COLAV/colav-interfaces/blob/main/LICENSE) license.