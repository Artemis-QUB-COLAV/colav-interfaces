cmake_minimum_required(VERSION 3.8)
project(colav_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# ADDED dependencies ------------------------------
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ADDED dependencies END --------------------------

rosidl_generate_interfaces(${PROJECT_NAME}
  "src/msg/mission_config/vessel/VesselConstraints.msg"
  "src/msg/mission_config/vessel/VesselGeometry.msg"
  "src/msg/mission_config/vessel/Vessel.msg"
  "src/msg/mission_config/waypoint/Waypoint.msg"
  "src/msg/mission_config/MissionRequest.msg"

  "src/msg/controller_data/data/CmdVelYaw.msg"
  "src/msg/controller_data/data/ControlMode.msg"
  "src/msg/controller_data/data/ControlStatus.msg"
  "src/msg/controller_data/ControllerFeedback.msg"

  "src/msg/state_updates/agent_update/AgentUpdate.msg"
  "src/msg/state_updates/obstacles_update/geometry/DynamicObstacleGeometry.msg"
  "src/msg/state_updates/obstacles_update/geometry/StaticObstacleGeometry.msg"
  "src/msg/state_updates/obstacles_update/DynamicObstacle.msg"
  "src/msg/state_updates/obstacles_update/StaticObstacle.msg"
  "src/msg/state_updates/obstacles_update/ObstaclesUpdate.msg"

  "src/msg/state_updates/unsafe_set_update/UnsafeSet.msg"

  "src/srv/hybrid_automaton/StartHybridAutomaton.srv"

  DEPENDENCIES geometry_msgs
  DEPENDENCIES nav_msgs
  DEPENDENCIES std_msgs
  DEPENDENCIES builtin_interfaces
)

ament_package()
