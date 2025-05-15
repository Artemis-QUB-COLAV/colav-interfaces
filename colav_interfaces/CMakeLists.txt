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
  "msg/mission_config/vessel/VesselConstraints.msg"
  "msg/mission_config/vessel/VesselGeometry.msg"
  "msg/mission_config/vessel/Vessel.msg"
  "msg/mission_config/waypoint/Waypoint.msg"
  "msg/mission_config/waypoint/Waypoints.msg"
  "msg/mission_config/MissionRequest.msg"
  "msg/controller_data/data/CmdVelYaw.msg"
  "msg/controller_data/data/ControlMode.msg"
  "msg/controller_data/data/ControlStatus.msg"
  "msg/controller_data/ControllerFeedback.msg"
  "msg/state_updates/agent_update/AgentUpdate.msg"
  "msg/state_updates/obstacles_update/geometry/DynamicObstacleGeometry.msg"
  "msg/state_updates/obstacles_update/geometry/StaticObstacleGeometry.msg"
  "msg/state_updates/obstacles_update/DynamicObstacle.msg"
  "msg/state_updates/obstacles_update/StaticObstacle.msg"
  "msg/state_updates/obstacles_update/ObstaclesUpdate.msg"
  "msg/state_updates/unsafe_set_update/UnsafeSet.msg"
  "msg/params/address/Address.msg"
  "msg/params/Params.msg"
  "msg/hybrid_automaton/dynamics/Dynamics.msg"
  "msg/hybrid_automaton/resets/CruiseToT2losReset.msg"
  "msg/hybrid_automaton/resets/WaypointReachedToCruiseReset.msg" 
  "msg/hybrid_automaton/DynamicsUpdate.msg"
  "msg/hybrid_automaton/GuardsStatus.msg"
  "msg/hybrid_automaton/ResetAck.msg"
  "msg/hybrid_automaton/TransitionPending.msg"
  "msg/state_updates/collision_metrics/DynamicObstacleCollisionMetric.msg"
  "msg/state_updates/collision_metrics/StaticObstacleCollisionMetric.msg"
  "msg/state_updates/collision_metrics/CollisionMetrics.msg"
  "srv/hybrid_automaton/Reset.srv"
  "srv/hybrid_automaton/StartHybridAutomaton.srv"
  "action/MissionExecutor.action"
  
  DEPENDENCIES geometry_msgs
  DEPENDENCIES nav_msgs
  DEPENDENCIES std_msgs
  DEPENDENCIES builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(your_test_name tests)
endif()

ament_package()

