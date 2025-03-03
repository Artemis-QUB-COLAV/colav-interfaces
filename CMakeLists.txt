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
  "msg/mission/vessel/VesselConstraints.msg"
  "msg/mission/vessel/VesselGeometry.msg"
  "msg/mission/vessel/Vessel.msg"
  "msg/mission/MissionRequest.msg"
  "msg/controller_feedback/CmdVelYaw.msg"
  "msg/controller_feedback/ControlMode.msg"
  "msg/controller_feedback/ControlStatus.msg"
  "msg/controller_feedback/ControllerFeedback.msg"
  "msg/hybrid_automaton/agent_update/AgentConfig.msg"
  "msg/hybrid_automaton/obstacles_update/DynamicObstacleConfig.msg"
  "msg/hybrid_automaton/obstacles_update/StaticObstacleConfig.msg"
  "msg/hybrid_automaton/obstacles_update/ObstaclesConfig.msg"
  "msg/unsafe_set/UnsafeSet.msg"
  "action/MissionExecutor.action"
  "srv/hybrid_automaton/StartHybridAutomaton.srv"
  DEPENDENCIES geometry_msgs
  DEPENDENCIES nav_msgs
  DEPENDENCIES std_msgs
  DEPENDENCIES builtin_interfaces
)

ament_package()
