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
  "msg/agent_state/AgentState.msg"

  "msg/colregs/COLREGClassification.msg"
  "msg/colregs/CRIClassification.msg"

  "msg/obstacles_state/dynamic_obstacle/DynamicObstacleGeometry.msg"
  "msg/obstacles_state/dynamic_obstacle/DynamicObstacleState.msg"
  "msg/obstacles_state/dynamic_obstacle/DynamicObstacleRiskState.msg"

  "msg/obstacles_state/static_obstacle/StaticObstacleGeometry.msg"
  "msg/obstacles_state/static_obstacle/StaticObstacleState.msg"
  "msg/obstacles_state/static_obstacle/StaticObstacleRiskState.msg"

  "msg/obstacles_state/ObstaclesState.msg"
  "msg/obstacles_state/ObstacleRiskStates.msg"

  "msg/unsafe_set_state/UnsafeSetState.msg"

  "msg/waypoints_state/Waypoint.msg"
  "msg/waypoints_state/WaypointsState.msg"
  
  "action/StartScenarioTest.action"
  
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

