from colav_interfaces.msg import MissionRequest

def mission_request_example() -> MissionRequest:
    """mission request example"""
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
    mission_request.goal_waypoint.position.z - 0.0
    mission_request.goal_waypoint.acceptance_radius = 1.0
    
    return mission_request
    
def main():
    """main function"""
    mission_request = mission_request_example()
    print (mission_request)

if __name__ == '__main__':
    """run main"""
    main()