from colav_interfaces.msg import AgentUpdate

def agent_update_example() -> AgentUpdate:
    """agent update example"""
    agent_update = AgentUpdate()
    agent_update.mission_tag = "agent_0001"
    agent_update.header.stamp.sec = 1000
    agent_update.header.stamp.nanosec = 20030
    agent_update.header.frame_id = "map"
    agent_update.pose.position.x = 10.0
    agent_update.pose.position.y = 20.0
    agent_update.pose.position.z = 0.0
    agent_update.pose.orientation.x = 0.0
    agent_update.pose.orientation.y = 0.0
    agent_update.pose.orientation.z = 0.0
    agent_update.pose.orientation.w = 1.0
    agent_update.velocity = 2.0
    agent_update.acceleration = 1.0
    agent_update.yaw_rate = 0.2
    
    return agent_update

def main():
    """main function"""
    agent_update = agent_update_example()
    print (agent_update)

if __name__ == '__main__':
    """run main"""
    main()