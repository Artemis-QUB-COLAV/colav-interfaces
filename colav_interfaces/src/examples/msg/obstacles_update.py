from colav_interfaces.msg import ObstaclesUpdate
from colav_interfaces.msg import StaticObstacle
from colav_interfaces.msg import DynamicObstacle
from typing import List

def obstacles_update_example() -> ObstaclesUpdate:
    """obstacles update example"""
    obstacles_update = ObstaclesUpdate()
    
    obstacles_update.mission_tag = "mission_0001"
    
    obstacles_update.header.stamp.sec = 1000
    obstacles_update.header.stamp.nanosec = 20030
    obstacles_update.header.frame_id = "map"
    
    obstacles_update.static_obstacles = static_obstacles_example()
    obstacles_update.dynamic_obstacles = dynamic_obstacles_example()
    
    return obstacles_update


def static_obstacles_example() -> List[StaticObstacle]:
    """example static obstacles"""
    static_obstacles = []
    
    for x in range(0,3):
        static_obstacle = StaticObstacle()
        static_obstacle.tag = "static_obstacle_000" + str(x)
        static_obstacle.type = "buoy"
        static_obstacle.pose.position.x = 10.0
        static_obstacle.pose.position.y = 20.0
        static_obstacle.pose.position.z = 0.0
        static_obstacle.pose.orientation.x = 0.0
        static_obstacle.pose.orientation.y = 0.0
        static_obstacle.pose.orientation.z = 0.0
        static_obstacle.pose.orientation.w = 1.0
        static_obstacle.geometry.loa = 1.0
        static_obstacle.geometry.beam = 2.0
        static_obstacle.geometry.safety_radius = 1.5
        
        static_obstacles.append(static_obstacle)
        
    return static_obstacles

def dynamic_obstacles_example() -> List[DynamicObstacle]:
    """example dynamic obstacles"""
    dynamic_obstacles = []
    
    for x in range(0,2):
        dynamic_obstacle = DynamicObstacle()
        dynamic_obstacle.tag = f"dynamic_obstacle_{x}"
        dynamic_obstacle.type = "vessel"
        
        dynamic_obstacle.pose.position.x = 10.0
        dynamic_obstacle.pose.position.y = 20.0
        dynamic_obstacle.pose.position.z = 0.0
        dynamic_obstacle.pose.orientation.x = 0.0
        dynamic_obstacle.pose.orientation.y = 0.0
        dynamic_obstacle.pose.orientation.z = 0.0
        dynamic_obstacle.pose.orientation.w = 1.0
        
        dynamic_obstacle.velocity = 2.0
        dynamic_obstacle.acceleration = 1.0
        dynamic_obstacle.yaw_rate = 0.2
        
        dynamic_obstacle.geometry.loa = 10.0
        dynamic_obstacle.geometry.beam = 2.0
        dynamic_obstacle.geometry.safety_radius = 1.5 
        
        dynamic_obstacles.append(dynamic_obstacle)
    
    return dynamic_obstacles

def main():
    obstacles_update = obstacles_update_example()
    print (obstacles_update)

if __name__ == "__main__":
    main()