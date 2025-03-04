import rclpy
from colav_interfaces.msg import ObstaclesUpdate, DynamicObstacle, DynamicObstacleGeometry, StaticObstacle, StaticObstacleGeometry
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
import pytest
from typing import List

@pytest.fixture(scope='class')
def rclpy_node():
    rclpy.init()
    node = rclpy.create_node('test_obstacles_update')
    yield node
    node.destroy_node()
    rclpy.shutdown()

@pytest.fixture(scope='class')
def setup_pub_sub(rclpy_node):
    received_message = {'msg': None}

    def callback(msg):
        received_message['msg'] = msg

    publisher = rclpy_node.create_publisher(ObstaclesUpdate, 'obstacles_update', 10)
    subscription = rclpy_node.create_subscription(ObstaclesUpdate, 'obstacles_update', callback, 10)
    
    yield publisher, received_message

    subscription.destroy()

class TestObstaclesUpdate:
    """Test Class for Controller Feedback Message"""
    def test_msg_struct(self):
        """Simple structure test of controller feedback message"""
        msg = ObstaclesUpdate()
        assert hasattr(msg, 'header')
        assert isinstance(msg.header, Header)
        assert hasattr(msg, 'mission_tag')
        assert isinstance(msg.mission_tag, str)
        assert hasattr(msg, 'static_obstacles')
        assert isinstance(msg.static_obstacles, list) # StaticObstacle list
        assert hasattr(msg, 'dynamic_obstacles')    
        assert isinstance(msg.dynamic_obstacles, list) # DynamicObstacle list
        
        # test DynamicObstacle struct
        dynamic_obstacle = DynamicObstacle()
        assert hasattr(dynamic_obstacle, 'tag')
        assert isinstance(dynamic_obstacle.tag, str)
        assert hasattr(dynamic_obstacle, 'type')
        assert isinstance(dynamic_obstacle.type, str)
        assert hasattr(dynamic_obstacle, 'pose')
        assert isinstance(dynamic_obstacle.pose, Pose)
        assert hasattr(dynamic_obstacle, 'velocity')
        assert isinstance(dynamic_obstacle.velocity, float)
        assert hasattr(dynamic_obstacle, 'acceleration')
        assert isinstance(dynamic_obstacle.acceleration, float)
        assert hasattr(dynamic_obstacle, 'yaw_rate')
        assert isinstance(dynamic_obstacle.yaw_rate, float)
        assert hasattr(dynamic_obstacle, 'geometry')
        assert isinstance(dynamic_obstacle.geometry, DynamicObstacleGeometry)
        assert hasattr(dynamic_obstacle.geometry, 'loa')
        assert isinstance(dynamic_obstacle.geometry.loa, float)
        assert hasattr(dynamic_obstacle.geometry, 'beam')
        assert isinstance(dynamic_obstacle.geometry.beam, float)
        assert hasattr(dynamic_obstacle.geometry, 'safety_radius')
        assert isinstance(dynamic_obstacle.geometry.safety_radius, float)
        # test StaticObstacle struct
        static_obstacle = StaticObstacle()
        assert hasattr(static_obstacle, 'tag')
        assert isinstance(static_obstacle.tag, str)
        assert hasattr(static_obstacle, 'type')
        assert isinstance(static_obstacle.type, str)
        assert hasattr(static_obstacle, 'pose')
        assert isinstance(static_obstacle.pose, Pose)
        assert hasattr(static_obstacle, 'geometry')
        assert isinstance(static_obstacle.geometry, StaticObstacleGeometry)
        assert hasattr(static_obstacle.geometry, 'loa')
        assert isinstance(static_obstacle.geometry.loa, float)
        assert hasattr(static_obstacle.geometry, 'beam')
        assert isinstance(static_obstacle.geometry.beam, float)
        assert hasattr(static_obstacle.geometry, 'safety_radius')
        assert isinstance(static_obstacle.geometry.safety_radius, float)
        
        print('Message structure test passed!') 

    def test_pub_sub(self, rclpy_node, setup_pub_sub):
        """Test publisher and subscriber"""
        publisher, received_message = setup_pub_sub

        obstacles_update = ObstaclesUpdate()
        obstacles_update.mission_tag = "test_mission"
        obstacles_update.header.stamp.sec = 1000
        obstacles_update.header.stamp.nanosec = 20030
        obstacles_update.header.frame_id = "map"

        dynamic_obstacle = DynamicObstacle()
        dynamic_obstacle.tag = "dynamic_1"
        dynamic_obstacle.type = "type_1"
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
        dynamic_obstacle.geometry.loa = 5.0
        dynamic_obstacle.geometry.beam = 2.0
        dynamic_obstacle.geometry.safety_radius = 1.0

        static_obstacle = StaticObstacle()
        static_obstacle.tag = "static_1"
        static_obstacle.type = "type_2"
        static_obstacle.pose.position.x = 15.0
        static_obstacle.pose.position.y = 25.0
        static_obstacle.pose.position.z = 0.0
        static_obstacle.pose.orientation.x = 0.0
        static_obstacle.pose.orientation.y = 0.0
        static_obstacle.pose.orientation.z = 0.0
        static_obstacle.pose.orientation.w = 1.0
        static_obstacle.geometry.loa = 6.0
        static_obstacle.geometry.beam = 3.0
        static_obstacle.geometry.safety_radius = 1.5

        obstacles_update.dynamic_obstacles.append(dynamic_obstacle)
        obstacles_update.static_obstacles.append(static_obstacle)

        publisher.publish(obstacles_update)
        rclpy.spin_once(rclpy_node, timeout_sec=0.1)

        assert received_message['msg'] is not None
        assert received_message['msg'].mission_tag == 'test_mission'
        assert received_message['msg'].header.stamp.sec == 1000
        assert received_message['msg'].header.stamp.nanosec == 20030
        assert received_message['msg'].header.frame_id == 'map'

        assert len(received_message['msg'].dynamic_obstacles) == 1
        assert received_message['msg'].dynamic_obstacles[0].tag == 'dynamic_1'
        assert received_message['msg'].dynamic_obstacles[0].type == 'type_1'
        assert received_message['msg'].dynamic_obstacles[0].pose.position.x == 10.0
        assert received_message['msg'].dynamic_obstacles[0].pose.position.y == 20.0
        assert received_message['msg'].dynamic_obstacles[0].pose.position.z == 0.0
        assert received_message['msg'].dynamic_obstacles[0].pose.orientation.x == 0.0
        assert received_message['msg'].dynamic_obstacles[0].pose.orientation.y == 0.0
        assert received_message['msg'].dynamic_obstacles[0].pose.orientation.z == 0.0
        assert received_message['msg'].dynamic_obstacles[0].pose.orientation.w == 1.0
        assert received_message['msg'].dynamic_obstacles[0].velocity == 2.0
        assert received_message['msg'].dynamic_obstacles[0].acceleration == 1.0
        assert received_message['msg'].dynamic_obstacles[0].yaw_rate == 0.2
        assert received_message['msg'].dynamic_obstacles[0].geometry.loa == 5.0
        assert received_message['msg'].dynamic_obstacles[0].geometry.beam == 2.0
        assert received_message['msg'].dynamic_obstacles[0].geometry.safety_radius == 1.0

        assert len(received_message['msg'].static_obstacles) == 1
        assert received_message['msg'].static_obstacles[0].tag == 'static_1'
        assert received_message['msg'].static_obstacles[0].type == 'type_2'
        assert received_message['msg'].static_obstacles[0].pose.position.x == 15.0
        assert received_message['msg'].static_obstacles[0].pose.position.y == 25.0
        assert received_message['msg'].static_obstacles[0].pose.position.z == 0.0
        assert received_message['msg'].static_obstacles[0].pose.orientation.x == 0.0
        assert received_message['msg'].static_obstacles[0].pose.orientation.y == 0.0
        assert received_message['msg'].static_obstacles[0].pose.orientation.z == 0.0
        assert received_message['msg'].static_obstacles[0].pose.orientation.w == 1.0
        assert received_message['msg'].static_obstacles[0].geometry.loa == 6.0
        assert received_message['msg'].static_obstacles[0].geometry.beam == 3.0
        assert received_message['msg'].static_obstacles[0].geometry.safety_radius == 1.5

        print('Publisher and subscriber test passed!')