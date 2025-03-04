import rclpy
from colav_interfaces.msg import MissionRequest, Vessel, Waypoint, VesselConstraints, VesselGeometry
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
import pytest

@pytest.fixture(scope='class')
def rclpy_node():
    rclpy.init()
    node = rclpy.create_node('test_mission_request')
    yield node
    node.destroy_node()
    rclpy.shutdown()

@pytest.fixture(scope='class')
def setup_pub_sub(rclpy_node):
    received_message = {'msg': None}

    def callback(msg):
        received_message['msg'] = msg

    publisher = rclpy_node.create_publisher(MissionRequest, 'mission_request', 10)
    subscription = rclpy_node.create_subscription(MissionRequest, 'mission_request', callback, 10)
    
    yield publisher, received_message

    subscription.destroy()

class TestMissionRequest:
    def test_msg_struct(self):
        """Simple structure test of controller feedback message"""
        msg = MissionRequest()
        assert hasattr(msg, 'header')
        assert isinstance(msg.header, Header)
        assert hasattr(msg, 'tag')
        assert isinstance(msg.tag, str)
        assert hasattr(msg, 'vessel')
        assert isinstance(msg.vessel, Vessel)
        
        assert hasattr(msg.vessel, 'tag')
        assert isinstance(msg.vessel.tag, str)
        assert hasattr(msg.vessel, 'type')
        assert isinstance(msg.vessel.type, str)
        assert hasattr(msg.vessel, 'constraints')
        assert isinstance(msg.vessel.constraints, VesselConstraints)
        
        assert hasattr(msg.vessel.constraints, 'max_acceleration')
        assert isinstance(msg.vessel.constraints.max_acceleration, float)
        assert hasattr(msg.vessel.constraints, 'max_deceleration')
        assert isinstance(msg.vessel.constraints.max_deceleration, float)
        assert hasattr(msg.vessel.constraints, 'max_velocity')
        assert isinstance(msg.vessel.constraints.max_velocity, float)
        assert hasattr(msg.vessel.constraints, 'min_velocity')
        assert isinstance(msg.vessel.constraints.min_velocity, float)
        assert hasattr(msg.vessel.constraints, 'max_yaw_rate')
        assert isinstance(msg.vessel.constraints.max_yaw_rate, float)
        
        assert hasattr(msg.vessel, 'geometry')
        assert isinstance(msg.vessel.geometry, VesselGeometry)
        
        assert hasattr(msg.vessel.geometry, 'loa')
        assert isinstance(msg.vessel.geometry.loa, float)
        assert hasattr(msg.vessel.geometry, 'beam')
        assert isinstance(msg.vessel.geometry.beam, float)
        assert hasattr(msg.vessel.geometry, 'safety_radius')
        assert isinstance(msg.vessel.geometry.safety_radius, float)
        
        assert hasattr(msg, 'init_position')
        assert isinstance(msg.init_position, Point32)
        assert hasattr(msg, 'goal_waypoint')
        assert isinstance(msg.goal_waypoint, Waypoint)
        assert hasattr(msg.goal_waypoint, 'position')
        assert isinstance(msg.goal_waypoint.position, Point32)
        assert hasattr(msg.goal_waypoint, 'acceptance_radius')
        assert isinstance(msg.goal_waypoint.acceptance_radius, float)
        
        print('Message structure test passed!')

    def test_pub_sub(self, rclpy_node, setup_pub_sub):
        """Test publisher and subscriber"""
        publisher, received_message = setup_pub_sub

        msg = MissionRequest()
        msg.header.stamp.sec = 1
        msg.header.stamp.nanosec = 1
        msg.header.frame_id = 'test_frame'
        msg.tag = "test_mission"
        msg.vessel.tag = "test_vessel"
        msg.vessel.type = "test_type"
        msg.vessel.constraints.max_acceleration = 1.0
        msg.vessel.constraints.max_deceleration = 1.0
        msg.vessel.constraints.max_velocity = 1.0
        msg.vessel.constraints.min_velocity = 1.0
        msg.vessel.constraints.max_yaw_rate = 1.0
        msg.vessel.geometry.loa = 1.0
        msg.vessel.geometry.beam = 1.0
        msg.vessel.geometry.safety_radius = 1.0
        msg.init_position.x = 1.0
        msg.init_position.y = 1.0
        msg.init_position.z = 1.0
        msg.goal_waypoint.position.x = 1.0
        msg.goal_waypoint.position.y = 1.0
        msg.goal_waypoint.position.z = 1.0
        msg.goal_waypoint.acceptance_radius = 1.0
        
        publisher.publish(msg)
        rclpy.spin_once(rclpy_node, timeout_sec=0.1)

        assert received_message['msg'] is not None
        assert received_message['msg'].header.stamp.sec == 1
        assert received_message['msg'].header.stamp.nanosec == 1
        assert received_message['msg'].header.frame_id == 'test_frame'
        assert received_message['msg'].tag == 'test_mission'
        assert received_message['msg'].vessel.tag == 'test_vessel'
        assert received_message['msg'].vessel.type == 'test_type'
        assert received_message['msg'].vessel.constraints.max_acceleration == 1.0
        assert received_message['msg'].vessel.constraints.max_deceleration == 1.0
        assert received_message['msg'].vessel.constraints.max_velocity == 1.0
        assert received_message['msg'].vessel.constraints.min_velocity == 1.0
        assert received_message['msg'].vessel.constraints.max_yaw_rate == 1.0
        assert received_message['msg'].vessel.geometry.loa == 1.0
        assert received_message['msg'].vessel.geometry.beam == 1.0
        assert received_message['msg'].vessel.geometry.safety_radius == 1.0
        assert received_message['msg'].init_position.x == 1.0
        assert received_message['msg'].init_position.y == 1.0
        assert received_message['msg'].init_position.z == 1.0
        assert received_message['msg'].goal_waypoint.position.x == 1.0
        assert received_message['msg'].goal_waypoint.position.y == 1.0
        assert received_message['msg'].goal_waypoint.position.z == 1.0
        assert received_message['msg'].goal_waypoint.acceptance_radius == 1.0
        
        print('Publisher and subscriber test passed!')