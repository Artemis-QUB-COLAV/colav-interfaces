import rclpy
from colav_interfaces.msg import UnsafeSet
from std_msgs.msg import Header, Float64MultiArray
from geometry_msgs.msg import Pose
import numpy as np
import pytest

@pytest.fixture(scope='class')
def rclpy_node():
    rclpy.init()
    node = rclpy.create_node('test_unsafe_set')
    yield node
    node.destroy_node()
    rclpy.shutdown()

@pytest.fixture(scope='class')
def setup_pub_sub(rclpy_node):
    received_message = {'msg': None}

    def callback(msg):
        received_message['msg'] = msg

    publisher = rclpy_node.create_publisher(UnsafeSet, 'unsafe_set', 10)
    subscription = rclpy_node.create_subscription(UnsafeSet, 'unsafe_set', callback, 10)
    
    yield publisher, received_message

    subscription.destroy()

class TestControllerFeedback:
    def test_msg_struct(self):
        """Simple structure test of controller feedback message"""
        msg = UnsafeSet()
        assert hasattr(msg, 'header')
        assert isinstance(msg.header, Header)
        assert hasattr(msg, 'mission_tag')
        assert isinstance(msg.mission_tag, str)
        assert hasattr(msg, 'vertices')
        assert isinstance(msg.vertices, Float64MultiArray)
        
        print('Message structure test passed!')

    def test_pub_sub(self, rclpy_node, setup_pub_sub):
        """Test publisher and subscriber"""
        publisher, received_message = setup_pub_sub

        agent_update = UnsafeSet()
        agent_update.mission_tag = "test_agent"
        agent_update.header.stamp.sec = 1000
        agent_update.header.stamp.nanosec = 20030
        agent_update.header.frame_id = "map"
        

        publisher.publish(agent_update)
        rclpy.spin_once(rclpy_node, timeout_sec=0.1)

        assert received_message['msg'] is not None
        assert received_message['msg'].mission_tag == 'test_agent'
        assert received_message['msg'].header.stamp.sec == 1000
        assert received_message['msg'].header.stamp.nanosec == 20030
        assert received_message['msg'].header.frame_id == 'map'
        
        print('Publisher and subscriber test passed!')

    def test_vertices_data(self, rclpy_node, setup_pub_sub):
        """Test vertices data in the message"""
        publisher, received_message = setup_pub_sub

        agent_update = UnsafeSet()
        agent_update.mission_tag = "test_agent"
        agent_update.header.stamp.sec = 1000
        agent_update.header.stamp.nanosec = 20030
        agent_update.header.frame_id = "map"
        agent_update.vertices.data = [1.0, 2.0, 3.0]

        publisher.publish(agent_update)
        rclpy.spin_once(rclpy_node, timeout_sec=0.1)

        assert received_message['msg'] is not None
        assert np.array_equal(received_message['msg'].vertices.data, np.array([1.0, 2.0, 3.0]))
        
        print('Vertices data test passed!')