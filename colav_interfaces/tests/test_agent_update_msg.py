import rclpy
from colav_interfaces.msg import AgentUpdate
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
import pytest

@pytest.fixture(scope='class')
def rclpy_node():
    rclpy.init()
    node = rclpy.create_node('test_agent_update')
    yield node
    node.destroy_node()
    rclpy.shutdown()

@pytest.fixture(scope='class')
def setup_pub_sub(rclpy_node):
    received_message = {'msg': None}

    def callback(msg):
        received_message['msg'] = msg

    publisher = rclpy_node.create_publisher(AgentUpdate, 'agent_update', 10)
    subscription = rclpy_node.create_subscription(AgentUpdate, 'agent_update', callback, 10)
    
    yield publisher, received_message

    subscription.destroy()

class TestControllerFeedback:
    def test_msg_struct(self):
        """Simple structure test of controller feedback message"""
        msg = AgentUpdate()
        assert hasattr(msg, 'header')
        assert isinstance(msg.header, Header)
        assert hasattr(msg, 'mission_tag')
        assert isinstance(msg.mission_tag, str)
        assert hasattr(msg, 'agent_tag')
        assert isinstance(msg.agent_tag, str)
        assert hasattr(msg, 'pose')
        assert isinstance(msg.pose, Pose)
        assert hasattr(msg, 'velocity')
        assert isinstance(msg.velocity, float)
        assert hasattr(msg, 'acceleration')
        assert isinstance(msg.acceleration, float)
        assert hasattr(msg, 'yaw_rate')
        assert isinstance(msg.yaw_rate, float)
        
        print('Message structure test passed!')

    def test_pub_sub(self, rclpy_node, setup_pub_sub):
        """Test publisher and subscriber"""
        publisher, received_message = setup_pub_sub

        agent_update = AgentUpdate()
        agent_update.mission_tag = "test_agent"
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

        publisher.publish(agent_update)
        rclpy.spin_once(rclpy_node, timeout_sec=0.1)

        assert received_message['msg'] is not None
        assert received_message['msg'].mission_tag == 'test_agent'
        assert received_message['msg'].header.stamp.sec == 1000
        assert received_message['msg'].header.stamp.nanosec == 20030
        assert received_message['msg'].header.frame_id == 'map'
        assert received_message['msg'].pose.position.x == 10.0
        assert received_message['msg'].pose.position.y == 20.0
        assert received_message['msg'].pose.position.z == 0.0
        assert received_message['msg'].pose.orientation.x == 0.0
        assert received_message['msg'].pose.orientation.y == 0.0
        assert received_message['msg'].pose.orientation.z == 0.0
        assert received_message['msg'].pose.orientation.w == 1.0
        assert received_message['msg'].velocity == 2.0
        assert received_message['msg'].acceleration == 1.0
        assert received_message['msg'].yaw_rate == 0.2
        
        print('Publisher and subscriber test passed!')