import rclpy
from colav_interfaces.msg import ControllerFeedback, CmdVelYaw, ControlMode, ControlStatus
from std_msgs.msg import Header
import pytest

@pytest.fixture(scope='class')
def rclpy_node():
    rclpy.init()
    node = rclpy.create_node('test_controller_feedback')
    yield node
    node.destroy_node()
    rclpy.shutdown()

@pytest.fixture(scope='class')
def setup_pub_sub(rclpy_node):
    received_message = {'msg': None}

    def callback(msg):
        received_message['msg'] = msg

    publisher = rclpy_node.create_publisher(ControllerFeedback, 'controller_feedback', 10)
    subscription = rclpy_node.create_subscription(ControllerFeedback, 'controller_feedback', callback, 10)
    
    yield publisher, received_message

    subscription.destroy()

class TestControllerFeedback:
    def test_msg_struct(self):
        """Simple structure test of controller feedback message"""
        msg = ControllerFeedback()
        assert hasattr(msg, 'header')
        assert isinstance(msg.header, Header)
        assert hasattr(msg, 'mission_tag')
        assert isinstance(msg.mission_tag, str)
        assert hasattr(msg, 'agent_tag')
        assert isinstance(msg.agent_tag, str)
        assert hasattr(msg, 'cmd')
        assert isinstance(msg.cmd, CmdVelYaw)
        assert hasattr(msg.cmd, 'velocity')
        assert isinstance(msg.cmd.velocity, float)
        assert hasattr(msg.cmd, 'yaw_rate')
        assert isinstance(msg.cmd.yaw_rate, float)
        assert hasattr(msg, 'mode')
        assert isinstance(msg.mode, ControlMode)
        assert hasattr(msg, 'status')
        assert isinstance(msg.status, ControlStatus)
        assert hasattr(msg.status, 'type')
        assert isinstance(msg.status.type, int)
        assert hasattr(msg.status, 'message')
        assert isinstance(msg.status.message, str)
        
        print('Message structure test passed!')

    def test_pub_sub(self, rclpy_node, setup_pub_sub):
        """Test publisher and subscriber"""
        publisher, received_message = setup_pub_sub

        msg = ControllerFeedback()
        msg.header.stamp.sec = 1
        msg.header.stamp.nanosec = 1
        msg.header.frame_id = 'test_frame'
        msg.mission_tag = 'test_mission'
        msg.agent_tag = 'test_agent'
        msg.cmd.velocity = 1.0
        msg.cmd.yaw_rate = 1.0
        msg.mode.type = 1
        msg.status.type = 1
        msg.status.message = 'mission on-going'

        publisher.publish(msg)
        rclpy.spin_once(rclpy_node, timeout_sec=0.1)

        assert received_message['msg'] is not None
        assert received_message['msg'].header.stamp.sec == 1
        assert received_message['msg'].header.stamp.nanosec == 1
        assert received_message['msg'].header.frame_id == 'test_frame'
        assert received_message['msg'].mission_tag == 'test_mission'
        assert received_message['msg'].agent_tag == 'test_agent'
        assert received_message['msg'].cmd.velocity == 1.0
        assert received_message['msg'].cmd.yaw_rate == 1.0
        assert received_message['msg'].mode.type == 1
        assert received_message['msg'].status.type == 1
        assert received_message['msg'].status.message == 'mission on-going'
        
        print('Publisher and subscriber test passed!')