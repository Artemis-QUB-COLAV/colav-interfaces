# import rclpy
# from colav_interfaces.msg import Params, Address
# from std_msgs.msg import Header
# from geometry_msgs.msg import Pose
# import pytest

# @pytest.fixture(scope='class')
# def rclpy_node():
#     rclpy.init()
#     node = rclpy.create_node('test_agent_update')
#     yield node
#     node.destroy_node()
#     rclpy.shutdown()

# @pytest.fixture(scope='class')
# def setup_pub_sub(rclpy_node):
#     received_message = {'msg': None}

#     def callback(msg):
#         received_message['msg'] = msg

#     publisher = rclpy_node.create_publisher(Params, 'params_update', 10)
#     subscription = rclpy_node.create_subscription(Params, 'params_update', callback, 10)
    
#     yield publisher, received_message

#     subscription.destroy()

# class TestParams:
#     def test_msg_struct(self):
#         """Simple structure test of controller feedback message"""
#         msg = Params()
#         assert hasattr(msg, 'mission_request_address')
#         assert isinstance(msg.mission_request_address, Address)
#         assert hasattr(msg, 'mission_response_address')
#         assert isinstance(msg.mission_response_address, Address)
#         assert hasattr(msg, 'agent_update_address')
#         assert isinstance(msg.agent_update_address, Address)
#         assert hasattr(msg, 'agent_update_sync_hz')
#         assert isinstance(msg.agent_update_sync_hz, float)
#         assert hasattr(msg, 'agent_update_sync_tolerance_secs')
#         assert isinstance(msg.agent_update_sync_tolerance_secs, float)
#         assert hasattr(msg, 'obstacles_update_address')
#         assert isinstance(msg.obstacles_update_address, Address)
#         assert hasattr(msg, 'obstacles_update_sync_hz')
#         assert isinstance(msg.obstacles_update_sync_hz, float)
#         assert hasattr(msg, 'obstacles_update_sync_tolerance_secs')
#         assert isinstance(msg.obstacles_update_sync_tolerance_secs, float)
#         assert hasattr(msg, 'controller_feedback_address')
#         assert isinstance(msg.controller_feedback_address, Address)
#         assert hasattr(msg, 'controller_feedback_sync_hz')
#         assert isinstance(msg.controller_feedback_sync_hz, float)
        
        
#         print('Message structure test passed!')

#     def test_pub_sub(self, rclpy_node, setup_pub_sub):
#         """Test publisher and subscriber"""
#         publisher, received_message = setup_pub_sub

#         params_update = Params()
#         params_update.mission_request_address = Address(ip="0.0.0.0", port=7000)
#         params_update.mission_response_address = Address(ip="192.168.1.4", port=7001)
#         params_update.agent_update_address = Address(ip="0.0.0.0", port=7100)
#         params_update.agent_update_sync_hz= 1.0
#         params_update.agent_update_sync_tolerance_secs= 0.2
#         params_update.obstacles_update_address = Address(ip="0.0.0.0", port=7100)
#         params_update.obstacles_update_sync_hz = 1.0
#         params_update.obstacles_update_sync_tolerance_secs = 0.2
#         params_update.controller_feedback_address = Address(ip="192.168.1.4", port=7300)
#         params_update.controller_feedback_sync_hz = 1.0

#         publisher.publish(params_update)
#         rclpy.spin_once(rclpy_node, timeout_sec=0.1)

#         assert received_message['msg'] is not None
#         assert received_message['msg'].mission_request_address.ip == params_update.mission_request_address.ip
#         assert received_message['msg'].mission_request_address.port == params_update.mission_request_address.port

#         assert received_message['msg'].mission_response_address.ip == params_update.mission_response_address.ip
#         assert received_message['msg'].mission_response_address.port == params_update.mission_response_address.port

#         assert received_message['msg'].agent_update_address.ip == params_update.agent_update_address.ip
#         assert received_message['msg'].agent_update_address.port == params_update.agent_update_address.port
#         assert received_message['msg'].agent_update_sync_hz == params_update.agent_update_sync_hz
#         assert received_message['msg'].agent_update_sync_tolerance_secs == params_update.agent_update_sync_tolerance_secs

#         assert received_message['msg'].obstacles_update_address.ip == params_update.obstacles_update_address.ip
#         assert received_message['msg'].obstacles_update_address.port == params_update.obstacles_update_address.port
#         assert received_message['msg'].obstacles_update_sync_hz == params_update.obstacles_update_sync_hz
#         assert received_message['msg'].obstacles_update_sync_tolerance_secs == params_update.obstacles_update_sync_tolerance_secs

#         assert received_message['msg'].controller_feedback_address.ip == params_update.controller_feedback_address.ip
#         assert received_message['msg'].controller_feedback_address.port == params_update.controller_feedback_address.port

#         assert received_message['msg'].controller_feedback_address.controller_feedback_sync_hz == params_update.controller_feedback_address.controller_feedback_sync_hz
        
#         print('Publisher and subscriber test passed!')

# def main():
#     TestParams.test_msg_struct()

# if __name__ == '__main__':
#     main()