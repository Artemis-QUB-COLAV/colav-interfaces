# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from rclpy.action import ActionServer
# import pytest
# from colav_interfaces.action import MissionExecutor
# from colav_interfaces.msg import MissionRequest, Vessel, Waypoint
# from geometry_msgs.msg import Point32
# from std_msgs.msg import Header, ByteMultiArray
# from rclpy.task import Future

# @pytest.fixture(scope='class')
# def rclpy_node():
#     rclpy.init()
#     node = rclpy.create_node('test_mission_executor')
#     yield node
#     node.destroy_node()
#     rclpy.shutdown()

# @pytest.fixture(scope='class')
# def setup_action_server(rclpy_node):
#     class TestMissionExecutorServer(Node):
#         def __init__(self):
#             super().__init__('test_mission_executor_server')
#             self._action_server = ActionServer(
#                 self,
#                 MissionExecutor,
#                 'mission_executor',
#                 self.execute_callback
#             )

#         def execute_callback(self, goal_handle):
#             self.get_logger().info('Executing goal...')
#             result = MissionExecutor.Result()
#             result.success = True
#             result.status_message = "Mission executed successfully"
#             result.serialised_controller_feedback = ByteMultiArray()
#             goal_handle.succeed()
#             return result

#     server_node = TestMissionExecutorServer()
#     yield server_node
#     server_node.destroy_node()

# class TestMissionExecutor:
#     def test_action_call(self, rclpy_node, setup_action_server):
#         """Test MissionExecutor action client-server communication"""
#         action_client = ActionClient(rclpy_node, MissionExecutor, 'mission_executor')
#         assert action_client.wait_for_server(timeout_sec=5.0)

#         goal_msg = MissionExecutor.Goal()
#         goal_msg.req = self._create_test_mission_request()
#         future = action_client.send_goal_async(goal_msg)

#         rclpy.spin_until_future_complete(rclpy_node, future, timeout_sec=5.0)
#         assert future.result() is not None
#         goal_handle = future.result()
#         assert goal_handle.accepted

#         result_future = goal_handle.get_result_async()
#         rclpy.spin_until_future_complete(rclpy_node, result_future, timeout_sec=5.0)
#         result = result_future.result().result

#         assert result.success is True
#         assert result.status_message == "Mission executed successfully"
#         assert isinstance(result.serialised_controller_feedback, ByteMultiArray)
#         print("MissionExecutor action test passed!")

#     def _create_test_mission_request(self):
#         msg = MissionRequest()
#         msg.header = Header()
#         msg.header.stamp.sec = 1
#         msg.header.stamp.nanosec = 1
#         msg.header.frame_id = 'test_frame'
#         msg.tag = "test_mission"
#         msg.vessel = Vessel()
#         msg.vessel.tag = "test_vessel"
#         msg.vessel.type = "test_type"
#         msg.init_position = Point32(x=1.0, y=1.0, z=1.0)
#         msg.goal_waypoint = Waypoint()
#         msg.goal_waypoint.position = Point32(x=5.0, y=5.0, z=0.0)
#         msg.goal_waypoint.acceptance_radius = 2.0
#         return msg

#     def test_mission_failure(self, rclpy_node, setup_action_server):
#         """Test failure case for MissionExecutor action"""
#         action_client = ActionClient(rclpy_node, MissionExecutor, 'mission_executor')
#         assert action_client.wait_for_server(timeout_sec=5.0)

#         goal_msg = MissionExecutor.Goal()
#         goal_msg.req = self._create_test_mission_request()
#         goal_msg.req.tag = "fail_mission"
#         future = action_client.send_goal_async(goal_msg)

#         rclpy.spin_until_future_complete(rclpy_node, future, timeout_sec=5.0)
#         assert future.result() is not None
#         goal_handle = future.result()
#         assert goal_handle.accepted

#         result_future = goal_handle.get_result_async()
#         rclpy.spin_until_future_complete(rclpy_node, result_future, timeout_sec=5.0)
#         result = result_future.result().result

#         assert result.success is False
#         assert result.status_message == "Mission execution failed"
#         print("MissionExecutor failure test passed!")