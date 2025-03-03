# test_agent_update.py
import pytest
import rclpy
from colav_interfaces.msg import AgentUpdate
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

def test_math():
    assert 1+1 == 2

# class AgentUpdatePublisher(Node):
#     def __init__(self):
#         super().__init__('agent_update_publisher')
#         self.publisher = self.create_publisher(AgentUpdate, 'agent_update_topic', 10)
#         self.timer = self.create_timer(1.0, self.publish_agent_update)  # Publish every 1 second

#     def publish_agent_update(self):
#         msg = AgentUpdate()
#         msg.header = Header()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = "map"  # Example frame_id

#         msg.mission_tag = "mission_001"
#         msg.agent_tag = "agent_007"

#         msg.pose = Pose()
        # msg.pose.position.x = 10.0
        # msg.pose.position.y = 5.0
        # msg.pose.position.z = 0.0
        # msg.pose.orientation.x = 0.0
        # msg.pose.orientation.y = 0.0
        # msg.pose.orientation.z = 0.0
        # msg.pose.orientation.w = 1.0

#         msg.velocity = 5.0
#         msg.acceleration = 2.0
#         msg.yaw_rate = 0.1

#         self.publisher.publish(msg)
#         self.get_logger().info(f'Publishing AgentUpdate: {msg}')

# class AgentUpdateSubscriber(Node):
#     def __init__(self):
#         super().__init__('agent_update_subscriber')
#         self.subscription = self.create_subscription(
#             AgentUpdate,
#             'agent_update_topic',
#             self.listener_callback,
#             10)

#     def listener_callback(self, msg):
#         self.get_logger().info(f'Received AgentUpdate: mission_tag={msg.mission_tag}, agent_tag={msg.agent_tag}')
#         self.received_message = msg


# @pytest.fixture(scope='module')
# def ros2_nodes():
#     # Initialize ROS2 context
#     rclpy.init()

#     # Create publisher and subscriber nodes
#     publisher = AgentUpdatePublisher()
#     subscriber = AgentUpdateSubscriber()

#     # Create a single-threaded executor to handle callbacks
#     executor = SingleThreadedExecutor()
#     executor.add_node(publisher)
#     executor.add_node(subscriber)

#     executor.spin_once(timeout_sec=0.1)
#     executor_thread = executor.spin_once(timeout_sec=0.1)

#     yield publisher, subscriber, executor

#     # Clean up
#     publisher.destroy_node()
#     subscriber.destroy_node()
#     rclpy.shutdown()

#     _, subscriber, executor = ros2_nodes
#     publisher, subscriber, executor = ros2_nodes
    
#     # Allow time for the message to be published and received
#     for _ in range(3):
#         executor.spin_once(timeout_sec=1.0)  # Wait for messages (real test may want better synchronization)
    
#     # Check that the subscriber received the message with the expected content
#     # The actual validation logic depends on what you need to assert
    
#     received_message = subscriber.received_message  # Access the message received in the callback
#     received_message = subscriber.listener_callback.call_args[0][0]  # Access the message received in the callback

#     assert received_message.mission_tag == "mission_001"
#     assert received_message.agent_tag == "agent_007"
#     assert received_message.velocity == 5.0
#     assert received_message.acceleration == 2.0
#     assert received_message.yaw_rate == 0.1

#     # You can add more assertions based on the pose or other fields as needed
