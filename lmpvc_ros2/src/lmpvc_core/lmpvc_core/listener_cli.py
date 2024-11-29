#!/usr/bin/env python
import threading
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.action import ActionClient
from lmpvc_interfaces.action import Listen

class ListenerActionClient(Node):
    """ROS2 action client for calling lmpvc_listener"""

    def __init__(self):
        super().__init__('listen_action_client')
        self.group = MutuallyExclusiveCallbackGroup()
        self.cli = ActionClient(self, Listen, 'listen_action', callback_group=self.group)
        self.done = threading.Event()
        self.transcript = ""
        self.get_logger().info("Client ready!")
    
    def listen(self, timeout = 30.0):
        """Listens to audio for 'timeout' seconds and attempts to transcribe any detected speech.
            Emulates Listener.listen() from the local implementation.
        """
        goal_msg = Listen.Goal()
        goal_msg.timeout = timeout
        
        self.cli.wait_for_server()

        self._send_goal_future = self.cli.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        # Achieving synchronisity with a threading.Event
        self.done.wait()
        self.done.clear()

        return self.transcript
    
    def goal_response_callback(self, future):
        # Check that the goal has been accepted by the ActionServer
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.transcript = ""
            self.done.set()
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.transcript = result.transcript
        self.done.set()
    
    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Listener: {0}'.format(feedback.state))


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    listener = ListenerActionClient()
    executor.add_node(listener)

    et = threading.Thread(target=executor.spin)
    et.start()

    result = listener.listen(timeout=5.0)
    print("Result:\n")
    print(result)

    executor.shutdown()
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()