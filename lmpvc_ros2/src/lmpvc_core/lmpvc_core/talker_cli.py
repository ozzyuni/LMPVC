#!/usr/bin/env python
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from lmpvc_interfaces.srv import Talker

class TalkerClient:
    """ROS2 action client for calling lmpvc_listener"""

    def __init__(self, node):
        self.node = node
        self.group = MutuallyExclusiveCallbackGroup()
        self.cli = self.node.create_client(Talker, 'talker', callback_group=self.group)
        self.req = Talker.Request()
        self.node.get_logger().info("[Talker] Client ready!")
    
    def say(self, utterance: str):
        self.req.utterance = utterance

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("[Talker] Service not available, trying again...")

        result = self.cli.call(self.req)

        if not result.success:
            self.node.get_logger().info("[Talker] Something went wrong, check talker!")

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = Node('talker_client')
    voice = TalkerClient(node)
    executor.add_node(node)

    et = threading.Thread(target=executor.spin)
    et.start()

    while True:
        cmd = input("Something to say ('q' to quit): ")
        if cmd == 'q':
            break
        voice.say(cmd)

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()