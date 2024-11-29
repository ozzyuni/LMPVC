#!/usr/bin/env python
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from lmpvc_interfaces.srv import Talker

class TalkerClient(Node):
    """ROS2 action client for calling lmpvc_listener"""

    def __init__(self):
        super().__init__('talker_client')
        self.group = MutuallyExclusiveCallbackGroup()
        self.cli = self.create_client(Talker, 'talker', callback_group=self.group)
        self.req = Talker.Request()
        self.get_logger().info("Client ready!")
    
    def say(self, utterance: str):
        self.req.utterance = utterance

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, trying again...")

        result = self.cli.call(self.req)

        if not result.success:
            self.get_logger().info("Something went wrong, check talker!")

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    voice = TalkerClient()
    executor.add_node(voice)

    et = threading.Thread(target=executor.spin)
    et.start()

    while True:
        cmd = input("Something to say ('q' to quit): ")
        if cmd == 'q':
            break
        voice.say(cmd)
