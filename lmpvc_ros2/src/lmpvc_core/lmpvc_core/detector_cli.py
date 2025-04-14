#!/usr/bin/env python3
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from lmpvc_interfaces.srv import Detector

class DetectorClient:
    """ROS2 client node for calling lmpvc_detector"""

    def __init__(self, node):
        self.node = node
        self.group = MutuallyExclusiveCallbackGroup()
        self.cli = self.node.create_client(Detector, 'detector', callback_group=self.group)
        self.req = Detector.Request()
        self.node.get_logger().info("Client ready!")
    
    def find(self, target: str):
        self.req.target = target

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Service not available, trying again...")

        result = self.cli.call(self.req)

        return result.pose, result.pickable, result.success


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = Node('detector_client')
    detector = DetectorClient(node)
    executor.add_node(node)

    et = threading.Thread(target=executor.spin)
    et.start()

    (pose, pickable, success) = detector.find(input("\nItem: "))
    print("\nPose:")
    print(pose)
    print("\nPickable:", pickable)
    print("Success:", success)

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()