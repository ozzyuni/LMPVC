#!/usr/bin/env python

# This is a ROS2 launcher for the LMPVC suite
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


from lmpvc_core.robot import RobotAPI as Robot
from lmpvc_core.codegen_cli import CodeGenClient
from lmpvc_core.controller_cli import ControllerClient
from lmpvc_core.detector_cli import DetectorClient
from lmpvc_core.listener_cli import ListenerActionClient
from lmpvc_core.talker_cli import TalkerClient
from lmpvc_core.voice_control import VoiceControl

def main():
    """Acts as a ROS2 wrapper for voice_control, the main LMPVC application.
        Sets up node, messaging and the executor used to run everything. This makes
        it possible to pass multiple independent clients to the main
        application.
    """

    rclpy.init()
    executor = MultiThreadedExecutor()
    node = Node('lmpvc_core')

    codegen = CodeGenClient(node)
    controller = ControllerClient(node)
    detector = DetectorClient(node)
    talker = TalkerClient(node)
    listener = ListenerActionClient(node)

    executor.add_node(node)
    et = threading.Thread(target=executor.spin)
    et.start()

    robot = Robot(controller, detector, talker)
    vc = VoiceControl(codegen, robot, listener)
    vc.interface()

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    print("\nDone.")

if __name__ == '__main__':
    main()