#!/usr/bin/env python

# This is a ROS2 launcher for the LMPVC suite
import rclpy
import threading
from pathlib import Path
from rclpy.executors import MultiThreadedExecutor


from lmpvc_core.robot import RobotAPI as Robot
from lmpvc_core.ros2_cli import CoreNode
from lmpvc_core.voice_control import VoiceControl

def main():
    """Acts as a ROS2 wrapper for voice_control, the main LMPVC application.
        Sets up client nodes and the executor used to run them. This makes
        it possible to pass multiple independent clients to the main
        application.
    """

    rclpy.init()
    executor = MultiThreadedExecutor()

    core = CoreNode()

    codegen = core.codegen
    controller = core.controller
    detector = core.detector
    talker = core.talker
    listener = core.listener

    executor.add_node(core)
    et = threading.Thread(target=executor.spin)
    et.start()

    robot = Robot(controller, detector, talker)
    vc = VoiceControl(codegen, robot, listener)
    vc.interface()

    executor.shutdown()
    core.destroy_node()
    rclpy.shutdown()
    print("\nDone.")

if __name__ == '__main__':
    main()