#!/usr/bin/env python
import json
import threading
import time
import rclpy
import geometry_msgs.msg
from pathlib import Path
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.action import ActionClient
from lmpvc_interfaces.action import Listen
from lmpvc_interfaces.srv import Talker, Detector, CodeGen, ControllerExec, ControllerGetPose, ControllerPlan, ControllerSetSpeed, ControllerStop, ControllerCloseHand, ControllerOpenHand

class ListenerActionClient():
    """Communicates with the ROS2 ActionServer implementation of Listener"""

    def __init__(self, node):
        self.node = node

        self.group = MutuallyExclusiveCallbackGroup()
        self.cli = ActionClient(node, Listen, 'listen_action', callback_group=self.group)
        self.done = threading.Event()
        self.transcript = ""
        self.node.get_logger().info("Listener: Client ready!")
    
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
            self.node.get_logger().info('Listener: Goal rejected :(')
            self.transcript = ""
            self.done.set()
            return

        self.node.get_logger().info('Listener: Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.transcript = result.transcript
        self.done.set()
    
    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info('Listener: {0}'.format(feedback.state))

class TalkerClient():

    def __init__(self, node):
        self.node = node

        self.group = MutuallyExclusiveCallbackGroup()
        self.cli = self.node.create_client(Talker, 'talker', callback_group=self.group)
        self.req = Talker.Request()
        self.node.get_logger().info("Talker: Client ready!")
    
    def say(self, utterance: str):
        self.req.utterance = utterance

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Talker: Service not available, trying again...")

        result = self.cli.call(self.req)

        if not result.success:
            self.node.get_logger().info("Talker: Something went wrong, check talker!")

class CodeGenClient():

    def __init__(self, node):
        self.node = node
        self.group = MutuallyExclusiveCallbackGroup()
        self.cli = self.node.create_client(CodeGen, 'code_gen', callback_group=self.group)
        self.req = CodeGen.Request()
        self.node.get_logger().info("CodeGen: Client ready!")
    
    def generate_inference(self, prompt: str, preamble = "", policies=""):
        self.req.prompt = prompt
        self.req.preamble = preamble
        self.req.policies = policies

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("CodeGen: Service not available, trying again...")

        self.node.get_logger().info("CodeGen: Sending request...")
        start = time.time()
        result = self.cli.call(self.req)
        end = time.time()

        self.node.get_logger().info("CodeGen: Response received in " + str(end - start) + " seconds!")
        return result.result

class ControllerClient():

    def __init__(self, node):
        self.node = node

        self._group = ReentrantCallbackGroup()
        self._close_hand_cli = self.node.create_client(ControllerCloseHand, 'controller_close_hand', callback_group=self._group)
        self._open_hand_cli = self.node.create_client(ControllerOpenHand, 'controller_open_hand', callback_group=self._group)
        self._exec_cli = self.node.create_client(ControllerExec, 'controller_exec', callback_group=self._group)
        self._get_pose_cli = self.node.create_client(ControllerGetPose, 'controller_get_pose', callback_group=self._group)
        self._plan_cli = self.node.create_client(ControllerPlan, 'controller_plan', callback_group=self._group)
        self._set_speed_cli = self.node.create_client(ControllerSetSpeed, 'controller_set_speed', callback_group=self._group)
        self._stop_cli = self.node.create_client(ControllerStop, 'controller_stop', callback_group=self._group)

        self.node.get_logger().info("Controller: Client ready!")
    
    def execute_plan(self):
        req = ControllerExec.Request()

        while not self._exec_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Controller: Service not available, trying again...")

        result = self._exec_cli.call(req)
        
        return result.success
    
    def get_pose(self):
        req = ControllerGetPose.Request()

        while not self._get_pose_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Controller: Service not available, trying again...")
        
        result = self._get_pose_cli.call(req)

        return result.pose
    
    def plan_waypoints(self, waypoints):
        req = ControllerPlan.Request()
        req.waypoints = waypoints

        while not self._plan_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Controller: Service not available, trying again...")
        
        result = self._plan_cli.call(req)

        return result.success
    
    def set_speed(self, speed):
        req = ControllerSetSpeed.Request()
        req.speed = speed

        while not self._set_speed_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Controller: Service not available, trying again...")
        
        result = self._set_speed_cli.call(req)

        return result.success
    
    def stop(self):
        req = ControllerStop.Request()

        while not self._stop_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Controller: Service not available, trying again...")
        
        result = self._stop_cli.call(req)

        return result.success
    
    def close_hand(self):
        req = ControllerCloseHand.Request()

        while not self._close_hand_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Controller: Service not available, trying again...")
        
        result = self._close_hand_cli.call(req)

        return result.success
    
    def open_hand(self):
        req = ControllerOpenHand.Request()

        while not self._open_hand_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Controller: Service not available, trying again...")
        
        result = self._open_hand_cli.call(req)

        return result.success
    
    def init_pose(self):
        pose = geometry_msgs.msg.Pose()
        return pose

class DetectorClient():

    def __init__(self, node):
        self.node = node
        self.group = MutuallyExclusiveCallbackGroup()
        self.cli = self.node.create_client(Detector, 'detector', callback_group=self.group)
        self.req = Detector.Request()
        self.node.get_logger().info("Detector: Client ready!")
    
    def find(self, target: str):
        self.req.target = target

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Detector: Service not available, trying again...")

        result = self.cli.call(self.req)

        return result.pose, result.pickable, result.success
    

class CoreNode(Node):

    def __init__(self):
        super().__init__('core')
        
        self.listener = ListenerActionClient(node=self)
        self.talker = TalkerClient(node=self)
        self.codegen = CodeGenClient(node=self)
        self.controller = ControllerClient(node=self)
        self.detector = DetectorClient(node=self)

def listener_test():
    rclpy.init()
    executor = MultiThreadedExecutor()
    core = CoreNode()
    listener = core.listener
    executor.add_node(core)

    et = threading.Thread(target=executor.spin)
    et.start()

    result = listener.listen(timeout=5.0)
    print("Result:\n")
    print(result)

    executor.shutdown()
    core.destroy_node()
    rclpy.shutdown()

def talker_test():
    rclpy.init()
    executor = MultiThreadedExecutor()
    core = CoreNode()
    voice = core.talker
    executor.add_node(core)

    et = threading.Thread(target=executor.spin)
    et.start()

    while True:
        cmd = input("Something to say ('q' to quit): ")
        if cmd == 'q':
            break
        voice.say(cmd)
    
    executor.shutdown()
    core.destroy_node()
    rclpy.shutdown()

def codegen_test():
    rclpy.init()
    executor = MultiThreadedExecutor()
    core = CoreNode()
    codegen = core.codegen
    executor.add_node(core)

    config_path = Path(__file__).with_name('core_config.json')
    config = {}

    with open(config_path, 'r') as config_file:
        config = json.load(config_file)['voice_control']

    preamble = ""
    preamble_path = Path(__file__).with_name(config['preamble_file'])

    with open(preamble_path, 'r') as preamble_file:
        preamble = preamble_file.read()

    et = threading.Thread(target=executor.spin)
    et.start()

    while True:
        cmd = input("\nInstruction (q to quit): ")
        if cmd == 'q':
            break
        result = codegen.generate_inference(cmd, preamble)
        print("Result:\n")
        print(result)

    executor.shutdown()
    core.destroy_node()
    rclpy.shutdown()

def controller_test():
    rclpy.init()
    executor = MultiThreadedExecutor()
    core = CoreNode()
    controller = core.controller

    executor.add_node(core)
    et = threading.Thread(target=executor.spin)
    et.start()

    input("Press any key to get pose")
    wpose = controller.get_pose()
    print("Result:\n")
    print(wpose)

    executor.shutdown()
    core.destroy_node()
    rclpy.shutdown()

def detector_test():
    rclpy.init()
    executor = MultiThreadedExecutor()
    core = CoreNode()
    detector = core.detector
    executor.add_node(detector)

    et = threading.Thread(target=executor.spin)
    et.start()

    (pose, pickable, success) = detector.find(input("\nItem: "))
    print("\nPose:")
    print(pose)
    print("\nPickable:", pickable)
    print("Success:", success)

    executor.shutdown()
    core.destroy_node()
    rclpy.shutdown()