#!/usr/bin/env python
import copy
import threading
import time
import rclpy
import geometry_msgs.msg
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from lmpvc_interfaces.srv import ControllerExec, ControllerGetPose, ControllerPlan, ControllerSetSpeed, ControllerStop, ControllerCloseHand, ControllerOpenHand

class ControllerClient:
    """ROS2 client node implementing services to call the functionality of lmpvc_controller."""

    def __init__(self, node):
        self._node = node
        self._group = ReentrantCallbackGroup()

        self._close_hand_cli = self._node.create_client(ControllerCloseHand, 'controller_close_hand', callback_group=self._group)
        self._open_hand_cli = self._node.create_client(ControllerOpenHand, 'controller_open_hand', callback_group=self._group)
        self._exec_cli = self._node.create_client(ControllerExec, 'controller_exec', callback_group=self._group)
        self._get_pose_cli = self._node.create_client(ControllerGetPose, 'controller_get_pose', callback_group=self._group)
        self._plan_cli = self._node.create_client(ControllerPlan, 'controller_plan', callback_group=self._group)
        self._set_speed_cli = self._node.create_client(ControllerSetSpeed, 'controller_set_speed', callback_group=self._group)
        self._stop_cli = self._node.create_client(ControllerStop, 'controller_stop', callback_group=self._group)

        self._node.get_logger().info("[Controller] Client ready!")
    
    def execute_plan(self):
        req = ControllerExec.Request()

        while not self._exec_cli.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info("[Controller] Service not available, trying again...")

        result = self._exec_cli.call(req)
        
        return result.success
    
    def get_pose(self):
        req = ControllerGetPose.Request()

        while not self._get_pose_cli.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info("[Controller] Service not available, trying again...")
        
        result = self._get_pose_cli.call(req)

        return result.pose
    
    def plan_waypoints(self, waypoints):
        req = ControllerPlan.Request()
        req.waypoints = waypoints

        while not self._plan_cli.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info("[Controller] Service not available, trying again...")
        
        result = self._plan_cli.call(req)

        return result.success
    
    def set_speed(self, speed):
        req = ControllerSetSpeed.Request()
        req.speed = speed

        while not self._set_speed_cli.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info("[Controller] Service not available, trying again...")
        
        result = self._set_speed_cli.call(req)

        return result.success
    
    def stop(self):
        req = ControllerStop.Request()

        while not self._stop_cli.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info("[Controller] Service not available, trying again...")
        
        result = self._stop_cli.call(req)

        return result.success
    
    def close_hand(self):
        req = ControllerCloseHand.Request()

        while not self._close_hand_cli.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info("[Controller] Service not available, trying again...")
        
        result = self._close_hand_cli.call(req)

        time.sleep(1.5)

        return result.success
    
    def open_hand(self):
        req = ControllerOpenHand.Request()

        while not self._open_hand_cli.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info("[Controller] Service not available, trying again...")
        
        result = self._open_hand_cli.call(req)

        time.sleep(1.5)

        return result.success
    
    def init_pose(self):
        pose = geometry_msgs.msg.Pose()
        return pose


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = Node('controller_client')
    controller = ControllerClient(node)

    executor.add_node(node)
    et = threading.Thread(target=executor.spin)
    et.start()


    input("Press any key to get pose")

    waypoints = []
    wpose = controller.get_pose()
    print("Result:\n")
    print(wpose)

    input("Press any key to set speed")
    result = controller.set_speed(0.02)
    if result:
        print("Success!")
    else:
        print("Failed!")
    
    input("Press any key to plan carthesian path")

    wpose.position.z -= 0.1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z += 0.1
    waypoints.append(copy.deepcopy(wpose))

    controller.plan_waypoints(waypoints)

    input("Press any key execute plan")
    t = threading.Thread(target=controller.execute_plan)
    t.start()

    input("Press any key to stop the robot")
    controller.stop()
    t.join()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()