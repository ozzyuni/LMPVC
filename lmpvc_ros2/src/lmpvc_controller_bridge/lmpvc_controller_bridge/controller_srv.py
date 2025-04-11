#!/usr/bin/env python
import argparse

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from lmpvc_interfaces.srv import ControllerExec, ControllerGetPose, ControllerPlan, ControllerSetSpeed, ControllerStop, ControllerCloseHand, ControllerOpenHand
from lmpvc_controller_bridge.ros2_web_bridge import ControllerROS2Bridge

class ControllerService(Node):
    
    def __init__(self, controller):
        super().__init__('controller_service')
        self._group = ReentrantCallbackGroup()
        self._controller = controller

        self._exec_srv = self.create_service(ControllerExec, 'controller_exec', self.exec_cb,
                                             callback_group=self._group)
        self._get_pose_srv = self.create_service(ControllerGetPose, 'controller_get_pose',
                                                 self.get_pose_cb, callback_group=self._group)
        self._plan_srv = self.create_service(ControllerPlan, 'controller_plan', self.plan_cb,
                                             callback_group=self._group)
        self._set_speed_srv = self.create_service(ControllerSetSpeed, 'controller_set_speed', self.set_speed_cb,
                                                  callback_group=self._group)
        self._stop_srv = self.create_service(ControllerStop, 'controller_stop', self.stop_cb,
                                             callback_group=self._group)
        self._close_hand_srv = self.create_service(ControllerCloseHand, 'controller_close_hand', self.close_hand_cb,
                                             callback_group=self._group)
        self._open_hand_srv = self.create_service(ControllerOpenHand, 'controller_open_hand', self.open_hand_cb,
                                             callback_group=self._group)
        
        self.get_logger().info("Service ready!")
    
    def exec_cb(self, request, response):
        self.get_logger().info("Request received: execute_plan")
        response.success = self._controller.execute_plan()
        self.get_logger().info("Sending response: execute_plan")
        return response
    
    def get_pose_cb(self, request, response):
        self.get_logger().info("Request received: get_pose")
        response.pose = self._controller.get_pose()
        self.get_logger().info("Sending response: get_pose")
        return response
    
    def plan_cb(self, request, response):
        self.get_logger().info("Request received: plan_waypoints")
        response.success = self._controller.plan_waypoints(request.waypoints)
        self.get_logger().info("Sending response: plan_waypoints")
        return response
    
    def set_speed_cb(self, request, response):
        self.get_logger().info("Request received: set_speed")
        response.success = self._controller.set_speed(request.speed)
        self.get_logger().info("Sending response: set_speed")
        return response
    
    def stop_cb(self, request, response):
        self.get_logger().info("Request received: stop")
        response.success = self._controller.stop()
        self.get_logger().info("Sending response: stop")
        return response
    
    def close_hand_cb(self, request, response):
        self.get_logger().info("Request received: close_hand")
        response.success = self._controller.close_hand()
        self.get_logger().info("Sending response: close_hand")
        return response
    
    def open_hand_cb(self, request, response):
        self.get_logger().info("Request received: open_hand")
        response.success = self._controller.open_hand()
        self.get_logger().info("Sending response: open_hand")
        return response

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-ip', help="set ip address for the other end of the bridge (default = 127.0.0.1)")
    args, unknown = parser.parse_known_args()

    ip = args.ip
    if ip is None:
        ip = '127.0.0.1'
        print("Using default IP.")
    print("Connecting to", ip)

    controller = ControllerROS2Bridge(ip)

    rclpy.init()
    executor = MultiThreadedExecutor()
    controller_service = ControllerService(controller=controller)
    executor.add_node(controller_service)
    executor.spin()
    executor.shutdown()
    controller_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()