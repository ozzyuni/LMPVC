#!/usr/bin/env python3

# A simple wrapper to launch MoveItController with ROS services enabled

import rospy
import time
import geometry_msgs.msg
from controller import MoveItController

from lmpvc_ros1_compatibility.srv import ControllerExec, ControllerGetPose, ControllerPlan, ControllerSetSpeed, ControllerStop
from lmpvc_ros1_compatibility.srv import ControllerExecResponse, ControllerGetPoseResponse, ControllerPlanResponse, ControllerSetSpeedResponse, ControllerStopResponse


class ControllerServer:
    def __init__(self, controller):
        self.controller = controller
        # Enable all supported services
        self.s1 = rospy.Service('controller_exec', ControllerExec, self.execute)
        self.s2 = rospy.Service('controller_get_pose', ControllerGetPose, self.get_pose)
        self.s3 = rospy.Service('controller_plan', ControllerPlan, self.plan)
        self.s4 = rospy.Service('controller_set_speed', ControllerSetSpeed, self.set_speed)
        self.s5 = rospy.Service('controller_stop', ControllerStop, self.stop)
    
    def execute(self, req):
        print("Executing!")
        success = self.controller.execute_plan(wait=True)
        print("Execution finished!")
        return ControllerExecResponse(success)
    
    def get_pose(self, req):
        print("Getting pose!")
        pose = self.controller.get_pose()
        return ControllerGetPoseResponse(pose)
    
    def plan(self, req):
        print("Planning!")
        success = self.controller.plan_waypoints(req.waypoints)
        return ControllerPlanResponse(success)
    
    def set_speed(self, req):
        print("Setting speed to", req.speed)
        self.controller.set_speed(req.speed)
        return ControllerSetSpeedResponse(True)
    
    def stop(self, req):
        print("Stopping!")
        self.controller.stop()
        return ControllerStopResponse(True)
    

def controller_srv():

    # Launch a MoveItController instance
    rospy.init_node('lmpvc_controller')
    controller = MoveItController()
    server = ControllerServer(controller=controller)

    # Listen to service calls
    print("Ready to accept commands.")
    rospy.spin()

if __name__ == "__main__":
    controller_srv()