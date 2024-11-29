#!/usr/bin/env python3

# This module implements a ROS1 compatibility layer for Robot API
# with service calls to corresponding server integrated with MoveIt
#
# The RobotAPI class provides a high level API for robot control,
# in a format suitable for code generation.

import copy
import rospy
import geometry_msgs.msg
from grasp_client import GraspClient
from lmpvc_ros1_compatibility.srv import ControllerExec, ControllerGetPose, ControllerPlan, ControllerSetSpeed, ControllerSetSpeed, ControllerStop
    
def plan_waypoints(waypoints):
    '''Calls a ROS service to plan trajectory to a pose goal'''

    rospy.wait_for_service('controller_plan')

    try:
        plan_waypoints_proxy = rospy.ServiceProxy('controller_plan', ControllerPlan)
        resp = plan_waypoints_proxy(waypoints)
        return resp.success
    
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False

def execute_plan():
    '''Calls a ROS service to ASYNCHRONOUSLY execute the current plan'''

    rospy.wait_for_service('controller_exec')

    try:
        execute_plan_proxy = rospy.ServiceProxy('controller_exec', ControllerExec)
        resp = execute_plan_proxy()
        return resp.success
    
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False

def get_pose():
    '''Calls a ROS service to get current pose information from MoveIt.'''

    rospy.wait_for_service('controller_get_pose')
    
    try:
        get_pose_proxy = rospy.ServiceProxy('controller_get_pose', ControllerGetPose)
        resp = get_pose_proxy()
        return resp.pose
    
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False

def set_speed(speed):
    '''Calls a ROS service to set a new speed limit for carthesian movement (m/s)'''

    rospy.wait_for_service('controller_set_speed')

    try:
        set_speed_proxy = rospy.ServiceProxy('controller_set_speed', ControllerSetSpeed)
        resp = set_speed_proxy(speed)
        return resp.success
    
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False

def init_pose():
    '''Returns a compatible Pose object for use by RobotAPI'''
    pose = geometry_msgs.msg.Pose()
    return pose

def stop():
    '''Calls a ROS service to make MoveIt immediately stop trajectory execution'''

    rospy.wait_for_service('controller_stop')

    try:
        stop_proxy = rospy.ServiceProxy('controller_stop', ControllerStop)
        resp = stop_proxy()
        return resp.success
    
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False

def open_hand():
    hand = GraspClient()
    hand.release()
    return True

def close_hand():
    hand = GraspClient()
    hand.grasp()
    return True

def test():

    input("============ Press `Enter` to plan and display a Cartesian path ...")
    waypoints = []
    wpose = get_pose()
    wpose.position.z -= 0.1  # First move up (z)
    wpose.position.y += 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    plan_waypoints(waypoints)

    input("============ Press `Enter` to execute a saved path ...")
    execute_plan()

    print("============ Demo finished!")

if __name__ == "__main__":
    test()
