#!/usr/bin/env python3
import franka_gripper.msg
import rospy
import sys
import actionlib

class GraspClient:
    def __init__(self):
        self.grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)
        self.release_client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)
        self.stop_client = actionlib.SimpleActionClient('/franka_gripper/stop', franka_gripper.msg.StopAction)
    
    def grasp(self, width = 0.04, force=5.0, epsilon_multiplier = 0.95):
        self.grasp_client.wait_for_server()

        goal = franka_gripper.msg.GraspGoal()
        goal.width = width
        goal.epsilon.inner = width * epsilon_multiplier
        goal.epsilon.outer = width * epsilon_multiplier
        goal.speed = 0.03
        goal.force = force

        self.grasp_client.send_goal(goal)
        self.grasp_client.wait_for_result()

        return self.grasp_client.get_result()
    
    def release(self):
        self.stop_client.wait_for_server()
        goal = franka_gripper.msg.StopGoal()
        self.stop_client.send_goal(goal)
        self.stop_client.wait_for_result()

        self.release_client.wait_for_server()
        goal = franka_gripper.msg.MoveGoal()
        goal.speed = 0.03
        goal.width = 10.0
        self.release_client.send_goal(goal)
        self.release_client.wait_for_result()

        return self.release_client.get_result()
    
if __name__ == '__main__':
    try:
        gripper = GraspClient()
        rospy.init_node('grasp_client')
        input("Press enter to Grasp!")
        gripper.grasp(0.04, 10.0)
        input("Press enter to Release!")
        gripper.release()
    except rospy.ROSInterruptException as e:
        print("ROS Exception:", e)