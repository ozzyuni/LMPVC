#!/usr/bin/env python3

# This is a simple LMPVC compatible robot controller implemented with MoveIt
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

GROUP_NAME = "panda_manipulator"

class MoveItController:
    """MoveItController"""

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        group_name = GROUP_NAME

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.eef_link = self.move_group.get_end_effector_link()
        self.plan = moveit_msgs.msg.RobotTrajectory

        self.move_group.set_max_velocity_scaling_factor(0.25)

        print("============ Found robot:")
        print(self.robot.get_current_state())
        print("")
    
    def execute_plan(self, wait=True):

        self.move_group.stop()
        self.move_group.execute(self.plan, wait=wait)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        return True
    
    def get_pose(self):

        pose = self.move_group.get_current_pose().pose
        return pose
    
    def plan_waypoints(self, waypoints, scale=1):

        self.move_group.stop()
        self.move_group.clear_pose_targets()

        for wpose in waypoints:
            wpose.position.x *= scale
            wpose.position.y *= scale
            wpose.position.z *= scale

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01)  # waypoints to follow, eef_step

        self.plan=plan
        return True
    
    def set_speed(self, speed):
        self.move_group.limit_max_cartesian_link_speed(speed, self.eef_link)
        return True
    
    def stop(self):
        self.move_group.stop()
        return True


def test():
    try:
        controller = MoveItController()

        offsets = [0.15, -0.3, 0.3, -0.3, 0.3, -0.15]

        input("============ Press `Enter` to start test ...")
        
        for i in range(len(offsets)):
            waypoints = []
            speed = 0.5 / (2*i + 1)

            print('Now moving at:', speed, 'm/s')

            controller.set_speed(speed)
            wpose = controller.get_pose()
            waypoints.append(copy.deepcopy(wpose))
            wpose.position.y += offsets[i]
            waypoints.append(copy.deepcopy(wpose))

            controller.plan_waypoints(waypoints)
            controller.execute_plan()

        print("============ Test complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    rospy.init_node("controller_node")
    controller = MoveItController()
    input("Press Enter to get pose")
    print("Pose:")
    print(controller.get_pose())
    test()
