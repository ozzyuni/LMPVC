import copy
# BODY

def pick(robot, target):
    (target_pose, pickable, success) = robot.find_verbose(target, correct=False)
    
    if pickable:
        robot.open_hand()

        robot_pose = robot.get_pose(correct=False)

        approach_pose = copy.deepcopy(target_pose)
        approach_pose.position.z += 0.15
        approach_pose.orientation = copy.deepcopy(robot_pose.orientation)

        grasp_pose = copy.deepcopy(target_pose)
        grasp_pose.orientation = copy.deepcopy(robot_pose.orientation)

        robot.add_waypoint(approach_pose, correct=False)

        robot.add_waypoint(grasp_pose, correct=False)

        robot.go()

        robot.close_hand()

        robot_pose = robot.get_pose(correct=False)

        escape_pose = copy.deepcopy(target_pose)
        escape_pose.position.z += 0.15
        escape_pose.orientation = copy.deepcopy(robot_pose.orientation)

        robot.add_waypoint(escape_pose, correct=False)

        robot.go()

    elif success:
        robot.say("can't pick up " + target)
    else:
        robot.say("can't find " + target)

# HINT
# define function: pick up banana
def pick_up_banana(robot):
    pick(robot, 'banana')
# end of function