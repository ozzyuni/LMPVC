import copy
# BODY

def place(robot, target):
    (target_pose, pickable, success) = robot.find_verbose(target, correct=False)
    
    if success:

        robot_pose = robot.get_pose(correct=False)

        approach_pose = copy.deepcopy(target_pose)
        approach_pose.position.z += 0.15
        approach_pose.orientation = copy.deepcopy(robot_pose.orientation)

        release_pose = copy.deepcopy(target_pose)
        release_pose.orientation = copy.deepcopy(robot_pose.orientation)

        robot.add_waypoint(approach_pose, correct=False)

        robot.add_waypoint(release_pose, correct=False)

        robot.go()

        robot.open_hand()

        robot_pose = robot.get_pose(correct=False)

        escape_pose = copy.deepcopy(target_pose)
        escape_pose.position.z += 0.15
        escape_pose.orientation = copy.deepcopy(robot_pose.orientation)

        robot.add_waypoint(escape_pose, correct=False)

        robot.go()

    else:
        robot.say("can't find " + target)

# HINT
# define function: place on the table
def place_on_the_table(robot):
    place(robot, 'table')
# end of function

# define function: place it in the box
def place_it_in_the_box(robot):
    place(robot, 'box')
# end of function