def find_the_assembly_and_move_thirty_centimeters_above_it(robot):
    (assembly_pose, assembly_found) = robot.find('assembly')
    if (not assembly_found):
        robot.say("Can't find the assembly!")
    else:
        waypoint = robot.get_pose()
        waypoint.position.x = assembly_pose.position.x
        waypoint.position.y = assembly_pose.position.y
        waypoint.position.z = (assembly_pose.position.z + 0.3)
        robot.add_waypoint(waypoint)
        robot.go()

def check_parts(robot):
    parts_check(robot)

def check_bolts(robot):
    bolts_check(robot)

def full_check(robot):
    find_the_assembly_and_move_thirty_centimeters_above_it(robot)
    check_parts(robot)
    check_bolts(robot)
# HINT
# define function: do a full inspection
def do_a_full_inspection(robot):
    full_check(robot)
# end of function