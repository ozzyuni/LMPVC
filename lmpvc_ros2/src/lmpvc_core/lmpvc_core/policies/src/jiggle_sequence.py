def move_five_centimeters_to_the_left(robot):
    waypoint = robot.get_pose()
    waypoint.position.y -= 0.05
    robot.add_waypoint(waypoint)
    robot.go()

def move_ten_centimeters_to_the_right(robot):
    waypoint = robot.get_pose()
    waypoint.position.y += 0.1
    robot.add_waypoint(waypoint)
    robot.go()

def move_five_centimeters_to_the_left(robot):
    waypoint = robot.get_pose()
    waypoint.position.y -= 0.05
    robot.add_waypoint(waypoint)
    robot.go()

def jiggle_sequence(robot):
    move_five_centimeters_to_the_left(robot)
    move_ten_centimeters_to_the_right(robot)
    move_five_centimeters_to_the_left(robot)
# HINT
# define function: perform jiggling
def perform_jiggling(robot):
    jiggle_sequence(robot)
# end of function