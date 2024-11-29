from scipy.spatial.transform import Rotation as R
# BODY

def rotate(robot, axis, angle, degrees=True):
    waypoint = robot.get_pose()

    substitute = {'x': 'y', 'y': 'x', 'z': 'z'}

    #multipliers = robot.eef.rotation.apply([1.0, 1.0, 1.0])
    #print(multipliers)
    #multipliers = {'x': multipliers[0], 'y': multipliers[1], 'z': multipliers[2]}
    #angle *= -1 * multipliers[axis]

    angle *= -1
    r0 = R.from_quat([waypoint.orientation.x, waypoint.orientation.y, waypoint.orientation.z, waypoint.orientation.w])
    r1 = R.from_euler(substitute[axis], angle, degrees=degrees)
    
    r2 = r0 * r1

    quat = r2.as_quat()

    waypoint.orientation.x = quat[0]
    waypoint.orientation.y = quat[1]
    waypoint.orientation.z = quat[2]
    waypoint.orientation.w = quat[3]

    robot.add_waypoint(waypoint)
    robot.go()


# HINT
# define function: tilt 25 degrees forwards
def tilt_25_degrees_forwards(robot):
    rotate(robot, 'x', 25)
# end of function

# define function: tilt 3.5 degrees backwards
def tilt_3_point_5_degrees_backwards(robot):
    rotate(robot, 'x', -3.5)
# end of function

# define function: tilt 10 degrees to the left
def tilt_10_degrees_to_the_left(robot):
    rotate(robot, 'y', -10)
# end of function

# define function: tilt 17 degrees to the right
def tilt_17_degrees_to_the_right(robot):
    rotate(robot, 'y', 17)
# end of function

# define function: rotate a quarter turn clockwise
def rotate_a_quarter_turn_clockwise(robot):
    rotate(robot, 'z', 90)
# end of function