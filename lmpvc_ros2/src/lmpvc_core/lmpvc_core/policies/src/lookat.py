import numpy as np
from scipy.spatial.transform import Rotation as R
# BODY

def look_at(robot, target):
    waypoint = robot.get_pose()
    (target_pose, success) = robot.find(target, correct=False)

    if success:
        vec2 = np.array([target_pose.position.x - waypoint.position.x,
                        target_pose.position.y - waypoint.position.y,
                        target_pose.position.z - waypoint.position.z])

        rot1 = R.from_quat([waypoint.orientation.x,
                            waypoint.orientation.y,
                            waypoint.orientation.z,
                            waypoint.orientation.w])

        vec1 = np.array(rot1.apply([0.0, 0.0, 1.0], inverse=True))
        #print("Vec1:\n", vec1)
        #print("Vec1:\n", vec2)
        rot2 = R.align_vectors(np.atleast_2d(vec1), np.atleast_2d(vec2))[0]

        rot3 = rot1 * rot2

        rot3 = rot3.as_euler('XYZ')
        rot3[0] *= -1
        rot3 = R.from_euler('XYZ', rot3)    
        quat = rot3.as_quat()

        waypoint.orientation.x = quat[0]
        waypoint.orientation.y = quat[1]
        waypoint.orientation.z = quat[2]
        waypoint.orientation.w = quat[3]

        robot.add_waypoint(waypoint)
        robot.go()
    
    else:
        robot.say("could not find " + target)

# HINT
# define function: look at the dumpster
def look_at_the_dumpster(robot):
    look_at(robot, 'dumpster')
# end of function