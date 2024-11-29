def if_you_find_a_bolt_hole_say_missing_bolts_otherwise_say_everything_secured(robot):
    (bolt_hole_pose, bolt_hole_found) = robot.find('bolt hole')
    if bolt_hole_found:
        robot.say('missing bolts')
    else:
        robot.say('everything secured')

def bolts_check(robot):
    if_you_find_a_bolt_hole_say_missing_bolts_otherwise_say_everything_secured(robot)
# HINT
# define function: inspect the bolts
def inspect_the_bolts(robot):
    bolts_check(robot)
# end of function