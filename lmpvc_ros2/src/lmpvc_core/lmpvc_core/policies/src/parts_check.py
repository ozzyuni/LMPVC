def if_you_see_a_pipe_and_a_cover_say_all_parts_found_otherwise_tell_me_whats_missing(robot):
    (pipe_pose, pipe_found) = robot.find('pipe')
    if (not pipe_found):
        robot.say("Can't find the pipe!")
    else:
        (cover_pose, cover_found) = robot.find('cover')
        if (not cover_found):
            robot.say("Can't find the cover!")
        else:
            robot.say('All parts found!')

def parts_check(robot):
    if_you_see_a_pipe_and_a_cover_say_all_parts_found_otherwise_tell_me_whats_missing(robot)
# HINT
# define function: inspect all parts
def inspect_all_parts(robot):
    parts_check(robot)
# end of function