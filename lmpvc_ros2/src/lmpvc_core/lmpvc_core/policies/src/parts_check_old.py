def if_you_see_a_pipe_and_a_cover_say_all_done_otherwise_tell_me_whats_missing(robot):
    (pipe_pose, pipe_found) = robot.find('pipe')
    (cover_pose, cover_found) = robot.find('cover')
    if (pipe_found and cover_found):
        robot.say('All done!')
    elif pipe_found:
        robot.say("I can't find the cover!")
    elif cover_found:
        robot.say("I can't find the pipe!")
    else:
        robot.say("I can't find the pipe or the cover!")

def parts_check(robot):
    if_you_see_a_pipe_and_a_cover_say_all_done_otherwise_tell_me_whats_missing(robot)
# HINT
# define function: check that all the parts are installed
def check_that_all_the_parts_are_installed(robot):
    parts_check(robot)
# end of function