def if_you_see_a_table_and_fruit_basket_say_all_done_otherwise_tell_me_whats_missing(robot):
    (table_pose, table_found) = robot.find('table')
    (fruit_basket_pose, fruit_basket_found) = robot.find('fruit basket')
    if (table_found and fruit_basket_found):
        robot.say('All done!')
    else:
        if (not table_found):
            robot.say("I can't see the table!")
        if (not fruit_basket_found):
            robot.say("I can't see the fruit basket!")

def serving_check(robot):
    if_you_see_a_table_and_fruit_basket_say_all_done_otherwise_tell_me_whats_missing(robot)
# HINT
# define function: inspect the food
def inspect_the_food(robot):
    serving_check(robot)
# end of function