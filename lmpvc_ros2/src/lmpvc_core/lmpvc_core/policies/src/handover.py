import time
#BODY

def handover(robot):
    # Find predefined handover pose
    (handover_pose, success) = robot.find('handover')

    # Move to it
    robot.add_waypoint(handover_pose)
    robot.go()

    robot.say("In handover position, releasing in two seconds!")
    start = time.time()
    end = time.time()

    while(end - start < 2.0):
        time.sleep(0.1)
        end = time.time()

    robot.open_hand()

    # HINT
    # define function: give me the held item
    def give_me_the_held_item(robot):
        handover(robot)
    # end of function
