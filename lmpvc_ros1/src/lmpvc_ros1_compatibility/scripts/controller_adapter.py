#!/usr/bin/env python3
#
# All-inclusive script for everything needed to connect Franka Panda in ROS1
# to LMPVC in ROS2. Less annoying to use than all the separate scripts.
#
import argparse
import rospy
import sys
import threading
import pickle
import geometry_msgs.msg
from controller import MoveItController
from controller_srv import ControllerServer
from grasp_client import GraspClient
from comms import Server, Client

def pose_to_list(pose):
    """Helper function to convert pose msg to a list representation for compatibility"""
    pose_list = [pose.position.x,
                 pose.position.y,
                 pose.position.z,
                 pose.orientation.x,
                 pose.orientation.y,
                 pose.orientation.z,
                 pose.orientation.w]
    
    return pose_list

def list_to_pose(pose_list):
    """Helper function to reconstruct a pose msg from list representation"""
    pose = geometry_msgs.msg.Pose()

    pose.position.x = pose_list[0]
    pose.position.y = pose_list[1]
    pose.position.z = pose_list[2]
    pose.orientation.x = pose_list[3]
    pose.orientation.y = pose_list[4]
    pose.orientation.z = pose_list[5]
    pose.orientation.w = pose_list[6]
    
    return pose

class PriorityThread(threading.Thread):
    """Provides concurrent access via a priority server to enable ceratin actions to preempt others"""

    def __init__(self, controller, ip='127.0.0.1'):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()
        self.quit = False
        # Used to preempt the normal communication, e.g. to stop the robot
        self.server = Server(5003)
        self.client = Client(ip, 5002)
        # Pointer to the main controller
        self.controller = controller
        # Setup services for ROS1 access
        self.services = ControllerServer(self.controller)
    
    def run(self):
        """Runs when the thread is started"""
        while not rospy.is_shutdown():
            #print("Ready to receive priority messages!")
            msg = {'command': 'none'}
            (resp, success) = self.server.receive(timeout=True, timeout_in_seconds=3)
        
            if success:
                msg = pickle.loads(resp)

            if msg['command'] == 'stop':
                print("\nMessage received: stop")
                if self.controller.stop():
                    self.client.send(b'success')
                else:
                    self.client.send(b'fail')
            
            with self.lock:
                if self.quit:
                    break


class ControllerWebServer:
    """Provides access to the functionality of controller_client.py (running in ROS1) over a web interface
        THIS IS THE ROS 1 SIDE OF THE BRIDGE
    """

    def __init__(self, ip='127.0.0.1', sim=False):
        """Networking provided by comms.py, ports and server ip (to whcih client connects)
        can be changed here
        """
        # Simulation tag prevents calls to non-existent grasp action
        self.sim = sim
        # Initating the controller class
        self.controller = MoveItController()
        # Used for most communication 
        self.server = Server(5001)
        self.client = Client(ip, 5000)
        # Separate thread handles priority communication
        self.priority_thread=PriorityThread(self.controller, ip)
        self.priority_thread.start()

    def run(self):
        """Listens to messages formatted as [instruction, data] and implements appropriate behaviour"""

        try:

            while not rospy.is_shutdown():
                msg = {'command': 'none'}
                #print("Ready to receive messages!")
                (resp, success) = self.server.receive(timeout=True, timeout_in_seconds=3)
            
                if success:
                    msg = pickle.loads(resp)

                if msg['command'] == 'plan_waypoints':
                    print("\nMessage received: plan_waypoints")
                    # Confirm that the server is ready to receive waypoints
                    self.client.send(b'plan_waypoints')
                    waypoints = []

                    # Receive the number of waypoint specified by the original message
                    print("Ready to receive", msg['amount'], "waypoints.")
                    all_received = False
                    while(len(waypoints) < msg['amount']):

                        (resp, success) = self.server.receive(timeout=True, timeout_in_seconds=3)

                        if not success:
                            break
                        
                        # Use pickle to recover object (list) containing waypoint data
                        # and convert to to geometrymsgs.msg.Pose()
                        waypoint = list_to_pose(pickle.loads(resp))
                        waypoints.append(waypoint)
                        
                        # Respond with 'continue' if more waypoints are expected
                        if len(waypoints) < msg['amount']:
                            self.client.send(b'continue')
                        # Respond with 'done' if not
                        else:
                            self.client.send(b'done')
                            all_received = True
                
                    if all_received:
                        print("Succesfully received", len(waypoints), "waypoints!")
                        self.controller.plan_waypoints(waypoints)
                    
                elif msg['command'] == 'execute_plan':
                    print("\nMessage received: execute_plan")
                    if self.controller.execute_plan():
                        self.client.send(b'success')
                    else:
                        self.client.send(b'fail')
                
                elif msg['command'] == 'get_pose':
                    print("\nMessage received: get_pose")
                    pose = pose_to_list(self.controller.get_pose())
                    self.client.send(pickle.dumps(pose))
                
                elif msg['command'] == 'set_speed':
                    print("\nMessage received: set_speed")
                    if self.controller.set_speed(msg['speed']):
                        self.client.send(b'success')
                    else:
                        self.client.send(b'fail')
                
                elif msg['command'] == 'open_hand':
                    print("\nMessage received: open_hand")

                    if self.sim:
                        print("Simulation mode: Opening hand!")
                        self.client.send(b'success')
                    else:
                        hand = GraspClient()
                        hand.release()
                        self.client.send(b'success')
                
                elif msg['command'] == 'close_hand':
                    print("\nMessage received: close_hand")

                    if self.sim:
                        print("Simulation mode: Closing hand!")
                        self.client.send(b'success')
                    else:
                        hand = GraspClient()
                        hand.grasp()
                        self.client.send(b'success')

        except KeyboardInterrupt:
            print("Entered error handling!")
            with self.priority_thread.lock:
                self.priority_thread.quit = True   
            self.priority_thread.join()
            print("Keyboard interrupt!")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-ip', help="set ip address for the other end of the bridge (default = 127.0.0.1)")
    parser.add_argument('-s', help="simulation mode, grasp controller is calls are replaced with printouts", action='store_true')
    args, found = parser.parse_known_args()

    sim = args.s
    if sim:
        print("Simulation mode enabled.")

    ip = args.ip
    if ip is None:
        ip = '127.0.0.1'
        print("Using default IP.")
    print("Connecting to", ip)
    
    # Must initiate node because we use an action to control the gripper
    rospy.init_node('controller_adapter')
    bridge = ControllerWebServer(ip, sim)
    bridge.run()

    print("\nDone.")