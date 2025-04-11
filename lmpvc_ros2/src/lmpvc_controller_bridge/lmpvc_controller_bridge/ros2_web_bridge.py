#!/usr/bin/env python
import threading
import pickle
import copy
import argparse
import geometry_msgs.msg
from lmpvc_controller_bridge.comms import Server, Client

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

class ControllerROS2Bridge:
    """Provides access to the functionality of controller_client.py (running in ROS1) over a web interface
        THIS IS THE ROS 2 SIDE OF THE BRIDGE
    """

    def __init__(self, ip='127.0.0.1'):
        
        # Used for most communication
        self.server = Server(5000)
        self.client = Client(ip, 5001)
        # Used to preempt the normal communication, e.g. to stop the robot
        self.priority_server = Server(5002)
        self.priority_client = Client(ip, 5003)
        self.comms_lock = threading.Lock()
    
    def plan_waypoints(self, waypoints):
        print("\nWeb client: Planning...")
        all_sent = False

        with self.comms_lock:
            # Send instruction
            msg = pickle.dumps({'command': 'plan_waypoints', 'amount': len(waypoints)})
            self.client.send(msg)
            (resp, success) = self.server.receive(timeout=True, timeout_in_seconds=3)

            if success and resp == b'plan_waypoints':

                # Send waypoints one at a time
                for waypoint in waypoints:
                    msg = pickle.dumps(pose_to_list(waypoint))
                    self.client.send(msg)

                    (resp, success) = self.server.receive(timeout=True, timeout_in_seconds=3)
                    
                    if resp == b'continue':
                        continue
                    elif resp == b'done':
                        all_sent = True
                        break
                    else:
                        break
        
        if all_sent:
            print("Success!")
        else:
            print("Failed!")
        
        return all_sent
    
    def execute_plan(self):
        print("\nWeb client: Executing...")
        # Send instruction
        with self.comms_lock:
            msg = pickle.dumps({'command': 'execute_plan'})
            self.client.send(msg)

            (resp, success) = self.server.receive()
            
        if resp == b'success':
            print("Success!")
            return True
        else:
            print("Failed!")
            return False
    
    def get_pose(self):
        print("\nWeb client: Getting pose...")
        # Send instruction
        pose = None

        with self.comms_lock:
            msg = pickle.dumps({'command': 'get_pose'})
            self.client.send(msg)

            (resp, success) = self.server.receive(timeout=True, timeout_in_seconds=3)

            if success:
                print("Success!")
                pose = list_to_pose(pickle.loads(resp))
            else:
                print("Failed!")

        return pose
    
    def set_speed(self, speed):
        print("\nWeb client: Setting speed...")
        # Send instruction
        with self.comms_lock:
            msg = pickle.dumps({'command': 'set_speed', 'speed': speed})
            self.client.send(msg)

            (resp, success) = self.server.receive(timeout=True, timeout_in_seconds=3)
            
        if resp == b'success':
            print("Success!")
            return True
        else:
            print("Failed!")
            return False
    
    def stop(self):
        print("\nWeb client: Stopping...")
        msg = pickle.dumps({'command': 'stop'})
        self.priority_client.send(msg)

        (resp, success) = self.priority_server.receive(timeout=True, timeout_in_seconds=3)

        if resp == b'success':
            print("Success!")
            return True
        else:
            print("Failed!")
            return False
    
    def close_hand(self):
        print("\nWeb client: Closing hand...")
        # Send instruction
        with self.comms_lock:
            msg = pickle.dumps({'command': 'close_hand'})
            self.client.send(msg)

            (resp, success) = self.server.receive()
            
        if resp == b'success':
            print("Success!")
            return True
        else:
            print("Failed!")
            return False
    
    def open_hand(self):
        print("\nWeb client: Opening hand...")
        # Send instruction
        with self.comms_lock:
            msg = pickle.dumps({'command': 'open_hand'})
            self.client.send(msg)

            (resp, success) = self.server.receive()
            
        if resp == b'success':
            print("Success!")
            return True
        else:
            print("Failed!")
            return False

    def init_pose(self):
        pose = geometry_msgs.msg.Pose()
        return pose

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-ip', help="set ip address for the other end of the bridge (default = 127.0.0.1)")
    args = parser.parse_args()

    ip = args.ip
    if ip is None:
        ip = '127.0.0.1'
        print("Using default IP.")
    print("Connecting to", ip)
    
    # This is a small test sequence
    controller = ControllerROS2Bridge(ip)

    input("Press any key to get pose")
    waypoints = []
    wpose = controller.get_pose()

    input("Press any key to set speed")
    controller.set_speed(0.1)

    input("Press any key to plan carthesian path")
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z -= 0.1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z += 0.1
    waypoints.append(copy.deepcopy(wpose))

    controller.plan_waypoints(waypoints)

    input("Press any key execute plan")
    controller.execute_plan()

    input("Press any key to stop the robot")
    controller.stop()



