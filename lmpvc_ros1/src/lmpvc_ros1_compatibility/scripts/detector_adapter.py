#!/usr/bin/env python3
import argparse
import sys
import rospy
import pickle
import geometry_msgs.msg
#import detector_cli as detector
from opendr_detection import OpenDRDetect
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

class DetectorWebServer:
    """Provides access to the functionality of controller_client.py (running in ROS1) over a web interface
        THIS IS THE ROS 1 SIDE OF THE BRIDGE
    """

    def __init__(self, ip = '127.0.0.1'):
        """Networking provided by comms.py, ports and server ip (to whcih client connects)
        can be changed here
        """
        # Used for most communication 
        self.server = Server(5008)
        self.client = Client(ip, 5009)
        self.detector = OpenDRDetect()
    
    def run(self):
        """Listens to messages formatted as [instruction, data] and implements appropriate behaviour"""

        try:

            while not rospy.is_shutdown():
                (resp, success) = self.server.receive(timeout=True, timeout_in_seconds=3)
            
                if success:
                    item = resp.decode('utf-8')
                
                    print("\nMessage received, getting detections for: '" + item + "'!")
                    (pose, pickable, success) = self.detector.find(item)

                    if success:
                        pose = pose_to_list(pose)
                    else:
                        pose = pose_to_list(geometry_msgs.msg.Pose())

                    msg = pickle.dumps({'pose': pose, 'pickable': pickable, 'success': success})
                    print("Sending response!")
                    self.client.send(msg)

        except KeyboardInterrupt:
            print("Keyboard interrupt!")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-ip', help="set ip address for the other end of the bridge (default = 127.0.0.1)")
    args, found = parser.parse_known_args()

    ip = args.ip
    if ip is None:
        ip = '127.0.0.1'
        print("Using default IP.")
    print("Connecting to", ip)

    rospy.init_node('detector_adapter')

    bridge = DetectorWebServer(ip=ip)
    bridge.run()