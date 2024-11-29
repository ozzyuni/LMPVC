#!/usr/bin/env python
import argparse
import pickle
import geometry_msgs.msg
from lmpvc_detector.comms import Server, Client

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

class DetectorWebClient:
    """Provides access to the functionality of opendr_detect.py (running in ROS1) over a web interface
        THIS IS THE ROS 2 SIDE OF THE BRIDGE
    """

    def __init__(self, ip='127.0.0.1'):
        """Networking provided by comms.py, ports and server ip (to which client connects)
        can be changed here. IP is also accessible via the command line argument -ip.
        """
        self.server = Server(5009)
        self.client = Client(ip, 5008)
    
    def find(self, item):
        pose = geometry_msgs.msg.Pose()
        pickable = False
        success = False

        msg = item.encode('utf-8')
        self.client.send(msg)
        resp, success = self.server.receive()

        if success:
            data = pickle.loads(resp)
            pose = list_to_pose(data['pose'])
            pickable = data['pickable']
            success = data['success']

        return pose, pickable, success

if __name__ == '__main__':
    # This is a small test sequence
    parser = argparse.ArgumentParser()
    parser.add_argument('-ip', help="set ip address for the other end of the bridge (default = 127.0.0.1)")
    args = parser.parse_args()

    ip = args.ip
    if ip is None:
        ip = '127.0.0.1'
        print("Using default IP.")
    print("Connecting to", ip)

    detector = DetectorWebClient(ip) #130.230.37.115

    item = input("Item: ")
    pose, pickable, success = detector.find(item)
    print("Pose:")
    print(pose)
    print("Pickable:", pickable)
    print("Success:", success)


