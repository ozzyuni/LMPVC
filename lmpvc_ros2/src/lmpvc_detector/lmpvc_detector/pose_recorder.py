#!/usr/bin/env python3
#
# This is a handy command line tool for manipulating predefined object poses
# Usage: rosrun lmpvc_ros1_compat pose_recorder.py -h
import argparse
import json
import os
from pathlib import Path

import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Pose

from lmpvc_core.controller_cli import ControllerClient

class PoseRecorder():

    def __init__(self, node):
        self.node = node
        self.file_path = ""
        try:
            from ament_index_python.packages import get_package_share_directory

            self.file_path = Path(os.path.join(
                get_package_share_directory('lmpvc_detector'),
                'config/test_objects.json'
                )
            )
        except:
            self.file_path = Path(__file__).with_name('test_objects.json')
        
        self.controller_cli = ControllerClient(self.node)
        self.json_data = {}

    def load_from_file(self):
        """Load saved poses from a file to memory.
            If the file does not exist, creates an empty one.
        """
        data = {}

        if not self.file_path.exists():
            with open(self.file_path, 'wb'):
                pass

        with open(self.file_path, 'rb') as file:
            try:
                self.json_data = json.load(file)
            except EOFError:
                self.json_data = {}
        
        for item in self.json_data["objects"]:
            pose = Pose()

            pose.position.x = item['pose']['position']['x']
            pose.position.y = item['pose']['position']['y']
            pose.position.z = item['pose']['position']['z']

            pose.orientation.x = item['pose']['orientation']['x']
            pose.orientation.y = item['pose']['orientation']['y']
            pose.orientation.z = item['pose']['orientation']['z']
            pose.orientation.w = item['pose']['orientation']['w']

            data[item['name']] = {'pose': pose, 'pickable': item['pickable']}

        return data

    def write_to_file(self, data):
        """Writes to saved poses to a file"""
        json_data = {'objects': []}

        print(data)

        for (name, content) in data.items():
            item = {
                    'name': name,
                    'pickable': content['pickable'],
                    'pose': {
                        'position': {
                            'x': content['pose'].position.x,
                            'y': content['pose'].position.y,
                            'z': content['pose'].position.z
                        },
                        'orientation': {
                            'x': content['pose'].orientation.x,
                            'y': content['pose'].orientation.y,
                            'z': content['pose'].orientation.z,
                            'w': content['pose'].orientation.w
                        }
                    }
                }
            json_data['objects'].append(item)

        with open(self.file_path, 'w', encoding="utf8") as file:
            json.dump(json_data, file, indent=2)

    def list_recorded(self):

        data = self.load_from_file()

        for (name, content) in data.items():
            print("\nName: " + name)
            print("Pose:")
            print(content['pose'])
            print("Pickable:", content['pickable'])

    def add_pose(self, name, pose, pickable):
        data = self.load_from_file()
        write = True

        print("\nName: " + name)
        print("Pose:")
        print(pose)
        print("Pickable:", pickable)
        
        while True:

            cmd = input("Confirm write (y/n): ")
            if cmd == 'y':
                write = True
                break
            elif cmd == 'n':
                write = False
                break
        
        if write and data.get(name) is not None:
            while True:
                cmd = input("A pose already exists for " + name + 
                            ", overwrite? (y/n): ")
                if cmd == 'y':
                    write = True
                    break
                elif cmd == 'n':
                    write = False
                    break
        
        if write:
            data[name] = {'pose': pose, 'pickable': pickable}
            self.write_to_file(data)

    def add_manually(self, name, pickable):
        """Add pose to memory by hand.
            All details must be manually typed in.
        """
        pose = Pose()
        print("Position:")
        pose.position.x = float(input("x: "))
        pose.position.y = float(input("y: "))
        pose.position.z = float(input("z: "))

        print("Orientation (quaternion):")
        pose.orientation.x = float(input("x: "))
        pose.orientation.y = float(input("y: "))
        pose.orientation.z = float(input("z: "))
        pose.orientation.w = float(input("w: "))

        self.add_pose(name, pose, pickable)

    def add_automatically(self, name, pickable):
        """Calls the contoroller service to get the current pose
            and adds it to memory. Service must be active!
        """
        pose = self.controller_cli.get_pose()
        self.add_pose(name, pose, pickable)

    def delete(self, name):
        """Removes a pose by name"""
        data = self.load_from_file()
        if data.get(name) is None:
            print("Unknown name, nothing happened!")
        else:
            while True:
                cmd = input("Are you sure you want to delete " + name + " from the database? (y/n): ")
                if cmd == 'y':
                    print("Deleted!")
                    del data[name]
                    self.write_to_file(data)
                    break
                elif cmd == 'n':
                    print("Cancelled!")
                    break
        

def main():
    
    parser = argparse.ArgumentParser()
    pgroup = parser.add_mutually_exclusive_group()

    pgroup.add_argument('-ls', '--list', help="list recorded poses", action='store_true')
    pgroup.add_argument('-a', '--auto_add', help="add pose from robot controller", action='store_true')
    pgroup.add_argument('-m', '--manual_add', help="add pose manually", action='store_true')
    parser.add_argument('-n', '--name', help="name of the object to add")
    parser.add_argument('-p', '--pickable', help="when adding a target, mark it as pickable", action='store_true') 
    pgroup.add_argument('-d', '--delete', help="delete pose from database", action='store_true')

    rclpy.init()
    executor = MultiThreadedExecutor()
    node = Node('pose_recorder')
    args = parser.parse_args()

    executor.add_node(node)
    et = threading.Thread(target=executor.spin)
    et.start()

    pose_recorder = PoseRecorder(node)

    print("Pose Recorder Started!")

    if args.list:
        print("Listing recorded poses...")
        pose_recorder.list_recorded()
    elif args.auto_add:
        print("Adding pose automatically from robot controller...")
        if args.name is not None:
            pose_recorder.add_automatically(args.name, args.pickable)
        else:
            print("You must specify a name with -n!")
    elif args.manual_add:
        print("Adding pose manually...")
        if args.name is not None:
            pose_recorder.add_manually(args.name, args.pickable)
        else:
            print("You must specify a name with -n!")
    elif args.delete is not None:
        print("Deleting pose...")
        if args.name is not None:
            pose_recorder.delete(args.name)
        else:
            print("You must specify a name with -n!")

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    print("\nExiting.")

if __name__ == '__main__':
    main()