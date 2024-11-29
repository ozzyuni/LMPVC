#!/usr/bin/env python3
#
# This is a handy command line tool for manipulating predefined object poses
# Usage: rosrun lmpvc_ros1_compat pose_recorder.py -h
import rospy
import argparse
import pickle
from pathlib import Path

import geometry_msgs.msg
import controller_cli

POSEFILE = 'poses.bin'

def load_from_file(filename):
    """Load saved poses from a file to memory.
        If the file does not exist, creates an empty one.
    """
    data = {}
    filepath = Path(__file__).with_name(filename)

    if not filepath.exists():
        with open(filepath, 'wb'):
            pass

    with open(filepath, 'rb') as file:
        try:
            data = pickle.load(file)
        except EOFError:
            data = {}
    
    return data

def write_to_file(data, filename):
    """Writes to saved poses to a file"""
    filepath = Path(__file__).with_name(filename)
    with open(filepath, 'wb') as file:
        pickle.dump(data, file)

def list_recorded():

    data = load_from_file(POSEFILE)

    for (name, content) in data.items():
        print("\nName: " + name)
        print("Pose:")
        print(content['pose'])
        print("Pickable:", content['pickable'])

def add_pose(name, pose, pickable):
    data = load_from_file(POSEFILE)
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
        write_to_file(data, POSEFILE)

def add_manually(name, pickable):
    """Add pose to memory by hand.
        All details must be manually typed in.
    """
    pose = geometry_msgs.msg.Pose()
    print("Position:")
    pose.position.x = float(input("x: "))
    pose.position.y = float(input("y: "))
    pose.position.z = float(input("z: "))

    print("Orientation (quaternion):")
    pose.orientation.x = float(input("x: "))
    pose.orientation.y = float(input("y: "))
    pose.orientation.z = float(input("z: "))
    pose.orientation.w = float(input("w: "))

    add_pose(name, pose, pickable)

def add_automatically(name, pickable):
    """Calls the contoroller service to get the current pose
        and adds it to memory. Service must be active!
    """
    pose = controller_cli.get_pose()
    add_pose(name, pose, pickable)

def delete(name):
    """Removes a pose by name"""
    data = load_from_file(POSEFILE)
    if data.get(name) is None:
        print("Unknown name, nothing happened!")
    else:
        while True:
            cmd = input("Are you sure you want to delete " + name + " from the database? (y/n): ")
            if cmd == 'y':
                print("Deleted!")
                del data[name]
                write_to_file(data, POSEFILE)
                break
            elif cmd == 'n':
                print("Cancelled!")
                break
        


if __name__ == '__main__':
    rospy.init_node('pose_recorder', anonymous=True)
    
    parser = argparse.ArgumentParser()
    pgroup = parser.add_mutually_exclusive_group()

    pgroup.add_argument('-ls', '--list', help="list recorded poses", action='store_true')
    pgroup.add_argument('-a', '--auto_add', help="add pose from robot controller", action='store_true')
    pgroup.add_argument('-m', '--manual_add', help="add pose manually", action='store_true')
    parser.add_argument('-n', '--name', help="name of the object to add")
    parser.add_argument('-p', '--pickable', help="when adding a target, mark it as pickable", action='store_true') 
    pgroup.add_argument('-d', '--delete', help="delete pose from database", action='store_true')

    args = parser.parse_args()

    print("Pose Recorder Started!")

    if args.list:
        print("Listing recorded poses...")
        list_recorded()
    elif args.auto_add:
        print("Adding pose automatically from robot controller...")
        if args.name is not None:
            add_automatically(args.name, args.pickable)
        else:
            print("You must specify a name with -n!")
    elif args.manual_add:
        print("Adding pose manually...")
        if args.name is not None:
            add_manually(args.name, args.pickable)
        else:
            print("You must specify a name with -n!")
    elif args.delete is not None:
        print("Deleting pose...")
        if args.name is not None:
            delete(args.name)
        else:
            print("You must specify a name with -n!")
    
