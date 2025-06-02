#!/usr/bin/env python

# This module implements Robot API 
#
# The RobotAPI class provides a high level API for robot control,
# in a format suitable for code generation.
#
# It also incorporates some manual configuration parameters.
import copy
import threading
import os
import json
from dataclasses import dataclass, field
from scipy.spatial.transform import Rotation as R
from pathlib import Path

from lmpvc_core.controller_cli import ControllerClient as Controller

@dataclass
class Position:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

@dataclass
class Orientation:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 0.0

# Any pose representation received from the controller API should have at least these members!
# For example, geometry_msgs.msg.Pose works. If a different internal format is required, convert
# to this one to communicate.
@dataclass
class Pose:
    position: Position = Position()
    orientation: Orientation = Orientation()

@dataclass
class Correction:
    offset: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rotation: R = field(default_factory=lambda: R.identity())

class SafetyBox:
    """Implements a rectangular safety zone defined by two points."""
    def __init__(self):
        self.xmin = [0.0, 0.0, 0.0]
        self.xmax = [0.0, 0.0, 0.0]
    
    def check(self, pose):
        """Checks that a given pose is inside the safety zone
            and limits x, y, z if necessary
        """
        position = [pose.position.x, pose.position.y, pose.position.z]
        
        violation = False
        for i in range(3):
            if position[i] < self.xmin[i]:
                position[i] = self.xmin[i]
                violation = True
            elif position [i] > self.xmax[i]:
                position[i] = self.xmax[i]
                violation = True
        
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]

        return violation

class RobotAPI:
    """RobotAPI can be thought of as a high level robot programming language primarily
        designed for code generation. It's also the most convenient way to write policies
        by hand.
    """

    def __init__(self, controller, detector, talker):

        config_path = ""
            
        try:
            from ament_index_python.packages import get_package_share_directory

            config_path = os.path.join(
                get_package_share_directory('lmpvc_core'),
                'core_config.json'
            )
        except:
            config_path = Path(__file__).with_name('core_config.json')

        config = {}

        with open(config_path, 'r') as config_file:
            config = json.load(config_file)['robot_api']

        # underlying controller which implements necessary commands on the actual robot
        self.controller = controller
        self.detector = detector
        # self.enabled should only be accessed with self.lock
        self.lock = threading.Lock()
        # robot can be prevented from moving by setting self.enaled = False
        self.enabled = True

        # text to speech engine for talking to the user
        self.voice = talker

        self.pose = self.controller.init_pose()
        self.waypoints = []
        self.speed = config['starting_speed']
        self.max_speed = config['max_speed']

        # defines the home position and world coordinate axes
        self.home = Correction()

        # some rudimentary safety limits, only for convenience during testing
        self.safety_limits = SafetyBox()
        self.safety_limits.xmin = config['safety_box']['min']
        self.safety_limits.xmax = config['safety_box']['max']

    def set_limits(self):
        """Sets new safety limits and saves them to file."""
        input("Press enter to set minimum.")
        xmin = self.get_pose(correct=False)
        print(xmin)
        input("Press enter to set maximum.")
        xmax = self.get_pose(correct=False)
        print(xmax)

        while True:
            cmd = input("Save these limits? (y/n): ")
            
            if cmd == 'n':
                print("Not saved!")
                break
            elif cmd =='y':
                config_path = Path(__file__).with_name('core_config.json')
                config = {}
                with open(config_path, 'r') as config_file:
                    config = json.load(config_file)

                self.safety_limits.xmin = xmin
                self.safety_limits.xmax = xmax
                config['robot_api']['safety_box']['min'] = xmin
                config['robot_api']['safety_box']['max'] = xmax

                with open(config_path, 'w') as config_file:
                    json.dump(config, config_file)

                print("Saved new limits!")    
                break

    
    def set_home(self):
        """Sets the home pose, and generates a new corrective rotation
            to keep directions stable
        """
        reference = self.get_pose(correct=False)

        self.home.offset = [reference.position.x, reference.position.y, reference.position.z]
        self.home.rotation = R.from_quat([reference.orientation.x,
                                            reference.orientation.y,
                                            reference.orientation.z,
                                            reference.orientation.w]) * self.eef.rotation.inv()
        
        print("Home rotation:", self.home.rotation.as_euler('xyz', degrees=True))
    
    def correct_world(self, pose, inverse=False):
        """Uses the home pose to temporarily redefine world frame to adjust relative directions
            such as left, up etc.
        """

        corrected_pose = copy.deepcopy(pose)
        position = [pose.position.x, pose.position.y, pose.position.z]

        if inverse:
            rot = self.home.rotation * R.from_quat([pose.orientation.x, pose.orientation.y,
                                pose.orientation.z, pose.orientation.w])
            position = self.home.rotation.apply(position, inverse=False)
        else:
            rot = self.home.rotation.inv() * R.from_quat([pose.orientation.x, pose.orientation.y,
                                pose.orientation.z, pose.orientation.w])
            position = self.home.rotation.apply(position, inverse=True)
        
        
        quat = rot.as_quat()

        corrected_pose.position.x = position[0]
        corrected_pose.position.y = position[1]
        corrected_pose.position.z = position[2]

        corrected_pose.orientation.x = quat[0]
        corrected_pose.orientation.y = quat[1]
        corrected_pose.orientation.z = quat[2]
        corrected_pose.orientation.w = quat[3]

        return corrected_pose

    def get_pose(self, correct=True):
        """Get current robot pose from controller"""
        self.pose = self.controller.get_pose()
        #self.waypoints[0] = self.pose

        # apply eef correction to mask the offset when
        # passing a pose outside the api
        pose = copy.deepcopy(self.pose)
        if correct:
            pose = self.correct_world(pose)
            
        return pose
    
    def stop(self, hard=False):
        """
        Stop any execution and freeze the robot in place.
        Setting hard = True also blocks any future executions.
        
        """
        if hard:
            with self.lock:
                self.enabled = False

        self.controller.stop()
    
    def enable(self):
        """Release the execution lock"""
        with self.lock:
            self.enabled = True

    def add_waypoint(self, pose, correct=True):
        """Add waypoint to local plan"""
        # undo eef correction to prepare poses for the controller
        if correct:
            pose = self.correct_world(pose, inverse=True)
        
        #if self.safety_limits.check(pose):
        #    self.say("Safety limit!")

        self.waypoints.append(pose)

    def clear_waypoints(self):
        """Clear local waypoints. Controller handles clearing it's own plan internally."""
        self.waypoints = []
    
    def go(self):
        """Instruct the controller to plan and execute the motion described by self.waypoints"""

        enabled = False
        self.controller.set_speed(self.speed)

        with self.lock:
            enabled = self.enabled

        if not enabled: # Hard fail if self.enabled = False
            return False    

        elif len(self.waypoints) < 1: # Fail if no waypoints were given
            return False
        
        else:
            if self.controller.plan_waypoints(self.waypoints): # Otherwise attempt to plan and execute the action
                self.clear_waypoints()
                return self.controller.execute_plan()
            else:
                self.clear_waypoints()
                return False
    
    def find(self, target, correct=False):
        """Find a target and return only a pose"""

        target = target.translate(str.maketrans('_', ' '))

        pose, pickable, success = self.detector.find(target)
        if not success:
            pose = self.get_pose(correct=correct)
        elif correct:
            pose = self.correct_world(pose)
            pose = self.correct_position(pose)

        return pose, success
    
    
    def find_verbose(self, target, correct=False):
        """Find a target and return the full result"""

        target = target.translate(str.maketrans('_', ' '))
        
        pose, pickable, success = self.detector.find(target)
        if not success:
            pose = self.get_pose(correct=correct)
        elif correct:
            pose = self.correct_world(pose)

        #print("\nDetection:")
        #print("Success:", success)
        #print("Pickable:", pickable)
        return pose, pickable, success
    
    def set_speed(self, speed):
        """Sets a maximum carthesian speed"""
        if 0 < speed < self.max_speed:
            self.speed = speed
        else:
            self.say("Speed value not within limits!")


    def say(self, utterance, wait=True):
        """Say something using TTS"""
        if wait:
            self.voice.say(str(utterance))
        else:
            speech_daemon = threading.Thread(target=self.say, args=[utterance],
                                             daemon=True)
            speech_daemon.start()

    
    def open_hand(self):
        """Instruct controller to open gripper"""
        success = self.controller.open_hand()
        return success

    def close_hand(self):
        """Instruct controller to close gripper"""
        success = self.controller.close_hand()
        return success

    def reset_joints(self):
        """Instruct controller to return joints to initial positions"""
        success = self.controller.reset_joints()
        return success