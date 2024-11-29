#!/usr/bin/env python3

import json
from pathlib import Path

from detector_sub import DetectionSubscriber
import pose_recorder

class OpenDRDetect:
    """Uses OpenDR based tools to implement object detection by name"""
    def __init__(self):
        data_path = Path(__file__).with_name('opendr_detection_data.json')
        self.grasp_data = {}
        self.predefined_poses = pose_recorder.load_from_file('poses.bin')
        self.detector = DetectionSubscriber()

        with open(data_path, 'r') as data_file:
            self.grasp_data = json.load(data_file)
    
    def find(self, item):
        """Detects and item in camera frame and returns an approximate pose"""
        pose = None
        pickable = False
        success = False
        predefined = False
        
        item_data = self.predefined_poses.get(item)

        if item_data is not None:
            predefined = True
        else:
            item_data = self.grasp_data.get(item)

        if item_data is not None:
            if predefined:
                pose = self.predefined_poses[item]['pose']
                pickable = self.predefined_poses[item]['pickable']
            
            else:
                pose = self.detector.get_detection(item_data['id'])
                pickable = self.grasp_data[item]['pickable']

            if pose is not None:
                success = True
                print("Found '" + item + "'!")
                print("Pose:\n", pose)
            else:
                success = False
                pickable = False
                print("Couldn't find '" + item + "'!")
        else:
            print("Unknown item!")

        return pose, pickable, success
