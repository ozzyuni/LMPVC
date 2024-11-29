#!/usr/bin/env python3
import tf
import copy
import rospy
from geometry_msgs.msg import PoseStamped

class PoseConversion:
    """TF based pose conversion class with configurable frames"""
    def __init__(self):
        self._tf_listener = tf.TransformListener()
    
    def convert(self, pose, source_frame, target_frame):
        self._tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(3.0))
        old = PoseStamped()
        old.header.frame_id = source_frame
        old.header.stamp = rospy.Time()
        old.pose = copy.deepcopy(pose)

        new = self._tf_listener.transformPose(target_frame, old)

        return new.pose