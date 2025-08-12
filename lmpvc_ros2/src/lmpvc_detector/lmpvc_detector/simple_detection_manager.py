#!/usr/bin/env python

import threading

import rclpy

from rclpy.node import Node
from vision_msgs.msg import ObjectHypothesisWithPose

class DetectionManager:
    
    def __init__(self, node, verbose=True):
        self.node = node
        self.verbose = verbose

        self.node.declare_parameter('detection_topic', '/opendr/grasp_detected')
        detection_topic = self.node.get_parameter('detection_topic').value

        self.detection_sub = self.node.create_subscription(
            ObjectHypothesisWithPose,
            detection_topic,
            self.detection_cb,
            10
        )

        # Lock to prevent race condition between detection_cb and get_detection
        self._lock = threading.Lock() 

        self._detections = {}
        self.node.declare_parameter('detection_threshold', 0.0)
        self._threshold = float(self.node.get_parameter('detection_threshold').value)
        self.node.declare_parameter('detection_timeout', 3.0)
        self._detection_timeout = float(self.node.get_parameter('detection_timeout').value) * (10**9)

        self.node.get_logger().info("Detection manager ready!")

    def detection_cb(self, msg):
        if self.verbose:
            self.node.get_logger().info("Incoming detection, id: {id}".format(id=msg.hypothesis.class_id))

        with self._lock:
            if msg.hypothesis.score >= self._threshold:
                pose = msg.pose.pose
                pose.position.z = 0.0185
                self._detections[msg.hypothesis.class_id] = {'pose': pose, 'timestamp': self.node.get_clock().now().nanoseconds}

    def get_detection(self, id):

            pose = None
            with self._lock:
                data = self._detections.get(id)
                
                if data is not None:    
                    if self.node.get_clock().now().nanoseconds - data['timestamp'] < self._detection_timeout:
                        self.node.get_logger().info("Valid detection found!")
                        pose = data['pose']
                    else:
                        self.node.get_logger().info("Detection too old, discarding!")
                        self._detections.pop(id)

                else:
                    self.node.get_logger().info("No detection found!")

            return pose
             

if __name__ == '__main__':
    rclpy.init()
    node = Node('detection_manager')
    detection_manager = DetectionManager(node)
    rclpy.spin(detection_manager)
    rclpy.shutdown()