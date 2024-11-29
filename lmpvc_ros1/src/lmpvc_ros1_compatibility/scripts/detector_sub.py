#!/usr/bin/env python3
import rospy
import threading
from vision_msgs.msg import ObjectHypothesisWithPose

class DetectionSubscriber:

    def __init__(self, threshold=0.0):
        rospy.Subscriber('/opendr/grasp_detected', ObjectHypothesisWithPose, self.detection_cb)
        self._threshold = threshold
        # Lock to prevent race condition between detection_cb and get_detection
        self._lock = threading.Lock() 
        self._detections = {}
        self._detection_timeout = 3.0

    def detection_cb(self, data):
        with self._lock:
            if data.score >= self._threshold:
                pose = data.pose.pose
                pose.position.z = 0.0185
                self._detections[data.id] = {'pose': pose, 'timestamp': rospy.get_time()} 
            
            rospy.loginfo(rospy.get_caller_id() + "Detections updated!")

            for (id, pose) in self._detections.items():
                print(id)
                print(pose)
    
    def get_detection(self, id):

        rospy.loginfo(rospy.get_caller_id() + "Detection requested!")

        pose = None
        with self._lock:
            data = self._detections.get(id)
            
            if data is not None:    
                if rospy.get_time() - data['timestamp'] < self._detection_timeout:
                    rospy.loginfo(rospy.get_caller_id() + "Valid detection found!")
                    pose = data['pose']
                else:
                    rospy.loginfo(rospy.get_caller_id() + "Detection too old, discarding!")
                    self._detections.pop(id)

            else:
                rospy.loginfo(rospy.get_caller_id() + "No detection found!")

        return pose
             

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    listener = DetectionSubscriber()
    rospy.spin()
