#!/usr/bin/env python
import rclpy
from rclpy.node import Node
<<<<<<< HEAD
from lmpvc_interfaces.srv import Detector
from lmpvc_detector.pose_recorder import PoseRecorder
from geometry_msgs.msg import Pose
=======
from geometry_msgs.msg import Pose

from lmpvc_interfaces.srv import Detector
from lmpvc_detector.pose_recorder import PoseRecorder
from lmpvc_detector.simple_detection_manager import DetectionManager

>>>>>>> experimental

class DetectorService(Node):
    
    def __init__(self):
        super().__init__('detector_service')
<<<<<<< HEAD
=======
        self.detection_manager = DetectionManager(self)
>>>>>>> experimental
        self.pose_recorder = PoseRecorder(self)
        self.predefined_poses = self.pose_recorder.load_from_file()
        self.srv = self.create_service(Detector, 'detector', self.detector_cb)
        self.get_logger().info("Service ready!")
    
    def detector_cb(self, request, response):
        self.get_logger().info("Incoming request, trying to find: " + request.target)
        response.pose = Pose()
        response.pickable = False
        response.success = False
        
        item_data = self.predefined_poses.get(request.target)

        if item_data is not None:
<<<<<<< HEAD
            self.get_logger().info("Detection successful, returning results.")
=======
            self.get_logger().info("Pre-defined pose, returning results")
>>>>>>> experimental
            response.pose = item_data['pose']
            response.pickable = item_data['pickable']
            response.success = True
        else:
            pose = self.detection_manager.get_detection(request.target)

<<<<<<< HEAD
=======
            if pose is not None:
                self.get_logger().info("Detection successful, returning results")
                response.pose = pose
                response.pickable = True
                response.success = True
            else:    
                self.get_logger().info("Detection failed, returning results")

>>>>>>> experimental
        return response

def main():
    rclpy.init()
    detector_service = DetectorService()
    rclpy.spin(detector_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()