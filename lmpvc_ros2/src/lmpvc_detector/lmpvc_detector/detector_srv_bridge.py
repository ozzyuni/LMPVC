#!/usr/bin/env python
import rclpy
import argparse
from rclpy.node import Node
from lmpvc_interfaces.srv import Detector

import lmpvc_detector.detector_web_client

class DetectorService(Node):
    
    def __init__(self, ip='127.0.0.1'):
        super().__init__('detector_service')
        self.detector = lmpvc_detector.detector_web_client.DetectorWebClient(ip)
        self.srv = self.create_service(Detector, 'detector', self.detector_cb)
        self.get_logger().info("Service ready!")
    
    def detector_cb(self, request, response):
        self.get_logger().info("Incoming request, trying to find: " + request.target)

        (pose, pickable, success) = self.detector.find(request.target)

        if success:
            self.get_logger().info("Detection successful, returning results.")
        else:
            self.get_logger().info("Detection failed, returning results.")

        response.pose = pose
        response.pickable = pickable
        response.success = success

        return response

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-ip', help="set ip address for the other end of the bridge (default = 127.0.0.1)")
    args, unknown = parser.parse_known_args()

    ip = args.ip
    if ip is None:
        ip = '127.0.0.1'
        print("Using default IP.")
    print("Connecting to", ip)

    rclpy.init()
    detector_service = DetectorService(ip)
    rclpy.spin(detector_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()