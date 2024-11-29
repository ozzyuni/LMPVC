#!/usr/bin/env python
import rclpy
import json
from rclpy.node import Node
from pathlib import Path

import lmpvc_interfaces.srv

config_path = Path(__file__).with_name('talker_config.json')
config = {}
with open(config_path, 'r') as config_file:
    config = json.load(config_file)

if config['engine'] == 'simple':
    from lmpvc_talker.simple_tts import Talker

elif config['engine'] == 'piper':
    from lmpvc_talker.piper_tts import Talker

class TalkerService(Node):
    
    def __init__(self):
        super().__init__('talker_service')
        self.voice = Talker()
        self.srv = self.create_service(lmpvc_interfaces.srv.Talker, 'talker', self.code_gen_cb)
        self.get_logger().info("Service ready!")
    
    def code_gen_cb(self, request, response):
        self.get_logger().info("Request received!")
        
        result = self.voice.say(request.utterance)

        if result:
            self.get_logger().info("Speech successful!")
        else:
            self.get_logger().info("Speech failed!")

        response.success = result

        self.get_logger().info("Returning result.")
        return response

def main():
    rclpy.init()
    code_gen_service = TalkerService()
    rclpy.spin(code_gen_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()