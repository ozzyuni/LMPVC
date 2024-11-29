#!/usr/bin/env python
import sys
import rclpy
from rclpy.node import Node

import lmpvc_interfaces.srv
import lmpvc_codegen.codegen

class CodeGenService(Node):
    """
        ROS2 Node implementing a service for calling hierarchical 
        code generation (codegen.py).
    """
    
    def __init__(self):
        super().__init__('code_gen_service')
        self.codegen = lmpvc_codegen.codegen.CodeGen()
        self.srv = self.create_service(lmpvc_interfaces.srv.CodeGen, 'code_gen', self.code_gen_cb)
        self.get_logger().info("Service ready!")
    
    def code_gen_cb(self, request, response):
        self.get_logger().info("Incoming request\nprompt:\n" + request.prompt
                               +"\npreamble \n" + request.preamble[:min(len(request.preamble), 50)]
                               + "...")
        
        # Generate a result
        result = self.codegen.generate_inference(request.prompt, request.preamble, request.policies, log=True)
        # If generation didn't return an empty string, log a success
        if result != '':
            self.get_logger().info("Generation successful!")
        else:
            self.get_logger().info("Generation failed!")

        response.result = result

        self.get_logger().info("Returning results.")
        return response

def main():
    rclpy.init()
    code_gen_service = CodeGenService()
    rclpy.spin(code_gen_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
