#!/usr/bin/env python3

import rospy
from codegen import CodeGen as Model
from lmpvc_ros1_compatibility.srv import CodeGen, CodeGenResponse

class CodeGenServer:
    def __init__(self):
        self.model = Model()
        rospy.init_node('codegen_server')
        s = rospy.Service('codegen', CodeGen, self.codegen_cb)
        print("Service online!?")
        rospy.spin()

    def codegen_cb(self, req):
        print("Request received, generating inference..")
        result = self.model.generate_inference(req.prompt, req.preamble, log=True)
        return CodeGenResponse(result)

if __name__ == '__main__':
    codegen = CodeGenServer()

