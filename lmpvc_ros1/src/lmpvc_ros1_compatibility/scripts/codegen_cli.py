#!/usr/bin/env python3

import rospy
from lmpvc_ros1_compatibility.srv import *

def generate_inference(prompt, preamble=""):
    rospy.wait_for_service('codegen')
    try:
        codegen = rospy.ServiceProxy('codegen', CodeGen)
        resp = codegen(prompt, preamble)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == '__main__':
    print("Result:", generate_inference("print hello world"))
