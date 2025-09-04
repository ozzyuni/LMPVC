#!/usr/bin/env python
import os
import pickle
import copy
import argparse
import threading
import geometry_msgs.msg
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from lmpvc_codegen.comms import Server, Client

class CodeGenWebClient:
    """Provides access to CodeGen over a tcp connection. Mostly useful for prototyping with a 
    remote GPU machine. NOT SECURE, so use e.g. SSH tunneling for access outside LAN.
    """

    def __init__(self, node=None, ip='127.0.0.1'):
        
        # Used for most communication
        self.node = node
        self.client = Client(ip, 5001)
    
    def log_info(self, msg):
        if self.node is not None:
            self.node.get_logger().info(msg)
        else:
            print("INFO: " + msg)

    def log_error(self, msg):
        if self.node is not None:
            self.node.get_logger().error(msg)
        else:
            print("ERROR: " + msg)
    
    def generate_inference(self, prompt, preamble='', policies=''):
        self.log_info("Web client: Requesting inference")
        # Send instruction
        msg = pickle.dumps({'prompt': prompt, 'preamble': preamble, 'policies': policies})
        (resp, success) = self.client.send(msg, timeout=True, timeout_in_seconds=15)

        if success:
            self.log_info("Response received")
            return resp.decode('utf-8')
        else:
            self.log_error("Did not receive a response from generation")
            return ""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-ip', help="set ip address for the other end of the bridge (default = 127.0.0.1)")
    args = parser.parse_args()

    ip = args.ip
    if ip is None:
        ip = '127.0.0.1'
        print("Using default IP.")
    print("Connecting to", ip)

    rclpy.init()
    executor = MultiThreadedExecutor()
    node = Node('codegen_client')

    bridge = CodeGenWebClient(node=node, ip=ip)

    executor.add_node(node)
    et = threading.Thread(target=executor.spin)
    et.start()

    bridge.log_info("Connecting to " + ip)

    prompt = input("Prompt: ")

    preamble = ""
    preamble_path = ""
    
    try:
        from ament_index_python.packages import get_package_share_directory

        preamble_path = os.path.join(
            get_package_share_directory('lmpvc_core'),
            'preamble.py'
        )
    except:
        preamble_path = Path(__file__).with_name('preamble.py')

    with open(preamble_path, 'r') as preamble_file:
        preamble = preamble_file.read()
    
    code = bridge.generate_inference(prompt, preamble)

    bridge.log_info("[CODE]:\n" + code)

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

