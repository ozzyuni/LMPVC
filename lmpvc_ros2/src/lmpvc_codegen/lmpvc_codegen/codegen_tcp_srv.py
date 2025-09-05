#!/usr/bin/env python3
import sys
import pickle
import rclpy
import threading
import argparse
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from lmpvc_codegen.codegen import CodeGen
from lmpvc_codegen.comms import Server, Client

class CodeGenWebServer:
    """Provides access to CodeGen over a tcp connection. Mostly useful for prototyping with a 
    remote GPU machine. NOT SECURE, so use e.g. SSH tunneling for access outside LAN.
    """

    def __init__(self, node=None, ip = '127.0.0.1'):
        """Networking provided by comms.py, ports and server ip (to whcih client connects)
        can be changed here
        """
        # Used for most communication
        self.node = node
        self.server = Server(5001)
        self.codegen = CodeGen()
    
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

    def run(self):
        """Listens to messages formatted as [instruction, data] and implements appropriate behaviour"""

        try:

            while True:
                (resp, success) = self.server.receive(timeout=True, timeout_in_seconds=3)
            
                if success:
                    msg = pickle.loads(resp)
                
                    self.log_info("\nMessage received, generating inference...")
                    result = self.codegen.generate_inference(msg['prompt'], msg['preamble'], msg['policies'], log=True)
                    self.log_info("Sending response")
                    self.server.respond(result.encode('utf-8'))

        except KeyboardInterrupt:
            self.log_error("Keyboard interrupt")

def main():
    rclpy.init()

    node = Node('codegen_web_server')

    parser = argparse.ArgumentParser()
    parser.add_argument('-ip', help="set ip address for the other end of the bridge (default = 127.0.0.1)")
    args, unknown = parser.parse_known_args()

    ip = args.ip
    if ip is None:
        ip = '127.0.0.1'
        print("Using default IP.")
    print("Connecting to", ip)

    bridge = CodeGenWebServer(node=node, ip=ip)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    et = threading.Thread(target=executor.spin)

    bridge.log_info("Connecting to " + ip)
    bridge.run()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
